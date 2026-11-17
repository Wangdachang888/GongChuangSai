/**
 * @file    app_imu.c
 * @brief   IMU 应用层封装实现（F407 + HAL + 麦轮底盘）
 *
 * 设计要点：
 *   1. 只在主循环里轮询，绝不放在 TIM6 的 1ms 中断里（阻塞 I2C 会打乱控制节拍）
 *   2. 200Hz 更新姿态，底盘 10ms 环直接读 app_imu_yaw_rad()
 *   3. IMU 掉线（连续 20 帧读失败）自动降级：app_imu_ok() 返回 false，
 *      底盘回到纯里程计航向；恢复后自动重新接管
 */
#include "app_imu.h"

#include <string.h>
#include <math.h>

#include "main.h"

#include "icm20948.h"
#include "imu_attitude.h"
#include "imu_port_stm32_hal.h"

/* ------------------------------------------------------------------------- */
/* 配置                                                                      */
/* ------------------------------------------------------------------------- */

/* 硬件 I2C 句柄。
 * 本板 ICM20948 接在 PB10/PB11（普通 IO，软件 I2C），工程里没有启用 I2C 外设，
 * 此时 HAL 不会定义 I2C_HandleTypeDef，因此必须按模块开关区分，否则编译报
 * "identifier I2C_HandleTypeDef is undefined"。
 * 以后改用硬件 I2C 时：在 CubeMX 里启用 I2C 并生成代码，这里会自动切过去。
 */
#ifndef APP_IMU_I2C_HANDLE
#  if defined(HAL_I2C_MODULE_ENABLED)
extern I2C_HandleTypeDef hi2c1;
#    define APP_IMU_I2C_HANDLE      (&hi2c1)
#  else
#    define APP_IMU_I2C_HANDLE      NULL
#  endif
#endif

/* 采样率：200Hz 轮询 → ODR 设到 225Hz 左右，保证每次读到的都是新数据 */
#define APP_IMU_SMPLRT_DIV      4u

/* 连续失败多少帧后判定 IMU 掉线 */
#define APP_IMU_FAIL_LIMIT      20u

/* 是否启用磁力计参与航向：0 = 只用陀螺（室内推荐），1 = 陀螺+磁力计 */
#ifndef APP_IMU_USE_MAG
#define APP_IMU_USE_MAG         0
#endif

/* ------------------------------------------------------------------------- */
/* 内部状态                                                                  */
/* ------------------------------------------------------------------------- */

static icm20948_t     s_imu;
static imu_attitude_t s_att;

static bool     s_inited;
static bool     s_ok;
static uint32_t s_last_tick;
static uint32_t s_frames;
static uint32_t s_fail_streak;
static float    s_last_dt;
/* ------------------------------------------------------------------------- */
/* 零漂治理：附加零偏层                                                      */
/*   1) 上电 10s 后，用 6 个 1 秒块的中位数做一次细化标定（块中位数抗突跳）   */
/*   2) 运行中检测到静止，用很长的时间常数慢速跟踪零偏（补偿温漂）           */
/* ------------------------------------------------------------------------- */
#define IMU_WARMUP_MS       10000u   /* 上电预热：等温度稳定再标定 */
#define IMU_CAL_BLOCKS      6u       /* 标定块数（每块约 1 秒） */
#define IMU_CAL_BLOCK_MS    1000u
#define IMU_STILL_GYRO_DPS  2.0f     /* 静止判据：三轴角速度 */
#define IMU_STILL_ACC_G     0.10f    /* 静止判据：加速度模长偏差 */
#define IMU_STILL_SETTLE_MS 500u     /* 连续静止多久才开始慢更新 */
#define IMU_BIAS_TAU_S      10.0f    /* 慢更新时间常数（越大越保守） */

static float    s_bias[3];                          /* 附加零偏 dps */
static uint8_t  s_cal_state;                        /* 0=等预热 1=标定中 2=运行 */
static float    s_cal_sum[3];
static uint16_t s_cal_block_n;
static uint32_t s_cal_block_ms;
static float    s_cal_blocks[IMU_CAL_BLOCKS][3];
static uint8_t  s_cal_bn;
static uint32_t s_still_ms;
static uint32_t s_first_tick;


/* EXTI 中断里置位，主循环消费 */
static volatile bool s_drdy;

/* 供中断上下文读取，32 位对齐访问在 Cortex-M4 上是原子的 */
static volatile float s_yaw_rad;
static volatile float s_yaw_deg;
static volatile float s_roll_deg;
static volatile float s_pitch_deg;

/* Keil Watch 窗口用的调试镜像 */
volatile app_imu_dbg_t app_imu_dbg;

/* ------------------------------------------------------------------------- */
/* PB12 数据就绪中断（可选）                                                 */
/* ------------------------------------------------------------------------- */
#if APP_IMU_USE_DRDY_INT

/** 配置 PB12 为 EXTI，极性跟 APP_IMU_INT_ACTIVE_LOW 走 */
static void drdy_exti_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Pin   = GPIO_PIN_12;
    g.Mode  = APP_IMU_INT_ACTIVE_LOW ? GPIO_MODE_IT_FALLING : GPIO_MODE_IT_RISING;
    g.Pull  = APP_IMU_INT_ACTIVE_LOW ? GPIO_PULLUP : GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

#if APP_IMU_EXTI_MANAGED
/** 中断入口：由本文件提供，CubeMX 里不要再给 PB12 配 EXTI */
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
}
#endif

#endif /* APP_IMU_USE_DRDY_INT */

/* ------------------------------------------------------------------------- */
/* 初始化                                                                    */
/* ------------------------------------------------------------------------- */

int app_imu_init(void)
{
    icm20948_cfg_t cfg;
    int rc;

    s_inited      = false;
    s_ok          = false;
    s_frames      = 0u;
    s_fail_streak = 0u;
    s_drdy        = false;
    s_yaw_rad     = 0.0f;
    s_yaw_deg     = 0.0f;
    s_roll_deg    = 0.0f;
    s_pitch_deg   = 0.0f;

    memset((void *)&app_imu_dbg, 0, sizeof(app_imu_dbg));

    /* 1. 绑定移植层（硬件 I2C 或软件 I2C，由 port 文件里的宏决定） */
    imu_port_stm32_init(APP_IMU_I2C_HANDLE);

#if APP_IMU_USE_DRDY_INT
    drdy_exti_init();          /* PB12 -> EXTI，就绪时置标志 */
#endif

    /* 2. 驱动配置 */
    icm20948_default_cfg(&cfg);
    cfg.accel_fs             = ICM20948_ACCEL_FS_4G;      /* 麦轮启停冲击大，别用 2g */
    cfg.gyro_fs              = ICM20948_GYRO_FS_1000DPS;
    cfg.gyro_dlpf            = 3u;   /* 51.2Hz，延迟约 5ms（原 6 = 5.7Hz 约 40ms） */
    cfg.accel_dlpf           = 3u;
    cfg.avg_samples          = 2u;                        /* 2 点平均，延迟 10ms */
    cfg.gyro_smplrt_div      = APP_IMU_SMPLRT_DIV;
    cfg.accel_smplrt_div     = APP_IMU_SMPLRT_DIV;
    cfg.enable_mag           = (APP_IMU_USE_MAG != 0);    /* 室内/电机磁场干扰会让航向漂，默认关 */
    cfg.enable_drdy_int      = (APP_IMU_USE_DRDY_INT != 0);
    cfg.int_active_low       = (APP_IMU_INT_ACTIVE_LOW != 0);
    cfg.auto_gyro_calib      = true;                      /* 此时车必须完全静止 */
    cfg.gyro_calib_samples   = 1000u;                     /* 2s 零偏标定，越久越准（车必须静止） */
    cfg.gyro_calib_period_ms = 2u;

    rc = icm20948_init(&s_imu, imu_port_stm32_get(), &cfg);
    if (rc != ICM20948_OK) {
        return rc;
    }

    /* 3. 姿态解算：地面小车用 Mahony，动态跟随好，带磁力计时航向不漂 */
    imu_attitude_init(&s_att, IMU_ALGO_MAHONY, 0.005f);
    imu_attitude_set_mahony_gains(&s_att, 1.0f, 0.0f);
    imu_attitude_set_mag_gain(&s_att, 0.3f);              /* 室内磁场差就设 0 */
    imu_attitude_reset_yaw(&s_att, 0.0f);

    s_last_tick = HAL_GetTick();
    s_inited    = true;
    s_ok        = true;          /* 先认为可用，第一帧数据到了之后再由 poll 维护 */
    return 0;
}

/* 零偏细化：块中位数标定 + 静止慢更新 */
static void imu_bias_poll(const float g[3], const float a[3], float dt)
{
    uint32_t dt_ms = (uint32_t)(dt * 1000.0f + 0.5f);
    float    an    = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    uint8_t  i;

    if (s_first_tick == 0u) {
        s_first_tick = HAL_GetTick();
    }

    /* ---- 预热结束 -> 开始标定 ---- */
    if (s_cal_state == 0u) {
        if ((uint32_t)(HAL_GetTick() - s_first_tick) < IMU_WARMUP_MS) {
            return;
        }
        s_cal_state    = 1u;
        s_cal_bn       = 0u;
        s_cal_block_n  = 0u;
        s_cal_block_ms = 0u;
        for (i = 0u; i < 3u; i++) {
            s_cal_sum[i] = 0.0f;
        }
        return;
    }

    /* ---- 标定中：每 1 秒存一个块均值，最后取中位数 ---- */
    if (s_cal_state == 1u) {
        for (i = 0u; i < 3u; i++) {
            s_cal_sum[i] += g[i];
        }
        s_cal_block_n++;
        s_cal_block_ms += dt_ms;

        if (s_cal_block_ms >= IMU_CAL_BLOCK_MS) {
            if ((s_cal_bn < IMU_CAL_BLOCKS) && (s_cal_block_n > 0u)) {
                float inv = 1.0f / (float)s_cal_block_n;
                for (i = 0u; i < 3u; i++) {
                    s_cal_blocks[s_cal_bn][i] = s_cal_sum[i] * inv;
                }
                s_cal_bn++;
            }
            for (i = 0u; i < 3u; i++) {
                s_cal_sum[i] = 0.0f;
            }
            s_cal_block_n  = 0u;
            s_cal_block_ms = 0u;
        }

        if (s_cal_bn >= IMU_CAL_BLOCKS) {
            for (i = 0u; i < 3u; i++) {
                float v[IMU_CAL_BLOCKS];
                uint8_t j, k;

                for (j = 0u; j < IMU_CAL_BLOCKS; j++) {
                    v[j] = s_cal_blocks[j][i];
                }
                for (j = 1u; j < IMU_CAL_BLOCKS; j++) {     /* 插入排序 */
                    float t = v[j];
                    k = j;
                    while ((k > 0u) && (v[k - 1u] > t)) {
                        v[k] = v[k - 1u];
                        k--;
                    }
                    v[k] = t;
                }
                s_bias[i] += 0.5f * (v[IMU_CAL_BLOCKS / 2u - 1u] + v[IMU_CAL_BLOCKS / 2u]);
            }
            s_cal_state = 2u;
            s_still_ms  = 0u;
        }
        return;
    }

    /* ---- 运行中：静止时慢速跟踪（补偿温漂） ---- */
    if ((fabsf(g[0]) < IMU_STILL_GYRO_DPS) &&
        (fabsf(g[1]) < IMU_STILL_GYRO_DPS) &&
        (fabsf(g[2]) < IMU_STILL_GYRO_DPS) &&
        (fabsf(an - 1.0f) < IMU_STILL_ACC_G)) {
        s_still_ms += dt_ms;
        if (s_still_ms >= IMU_STILL_SETTLE_MS) {
            float kk = dt / IMU_BIAS_TAU_S;
            for (i = 0u; i < 3u; i++) {
                s_bias[i] += g[i] * kk;
            }
        }
    } else {
        s_still_ms = 0u;
    }
}

/* ------------------------------------------------------------------------- */
/* 周期轮询                                                                  */
/* ------------------------------------------------------------------------- */

void app_imu_poll(void)
{
    icm20948_sample_t s;
    float mag[3];
    float dt;
    uint32_t now;
    bool due;

    if (!s_inited) {
        return;
    }

    now = HAL_GetTick();

    /* 正常情况下按 5ms 定时取样；若 INT 引脚已触发则立即读，降低延迟 */
    due = ((uint32_t)(now - s_last_tick) >= APP_IMU_PERIOD_MS);
    if (s_drdy) {
        s_drdy = false;
        due = true;
    }
    if (!due) {
        return;
    }

    dt = (float)(now - s_last_tick) * 0.001f;
    s_last_tick = now;
    if (dt > 0.05f) {
        dt = 0.05f;                              /* 主循环被长时间阻塞时保护积分 */
    }

    if (icm20948_read(&s_imu, &s) != ICM20948_OK) {
        s_fail_streak++;
        if (s_fail_streak >= APP_IMU_FAIL_LIMIT) {
            s_ok = false;                        /* 掉线：底盘退回里程计航向 */
        }
        app_imu_dbg.errors      = s_imu.error_count;
        app_imu_dbg.fail_streak = s_fail_streak;
        app_imu_dbg.ok          = s_ok ? 1u : 0u;
        return;
    }
    s_fail_streak = 0u;
    s_ok = true;

    /* 扣除附加零偏，再用残差做在线估计 */
    s.gyro_dps[0] -= s_bias[0];
    s.gyro_dps[1] -= s_bias[1];
    s.gyro_dps[2] -= s_bias[2];
    imu_bias_poll(s.gyro_dps, s.accel_g, dt);

    /* 磁力计只有 100Hz，读不到新数据就只用六轴 */
    if (icm20948_read_mag(&s_imu, mag) == ICM20948_OK) {
        (void)imu_attitude_update(&s_att, s.accel_g, s.gyro_dps, mag, dt);
    } else {
        (void)imu_attitude_update(&s_att, s.accel_g, s.gyro_dps, NULL, dt);
    }

    {
        float r, p, y;

        imu_attitude_get_euler(&s_att, &r, &p, &y);
        s_roll_deg  = r;
        s_pitch_deg = p;
        s_yaw_deg   = y * APP_IMU_YAW_SIGN;
        s_yaw_rad   = s_yaw_deg * 0.017453292519943295f;
    }

    s_last_dt = dt;
    s_frames++;

    app_imu_dbg.yaw_deg     = s_yaw_deg;
    app_imu_dbg.pitch_deg   = s_pitch_deg;
    app_imu_dbg.roll_deg    = s_roll_deg;
    app_imu_dbg.dt          = dt;
    app_imu_dbg.frames      = s_frames;
    app_imu_dbg.errors      = s_imu.error_count;
    app_imu_dbg.fail_streak = 0u;
    app_imu_dbg.ok          = 1u;
}

void app_imu_on_drdy_isr(void)
{
    s_drdy = true;      /* 只置标志，中断里不做 I2C */
}

/* ------------------------------------------------------------------------- */
/* 查询接口                                                                  */
/* ------------------------------------------------------------------------- */

bool app_imu_ok(void)
{
    return s_ok;
}

bool app_imu_bias_ready(void)
{
    return s_inited && s_cal_state == 2U;
}

float app_imu_yaw_rad(void)
{
    return s_yaw_rad;
}

float app_imu_yaw_deg(void)
{
    return s_yaw_deg;
}

void app_imu_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (roll_deg != NULL) {
        *roll_deg = s_roll_deg;
    }
    if (pitch_deg != NULL) {
        *pitch_deg = s_pitch_deg;
    }
    if (yaw_deg != NULL) {
        *yaw_deg = s_yaw_deg;
    }
}

void app_imu_zero_yaw(void)
{
    if (!s_inited) {
        return;
    }
    imu_attitude_reset_yaw(&s_att, 0.0f);
    s_yaw_rad = 0.0f;
    s_yaw_deg = 0.0f;
}

uint32_t app_imu_frames(void)
{
    return s_frames;
}

uint32_t app_imu_errors(void)
{
    return s_imu.error_count;
}

float app_imu_last_dt(void)
{
    return s_last_dt;
}

/* ------------------------------------------------------------------------- */
/* OLED 显示（可选）                                                         */
/* ------------------------------------------------------------------------- */

#ifdef APP_IMU_USE_OLED

#include "bsp_oled.h"

/** 显示带符号的 3 位整数：+123 / -045，占用 4 个字符（24px） */
static void app_imu_disp3(uint8_t x, uint8_t y, int16_t v)
{
    if (v < 0) {
        OLED_ShowChar(x, y, (uint8_t)'-', 12, 1);
        v = (int16_t)(-v);
    } else {
        OLED_ShowChar(x, y, (uint8_t)'+', 12, 1);
    }
    if (v > 999) {
        v = 999;
    }
    OLED_ShowChar((uint8_t)(x + 6u),  y, (uint8_t)('0' + (v / 100)), 12, 1);
    OLED_ShowChar((uint8_t)(x + 12u), y, (uint8_t)('0' + ((v / 10) % 10)), 12, 1);
    OLED_ShowChar((uint8_t)(x + 18u), y, (uint8_t)('0' + (v % 10)), 12, 1);
}

void app_imu_display(uint8_t y)
{
    if (!s_ok) {
        OLED_ShowString(0, y, (const uint8_t *)"IMU --");
        return;
    }

    OLED_ShowChar(0, y, (uint8_t)'Y', 12, 1);
    app_imu_disp3(6, y, (int16_t)s_yaw_deg);

    OLED_ShowChar(0, (uint8_t)(y + 12u), (uint8_t)'P', 12, 1);
    app_imu_disp3(6, (uint8_t)(y + 12u), (int16_t)s_pitch_deg);
}

#else

void app_imu_display(uint8_t y)
{
    (void)y;
}

#endif /* APP_IMU_USE_OLED */
