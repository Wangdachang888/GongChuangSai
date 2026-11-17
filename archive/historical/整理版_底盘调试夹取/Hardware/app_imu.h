/**
 * @file    app_imu.h
 * @brief   IMU 应用层封装（面向 F407 麦轮底盘工程）
 *
 * 把 icm20948 驱动 + 姿态解算封装成“初始化一次、周期轮询”的简单接口，
 * 供 main.c 的 while(1) 调用，并把 yaw 提供给底盘位置环使用。
 *
 * 使用（main.c）：
 *   USER CODE Includes:  #include "app_imu.h"
 *   USER CODE 2:         if (app_imu_init() != 0) { 打印错误; }
 *   while(1):            app_imu_poll();
 *   mecanum_chassis.c:   chassis->theta_pos = app_imu_yaw_rad();
 */
#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 姿态更新周期（ms），5ms = 200Hz，与 TIM6 的 10ms 底盘环错开 */
#ifndef APP_IMU_PERIOD_MS
#define APP_IMU_PERIOD_MS       5u
#endif

/**
 * yaw 方向系数：
 *   底盘约定 “逆时针为正” 时填 +1.0f；
 *   若手转小车逆时针、app_imu_yaw_deg() 反而变小，改成 -1.0f。
 */
#ifndef APP_IMU_YAW_SIGN
#define APP_IMU_YAW_SIGN        (1.0f)
#endif

/**
 * 数据就绪中断（IMU 的 INT 引脚，你板子上是 PB12）：
 *   0 = 不配置 INT 引脚，纯靠 5ms 定时轮询（默认，最稳）
 *   1 = 初始化时打开 RAW_DATA_RDY 输出，配合 app_imu_on_drdy_isr()
 *       需要同时在 CubeMX 里把 PB12 配成 EXTI，极性见下
 */
#ifndef APP_IMU_USE_DRDY_INT
#define APP_IMU_USE_DRDY_INT    0
#endif

/**
 * INT 极性：
 *   0 = 高有效推挽（INT 空闲低、就绪拉高）→ EXTI 上升沿，无需外部上拉
 *   1 = 低有效开漏（INT 空闲高、就绪拉低）→ EXTI 下降沿，需要外部上拉
 * 实测方法：上电后万用表量 PB12，静止时是高还是低，再决定改不改。
 */
#ifndef APP_IMU_INT_ACTIVE_LOW
#define APP_IMU_INT_ACTIVE_LOW  0
#endif

/**
 * 是否由 app_imu.c 自己配置 PB12 的 EXTI（引脚 + NVIC + 中断入口）
 *   1 = 全自动，不依赖 CubeMX 配置
 *   0 = 你自己在 CubeMX 里配 PB12 为 EXTI，此时本文件不碰 GPIO/NVIC
 * @note 若以后在 CubeMX 里给 PB12 加了 EXTI，一定要把这里改成 0，
 *       否则 EXTI15_10_IRQHandler 会重复定义。
 */
#ifndef APP_IMU_EXTI_MANAGED
#define APP_IMU_EXTI_MANAGED    1
#endif

/**
 * @brief 在 EXTI 回调里调用，标记“有新数据”
 *
 * 只置一个标志，不做 I2C 访问，可以在中断里安全调用。
 * 未启用 APP_IMU_USE_DRDY_INT 时调用它也无害（poll 会立刻处理一次）。
 */
void app_imu_on_drdy_isr(void);

/**
 * @brief 初始化（内部含 100ms 复位等待 + 约 0.8s 零偏标定，调用时车必须静止）
 * @return 0 成功；负数见 icm20948_strerror()
 * @note  必须在 HAL_Init() / MX_I2Cx_Init() 之后、启动底盘运动之前调用
 */
int app_imu_init(void);

/**
 * @brief 周期轮询（在主循环里反复调用即可，内部按 APP_IMU_PERIOD_MS 自动分频）
 * @note  绝对不要在 TIM6 中断里调用：HAL_I2C_Mem_Read 是阻塞的，
 *        总线异常时最长会卡住 50ms，会直接打乱 1ms 控制节拍。
 */
void app_imu_poll(void);

/** IMU 是否初始化成功且正在输出姿态 */
bool app_imu_ok(void);
bool app_imu_bias_ready(void);

/** 航向角，弧度，[-π, π]，已乘 APP_IMU_YAW_SIGN */
float app_imu_yaw_rad(void);

/** 航向角，度，[-180, 180]，已乘 APP_IMU_YAW_SIGN */
float app_imu_yaw_deg(void);

/** 读取欧拉角（度），指针可为 NULL；yaw 已乘 APP_IMU_YAW_SIGN */
void app_imu_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);

/** 把当前航向记为 0°（换场地、重新对位时调用） */
void app_imu_zero_yaw(void);

/** 累计成功更新的帧数（判断“有没有真的在出数据”） */
uint32_t app_imu_frames(void);

/** 累计总线错误次数 */
uint32_t app_imu_errors(void);

/** 上一帧实际使用的 dt（秒），用于观察主循环是否被阻塞 */
float app_imu_last_dt(void);

/**
 * @brief 调试镜像：全部状态集中在一个结构体里
 *
 * Keil 调试时只要在 Watch 窗口加 `app_imu_dbg`，展开就能看到
 * yaw / frames / errors，不需要串口打印。
 */
typedef struct {
    float    yaw_deg;
    float    pitch_deg;
    float    roll_deg;
    float    dt;
    uint32_t frames;
    uint32_t errors;
    uint32_t fail_streak;
    uint8_t  ok;
} app_imu_dbg_t;

extern volatile app_imu_dbg_t app_imu_dbg;

/**
 * @brief 在 OLED 上显示 yaw / pitch（两行，12px 字体）
 * @param y 起始行坐标，例如 0 / 12 / 24 / 36 / 48
 * @note  需要定义 APP_IMU_USE_OLED，并保证 bsp_oled.h 可用
 */
void app_imu_display(uint8_t y);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_H */
