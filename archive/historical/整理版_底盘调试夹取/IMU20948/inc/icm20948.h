/**
 * @file    icm20948.h
 * @brief   ICM-20948 九轴 IMU 驱动（可移植，无 MCU / HAL 依赖）
 *
 * 本驱动由 WHEELTEC L150Pro 工程 HARDWARE/ICM20948/ICM20948.c 提取并重构：
 *   - 去掉对 STM32 标准库、全局变量、FreeRTOS、LED/按键等业务代码的依赖
 *   - 所有 I2C 访问改为通过 imu_port_t 回调，可跑在任意 MCU / Linux 上
 *   - 返回值统一为错误码，不再出现“返回未初始化变量”的问题
 *   - 原始数据 -> 物理量（g / dps / ℃ / uT）的换算集中在本层
 *   - 陀螺零偏、磁力计硬磁补偿、轴系重映射均可配置
 *
 * 典型用法：
 * @code
 *   static icm20948_t imu;
 *   icm20948_cfg_t cfg;
 *
 *   imu_port_stm32f4_init();                     // 移植层
 *   icm20948_default_cfg(&cfg);
 *   cfg.enable_mag = true;
 *   if (icm20948_init(&imu, imu_port_stm32f4_get(), &cfg) != ICM20948_OK) { ... }
 *
 *   while (1) {                                  // 100~200 Hz
 *       icm20948_sample_t s;
 *       if (icm20948_read(&imu, &s) == ICM20948_OK) {
 *           // s.accel_g[3] / s.gyro_dps[3] 可直接送入姿态解算
 *       }
 *   }
 * @endcode
 */
#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_port.h"
#include "icm20948_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* 配置与数据结构                                                            */
/* ------------------------------------------------------------------------- */

#define ICM20948_MAX_AVG_SAMPLES   8u     /* 硬件滑动平均最大点数（与原工程一致） */
#define ICM20948_GYRO_CALIB_MAX    2000u  /* 零偏标定最大采样次数 */

/** 总线类型 */
typedef enum {
    ICM20948_BUS_I2C = 0,
    ICM20948_BUS_SPI = 1
} icm20948_bus_t;

/** 加速度量程 */
typedef enum {
    ICM20948_ACCEL_FS_2G  = 0,
    ICM20948_ACCEL_FS_4G  = 1,
    ICM20948_ACCEL_FS_8G  = 2,
    ICM20948_ACCEL_FS_16G = 3
} icm20948_accel_fs_t;

/** 陀螺量程 */
typedef enum {
    ICM20948_GYRO_FS_250DPS  = 0,
    ICM20948_GYRO_FS_500DPS  = 1,
    ICM20948_GYRO_FS_1000DPS = 2,
    ICM20948_GYRO_FS_2000DPS = 3
} icm20948_gyro_fs_t;

/**
 * 数字低通滤波带宽（DLPFCFG，0~7）
 * 陀螺：0=196.6Hz 1=151.8Hz 2=119.5Hz 3=51.2Hz 4=23.9Hz 5=11.6Hz 6=5.7Hz 7=361.4Hz
 * 加计：0=246.0Hz 1=246.0Hz 2=111.4Hz 3=50.4Hz 4=23.9Hz 5=11.5Hz 6=5.7Hz 7=473Hz
 */
#define ICM20948_DLPF_CFG_MAX  7u

/**
 * @brief 初始化配置
 *
 * 结构体全部字段都有默认值，先调用 icm20948_default_cfg() 再按需修改。
 */
typedef struct {
    icm20948_bus_t      bus;              /* 总线类型，默认 I2C */
    uint8_t             i2c_addr7;        /* 7 位从机地址：0x68(AD0=0) / 0x69(AD0=1) */
    icm20948_accel_fs_t accel_fs;         /* 加计量程 */
    icm20948_gyro_fs_t  gyro_fs;          /* 陀螺量程 */
    uint8_t             accel_dlpf;       /* 加计数字低通 0~7 */
    uint8_t             gyro_dlpf;        /* 陀螺数字低通 0~7 */
    uint8_t             accel_smplrt_div; /* 加计采样分频，ODR = 1125Hz/(1+div) */
    uint8_t             gyro_smplrt_div;  /* 陀螺采样分频，ODR = 1125Hz/(1+div) */
    bool                enable_mag;       /* 是否启用 AK09916 磁力计（100Hz 连续） */
    uint8_t             avg_samples;      /* 原始值滑动平均点数：0/1=关闭，最大 8 */
    bool                enable_drdy_int;  /* 是否把 RAW_DATA_RDY 输出到 INT1 引脚 */
    bool                int_active_low;   /* INT1 极性：true=低有效开漏，false=高有效推挽 */
    bool                auto_gyro_calib;  /* 初始化时自动做陀螺零偏标定（需保持静止） */
    uint16_t            gyro_calib_samples;  /* 零偏标定采样次数 */
    uint16_t            gyro_calib_period_ms;/* 零偏标定采样间隔 */
    int8_t              axis_map[3];      /* 轴系重映射：输出轴 i = sign * 源轴(|v|-1)，默认 {1,2,3} */
} icm20948_cfg_t;

/** 一次采样结果 */
typedef struct {
    int16_t  accel_raw[3];   /* 传感器原始值（未做轴系映射，单位 LSB） */
    int16_t  gyro_raw[3];    /* 同上 */
    int16_t  temp_raw;       /* 温度原始值 */
    float    accel_g[3];     /* 已做量程换算 + 轴系映射，单位 g */
    float    gyro_dps[3];    /* 已扣除零偏 + 轴系映射，单位 deg/s */
    float    temp_c;         /* 温度，单位 ℃ */
    uint32_t timestamp_ms;   /* 采样时间戳（移植层未提供 millis 时为 0） */
} icm20948_sample_t;

/** 驱动状态（内部字段请勿直接修改） */
typedef struct {
    const imu_port_t *port;
    icm20948_cfg_t    cfg;

    uint8_t  bank;              /* 当前 bank，0xFF 表示未知 */
    bool     mag_ok;            /* AK09916 是否可用 */
    bool     initialized;

    float    gyro_bias_dps[3];  /* 陀螺零偏（dps） */
    float    mag_offset_uT[3];  /* 磁力计硬磁偏移（uT） */

    int16_t  avg_buf[6][ICM20948_MAX_AVG_SAMPLES];  /* 0~2 = accel XYZ，3~5 = gyro XYZ */
    uint8_t  avg_idx;
    uint8_t  avg_fill;

    uint32_t sample_count;
    uint32_t error_count;
} icm20948_t;

/** 错误码：0 = 成功，负数 = 失败 */
typedef enum {
    ICM20948_OK             = 0,
    ICM20948_ERR_PARAM      = -1,  /* 入参非法 / 配置越界 */
    ICM20948_ERR_PORT       = -2,  /* 移植层回调缺失 */
    ICM20948_ERR_WHO_AM_I   = -3,  /* WHO_AM_I 不匹配，器件未接好或供电异常 */
    ICM20948_ERR_BUS        = -4,  /* 总线读写失败（NACK / 超时） */
    ICM20948_ERR_MAG        = -5,  /* AK09916 未检测到 */
    ICM20948_ERR_NO_DATA    = -6,  /* 磁力计暂无新数据 */
    ICM20948_ERR_UNSUPPORTED= -7   /* 功能未启用 */
} icm20948_status_t;

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */

/** 填充默认配置（与原工程参数等价：±500dps / ±2g / DLPF=6 / 1kHz分频8） */
void icm20948_default_cfg(icm20948_cfg_t *cfg);

/**
 * @brief 绑定移植层与配置（不做任何总线访问）
 * @note  之后可调用 icm20948_probe() 单独探测器件是否存在
 */
int icm20948_attach(icm20948_t *dev, const imu_port_t *port, const icm20948_cfg_t *cfg);

/**
 * @brief 读取 WHO_AM_I 原始值（不做校验），对应原工程 ICM20948_getDeviceID()
 * @param[out] id 读到的值
 */
int icm20948_read_id(icm20948_t *dev, uint8_t *id);

/**
 * @brief 探测器件：读取并校验 WHO_AM_I
 * @param[out] id 读到的 WHO_AM_I 原始值，可为 NULL
 * @return ICM20948_OK / ICM20948_ERR_BUS / ICM20948_ERR_WHO_AM_I
 */
int icm20948_probe(icm20948_t *dev, uint8_t *id);

/**
 * @brief 完整初始化：软复位 -> 唤醒 -> 配置量程/滤波/采样率 -> 磁力计 -> 零偏标定
 * @note  调用前需保证器件已上电并保持静止（若开启了 auto_gyro_calib）
 */
int icm20948_init(icm20948_t *dev, const imu_port_t *port, const icm20948_cfg_t *cfg);

/** 软复位（复位后自动唤醒并恢复 bank0） */
int icm20948_reset(icm20948_t *dev);

/** 进入/退出低功耗睡眠模式 */
int icm20948_sleep(icm20948_t *dev, bool enable);

/** 读取原始值（含可选的滑动平均），不做量程换算 */
int icm20948_read_raw(icm20948_t *dev, icm20948_sample_t *out);

/** 读取一次完整数据：原始值 + 物理量（推荐使用） */
int icm20948_read(icm20948_t *dev, icm20948_sample_t *out);

/** 读取磁力计（uT，已扣除硬磁偏移）。需 cfg.enable_mag = true */
int icm20948_read_mag(icm20948_t *dev, float mag_uT[3]);

/** 磁力计是否可用（初始化时 AK09916 校验通过） */
bool icm20948_mag_available(const icm20948_t *dev);

/** 判断是否有新的原始数据（读 INT_STATUS_1.RAW_DATA_RDY_INT） */
int icm20948_data_ready(icm20948_t *dev, bool *ready);

/**
 * @brief 使能/关闭“数据就绪”中断输出（INT1 引脚）
 *
 * 配置 INT_PIN_CFG：任意寄存器读操作自动清中断，保证 INT 是短脉冲，
 * 适合接 MCU 的 EXTI 边沿触发。极性由 cfg.int_active_low 决定。
 *
 * @note 引脚接法必须与极性一致：
 *       - 高有效推挽（默认）：INT 空闲低、就绪时拉高 → EXTI 上升沿
 *       - 低有效开漏：INT 需要外部上拉，就绪时拉低 → EXTI 下降沿
 */
int icm20948_enable_drdy_int(icm20948_t *dev, bool enable);

/**
 * @brief 陀螺零偏标定：静止状态下连续采样取平均
 * @param samples    采样次数（1 ~ 2000）
 * @param period_ms  采样间隔（原工程为 10ms）
 * @note  标定结果会在 icm20948_read() 中自动扣除
 */
int icm20948_calibrate_gyro(icm20948_t *dev, uint16_t samples, uint32_t period_ms);

/** 手动设置/读取陀螺零偏（dps） */
void icm20948_set_gyro_bias(icm20948_t *dev, const float bias_dps[3]);
void icm20948_get_gyro_bias(const icm20948_t *dev, float bias_dps[3]);

/** 手动设置磁力计硬磁偏移（uT） */
void icm20948_set_mag_offset(icm20948_t *dev, const float offset_uT[3]);

/** 错误码转字符串 */
const char *icm20948_strerror(int err);

#ifdef __cplusplus
}
#endif

#endif /* ICM20948_H */
