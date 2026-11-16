/**
 * @file    imu_port.h
 * @brief   ICM-20948 可移植模块 —— 硬件抽象层（移植层）接口
 *
 * 本文件是整个模块唯一的“平台相关”接口。移植到新平台时，
 * 只需要实现下面这些回调函数，驱动与姿态解算代码一行都不用改。
 *
 * 约定：
 *   - 所有回调返回 0 表示成功，返回非 0 表示失败（驱动统一映射为 ICM20948_ERR_BUS）。
 *   - I2C 回调使用 7 位从机地址（不含读写位）。原工程里的 0xD0 是 8 位写地址，
 *     即 7 位地址 0x68（0xD0 >> 1），移植时注意转换。
 *   - SPI 回调使用 “片选在外部控制” 的方式：一次调用完成 1 个命令字节 + N 个数据字节，
 *     tx/rx 长度相同；写操作时 rx 可以为 NULL。
 *   - delay_ms 必须实现；delay_us 可选（软件 I2C 需要）。
 *   - millis 可选，用于给采样数据打时间戳。
 *   - log 可选，用于输出调试信息。
 */
#ifndef IMU_PORT_H
#define IMU_PORT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** I2C 写寄存器：向 addr7 设备的 reg 寄存器开始写入 len 字节 */
typedef int (*imu_i2c_write_fn)(void *ctx, uint8_t addr7, uint8_t reg,
                                const uint8_t *data, uint16_t len);

/** I2C 读寄存器：从 addr7 设备的 reg 寄存器开始连续读出 len 字节 */
typedef int (*imu_i2c_read_fn)(void *ctx, uint8_t addr7, uint8_t reg,
                               uint8_t *data, uint16_t len);

/** SPI 全双工传输：片选由调用者（驱动侧）之外控制，长度 = 1 + N */
typedef int (*imu_spi_xfer_fn)(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len);

/** 毫秒/微秒延时 */
typedef void (*imu_delay_ms_fn)(void *ctx, uint32_t ms);
typedef void (*imu_delay_us_fn)(void *ctx, uint32_t us);

/** 毫秒级系统时间（可选，用于时间戳） */
typedef uint32_t (*imu_millis_fn)(void *ctx);

/** 调试输出（可选） */
typedef void (*imu_log_fn)(void *ctx, const char *msg);

/**
 * @brief 硬件移植层描述结构体
 *
 * ctx 会原样传回给每一个回调，可以放 I2C 句柄、SPI 句柄等。
 * 该结构体在驱动初始化时被复制引用，生命周期必须覆盖整个使用期间。
 */
typedef struct imu_port {
    void             *ctx;

    imu_i2c_write_fn  i2c_write;   /* I2C 模式必填 */
    imu_i2c_read_fn   i2c_read;    /* I2C 模式必填 */
    imu_spi_xfer_fn   spi_xfer;    /* SPI 模式必填 */

    imu_delay_ms_fn   delay_ms;    /* 必填 */
    imu_delay_us_fn   delay_us;    /* 软件 I2C 必填 */
    imu_millis_fn     millis;      /* 可选 */
    imu_log_fn        log;         /* 可选 */
} imu_port_t;

#ifdef __cplusplus
}
#endif

#endif /* IMU_PORT_H */
