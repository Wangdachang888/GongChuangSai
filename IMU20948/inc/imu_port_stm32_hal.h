/**
 * @file    imu_port_stm32_hal.h
 * @brief   STM32 + HAL 通用移植层接口（F0/F1/F3/F4/F7/G0/G4/H5/H7/L4/U5 通用）
 *
 * 不依赖任何 HAL 头文件，只传 void* 句柄，因此可以放在公共 include 路径里，
 * 驱动与姿态解算代码完全不感知芯片型号。
 *
 * 两种实现，编译期二选一：
 *   IMU_STM32_USE_SOFT_I2C = 0（默认）硬件 I2C，使用 HAL_I2C_Mem_Read/Write
 *   IMU_STM32_USE_SOFT_I2C = 1        软件 I2C，任意 GPIO + DWT 微秒延时
 *
 * 典型用法（F407 + CubeMX）：
 * @code
 *   MX_I2C1_Init();                              // CubeMX 生成
 *   imu_port_stm32_init(&hi2c1);                 // 软件 I2C 时传 NULL
 *   icm20948_init(&imu, imu_port_stm32_get(), &cfg);
 * @endcode
 */
#ifndef IMU_PORT_STM32_HAL_H
#define IMU_PORT_STM32_HAL_H

#include "imu_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 STM32 移植层（可重复调用）
 * @param i2c_handle 硬件 I2C 模式传 &hi2c1 / &hi2c2 / &hi2c4 …；软件 I2C 模式传 NULL
 * @note  硬件 I2C 模式下要求该句柄已经由 MX_I2Cx_Init() 初始化完成
 */
void imu_port_stm32_init(void *i2c_handle);

/** 获取移植层实例，直接传给 icm20948_init() */
const imu_port_t *imu_port_stm32_get(void);

/**
 * @brief 手动总线恢复
 * @note  软件 I2C 模式：打 9 个时钟 + STOP；
 *        硬件 I2C 模式：HAL_I2C_DeInit/HAL_I2C_Init 复位外设（清 AF/BUSY）
 */
void imu_port_stm32_bus_recover(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_PORT_STM32_HAL_H */
