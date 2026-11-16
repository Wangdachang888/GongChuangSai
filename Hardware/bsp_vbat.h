/**
 * @file    bsp_vbat.h
 * @brief   电源电压检测（双通道）
 *
 * 通道 1（驱动器侧）：D36A 的电源检测输出 = VIN/11  -> PC0 = ADC1_IN10（H2 第 15 脚）
 * 通道 2（主板侧）  ：板上自带 1/11 分压（R27=100k / R26=10k）-> PB0 = ADC1_IN8
 *
 * 两个通道的分压比都是 11：VIN = raw / 4095 * VREF * 11
 *
 * 用法：
 *   Vbat_Init();                  // 初始化 (PC0/PB0 模拟输入 + ADC1 校准)
 *   Vbat_Poll();                  // 主循环反复调用，内部 100ms 节流
 *   Vbat_GetDriverVoltage();      // 驱动器侧 VIN (V)
 *   Vbat_GetBoardVoltage();       // 主板侧 VIN (V)
 */
#ifndef __BSP_VBAT_H
#define __BSP_VBAT_H

#include <stdint.h>

/* ---------------- 硬件参数 ---------------- */
#define VBAT_CH_DRIVER        10u      /* PC0 = ADC1_IN10，接 D36A 的 VIN/11 */
#define VBAT_CH_BOARD          8u      /* PB0 = ADC1_IN8，板上 1/11 分压 */

#define VBAT_DIVIDER_RATIO    11.0f    /* 两路都是 1/11 */
#define VBAT_VREF             3.30f    /* VDDA 实测值；3.3V 供电一般 3.28~3.32 */
#define VBAT_ADC_MAX          4095.0f  /* 12 位 ADC */

/* ---------------- 接口 ---------------- */
void     Vbat_Init(void);
void     Vbat_Poll(void);

/* 驱动器侧（PC0） */
float    Vbat_GetDriverVoltage(void);
uint16_t Vbat_GetDriverRaw(void);
uint8_t  Vbat_IsDriverValid(void);

/* 主板侧（PB0） */
float    Vbat_GetBoardVoltage(void);
uint16_t Vbat_GetBoardRaw(void);
uint8_t  Vbat_IsBoardValid(void);

/* 兼容旧接口：等价于驱动器侧 */
float    Vbat_GetVoltage(void);
uint8_t  Vbat_IsValid(void);

#endif /* __BSP_VBAT_H */
