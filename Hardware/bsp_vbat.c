/**
 * @file    bsp_vbat.c
 * @brief   电源电压检测（双通道）—— 寄存器级 ADC1 读取
 *
 * 为什么不用 HAL_ADC：
 *   本工程没有启用 HAL 的 ADC 模块（stm32f4xx_hal_conf.h 里
 *   HAL_ADC_MODULE_ENABLED 是注释掉的，Keil 工程也没编译 hal_adc.c），
 *   所以这里直接操作 ADC1 寄存器，不增加任何工程配置。
 *
 * 采样流程：
 *   PC0/PB0 模拟输入 -> ADC1 单次转换（软件触发）-> 8 次平均
 *   -> 乘 11 得到 VIN -> 一阶低通（0.8 旧 + 0.2 新）
 */
#include "bsp_vbat.h"
#include "main.h"

/* ---------------- 内部参数 ---------------- */
#define VBAT_SMP_CYCLES     7u      /* 采样时间 = 480 个 ADC 周期 */
#define VBAT_POLL_MS        100u    /* 10Hz 更新 */
#define VBAT_SAMPLES        8u      /* 每次平均 8 个采样 */
#define VBAT_VALID_MIN      3.0f    /* 低于 3V 认为没接/断线 */

/* CMSIS 的 F4 头文件没有给出这两个位宏，按手册位号定义 */
#define VBAT_CR2_RSTCAL     (1u << 3)   /* 复位校准 */
#define VBAT_CR2_CAL        (1u << 2)   /* 启动校准 */

typedef struct {
    uint16_t raw;
    float    voltage;
    uint8_t  valid;
} VbatChannel_t;

static VbatChannel_t s_driver;      /* PC0：D36A 的 VIN/11 */
static VbatChannel_t s_board;       /* PB0：主板 1/11 分压 */
static uint32_t      s_last_ms;

/* ------------------------------------------------------------------------- */
/* 初始化                                                                    */
/* ------------------------------------------------------------------------- */
void Vbat_Init(void)
{
    volatile uint32_t i;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* PC0 -> 模拟输入（MODER=11），关闭上下拉 */
    GPIOC->MODER |= (3u << (0u * 2u));
    GPIOC->PUPDR &= ~(3u << (0u * 2u));

    /* PB0 -> 模拟输入 */
    GPIOB->MODER |= (3u << (0u * 2u));
    GPIOB->PUPDR &= ~(3u << (0u * 2u));

    /* ADC 时钟 = PCLK2 / 4 = 21MHz（F407 的 ADC 上限 36MHz） */
    ADC123_COMMON->CCR = (ADC123_COMMON->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

    ADC1->CR1 = 0u;                     /* 12 位、非扫描、单通道 */
    ADC1->CR2 = 0u;
    ADC1->CR2 |= ADC_CR2_ADON;          /* 给 ADC 上电 */
    for (i = 0u; i < 2000u; i++) { }     /* 等待稳定 tSTAB */

    /* 采样时间：通道 10 在 SMPR1[2:0]，通道 8 在 SMPR2[26:24] */
    ADC1->SMPR1 = (ADC1->SMPR1 & ~(7u << 0))  | ((uint32_t)VBAT_SMP_CYCLES << 0);
    ADC1->SMPR2 = (ADC1->SMPR2 & ~(7u << 24)) | ((uint32_t)VBAT_SMP_CYCLES << 24);

    /* 规则组只有 1 个转换，具体通道每次转换前写 SQR3 */
    ADC1->SQR1 = 0u;
    ADC1->SQR3 = VBAT_CH_DRIVER;

    /* 校准：先复位校准寄存器，再启动校准，各自等待完成 */
    ADC1->CR2 |= VBAT_CR2_RSTCAL;
    while ((ADC1->CR2 & VBAT_CR2_RSTCAL) != 0u) { }

    ADC1->CR2 |= VBAT_CR2_CAL;
    while ((ADC1->CR2 & VBAT_CR2_CAL) != 0u) { }

    s_driver.raw = 0u; s_driver.voltage = 0.0f; s_driver.valid = 0u;
    s_board.raw  = 0u; s_board.voltage  = 0.0f; s_board.valid  = 0u;
    s_last_ms = 0u;
}

/* ------------------------------------------------------------------------- */
/* 单次转换                                                                  */
/* ------------------------------------------------------------------------- */
static uint16_t vbat_read_channel(uint8_t channel)
{
    uint32_t t0;

    ADC1->SQR3 = (uint32_t)channel;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    t0 = HAL_GetTick();
    while ((ADC1->SR & ADC_SR_EOC) == 0u) {
        if ((uint32_t)(HAL_GetTick() - t0) > 2u) {
            return 0u; /* ADC hang -> do not block main loop forever */
        }
    }
    return (uint16_t)(ADC1->DR & 0x0FFFu);
}

/* ------------------------------------------------------------------------- */
/* 采样并换算                                                                */
/* ------------------------------------------------------------------------- */
static void vbat_update(VbatChannel_t *ch, uint8_t channel)
{
    uint32_t sum = 0u;
    uint8_t  i;
    float    v;

    for (i = 0u; i < VBAT_SAMPLES; i++) {
        sum += vbat_read_channel(channel);
    }
    ch->raw = (uint16_t)(sum / VBAT_SAMPLES);

    /* 引脚电压 -> VIN：raw/4095 * VREF * 11 */
    v = (float)ch->raw * (VBAT_VREF / VBAT_ADC_MAX) * VBAT_DIVIDER_RATIO;

    if (ch->valid == 0u) {
        ch->voltage = v;                                /* 首次直接赋值 */
    } else {
        ch->voltage = ch->voltage * 0.8f + v * 0.2f;    /* 一阶低通 */
    }
    ch->valid = (ch->voltage >= VBAT_VALID_MIN) ? 1u : 0u;
}

/* ------------------------------------------------------------------------- */
/* 周期采样（主循环调用，内部 100ms 节流）                                    */
/* ------------------------------------------------------------------------- */
void Vbat_Poll(void)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - s_last_ms) < VBAT_POLL_MS) {
        return;
    }
    s_last_ms = now;

    vbat_update(&s_driver, VBAT_CH_DRIVER);   /* PC0 */
    vbat_update(&s_board,  VBAT_CH_BOARD);    /* PB0 */
}

/* ------------------------------------------------------------------------- */
/* 查询                                                                      */
/* ------------------------------------------------------------------------- */
float Vbat_GetDriverVoltage(void) { return s_driver.valid ? s_driver.voltage : 0.0f; }
float Vbat_GetBoardVoltage(void)  { return s_board.valid  ? s_board.voltage  : 0.0f; }
uint16_t Vbat_GetDriverRaw(void)  { return s_driver.raw; }
uint16_t Vbat_GetBoardRaw(void)   { return s_board.raw; }
uint8_t Vbat_IsDriverValid(void)  { return s_driver.valid; }
uint8_t Vbat_IsBoardValid(void)   { return s_board.valid; }

/* 兼容旧接口 */
float Vbat_GetVoltage(void)       { return Vbat_GetDriverVoltage(); }
uint8_t Vbat_IsValid(void)        { return Vbat_IsDriverValid(); }
