/**
 * @file    imu_port_stm32_hal.c
 * @brief   STM32 + HAL 通用移植层（F0/F1/F3/F4/F7/G0/G4/H5/H7/L4/U5）
 *
 * 默认使用硬件 I2C（HAL_I2C_Mem_Read / HAL_I2C_Mem_Write），
 * 定义 IMU_STM32_USE_SOFT_I2C = 1 可切换为软件 I2C（任意 GPIO + DWT 微秒延时）。
 *
 * 设计要点：
 *   1. 只用阻塞式 HAL 接口，不涉及 DMA：
 *      - F407 没有 D-Cache，天然无一致性问题；
 *      - H7 有 D-Cache，但阻塞模式由 CPU 直接访问缓冲区，同样不需要 cache 维护。
 *      若你把工程改成 Mem_Read_DMA，必须自己做 SCB_InvalidateDCache_by_Addr()。
 *   2. 传输失败时自动 HAL_I2C_DeInit + HAL_I2C_Init 复位外设（清 AF/BUSY）后重试一次；
 *      这条也顺带规避了 F4 系列 I2C 的 “BUSY 位卡死” 勘误。
 *   3. HAL 的 I2C 句柄内部带 Lock，多任务同时访问会返回 HAL_BUSY；
 *      建议只在一个任务里读 IMU。
 */
#include "imu_port_stm32_hal.h"

/* CubeMX 工程一定有 main.h（内含对应的 stm32xxxx_hal.h）；
 * 非 CubeMX 工程按芯片型号替换下面这一行即可。 */
#if defined(__has_include)
#  if __has_include("main.h")
#    include "main.h"
#  elif __has_include("stm32f4xx_hal.h")
#    include "stm32f4xx_hal.h"
#  elif __has_include("stm32h7xx_hal.h")
#    include "stm32h7xx_hal.h"
#  elif __has_include("stm32f7xx_hal.h")
#    include "stm32f7xx_hal.h"
#  else
#    error "请在此处包含你的芯片 HAL 头文件（stm32f4xx_hal.h 等）"
#  endif
#else
#  include "main.h"
#endif

/* ========================================================================= */
/* 可配置项                                                                  */
/* ========================================================================= */

/** 0 = 硬件 I2C（推荐），1 = 软件 I2C */
#ifndef IMU_STM32_USE_SOFT_I2C
#define IMU_STM32_USE_SOFT_I2C      1
#endif

/** 单次 HAL I2C 传输超时（ms） */
#ifndef IMU_STM32_I2C_TIMEOUT_MS
#define IMU_STM32_I2C_TIMEOUT_MS    50u
#endif

/** 毫秒延时：接了 FreeRTOS 可以改成 vTaskDelay(pdMS_TO_TICKS(ms)) 避免忙等 */
#ifndef IMU_STM32_DELAY_MS
#define IMU_STM32_DELAY_MS(ms)      HAL_Delay(ms)
#endif

/** 软件 I2C 半周期延时（us），2us ≈ 250kHz */
#ifndef IMU_STM32_SOFT_HALF_US
#define IMU_STM32_SOFT_HALF_US      2u
#endif

/** 软件 I2C 引脚（换成你的板子实际接线；F407 常见 I2C1 为 PB6/PB7 或 PB8/PB9） */
#ifndef IMU_STM32_SCL_PORT
#define IMU_STM32_SCL_PORT          GPIOB
#define IMU_STM32_SCL_PIN           GPIO_PIN_10
#define IMU_STM32_SDA_PORT          GPIOB
#define IMU_STM32_SDA_PIN           GPIO_PIN_11
#endif

/** 软件 I2C 引脚时钟使能（SCL/SDA 不同端口时改成两句） */
#ifndef IMU_STM32_SOFT_GPIO_CLK_ENABLE
#define IMU_STM32_SOFT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#endif

/* ========================================================================= */
/* 公共回调                                                                  */
/* ========================================================================= */

static void port_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    IMU_STM32_DELAY_MS(ms);
}

static uint32_t port_millis(void *ctx)
{
    (void)ctx;
    return HAL_GetTick();
}

/* ========================================================================= */
/* 方案 A：硬件 I2C（默认）                                                  */
/* ========================================================================= */
#if !IMU_STM32_USE_SOFT_I2C

static void hal_i2c_recover(I2C_HandleTypeDef *h)
{
    if (h == NULL) {
        return;
    }
    /* DeInit 会清 BUSY/AF 等状态并调用 MspDeInit，Init 再重新配置一次；
     * CubeMX 生成的 MspInit/MspDeInit 成对存在，这样复位最彻底。 */
    (void)HAL_I2C_DeInit(h);
    (void)HAL_I2C_Init(h);
}

static int hal_i2c_write(void *ctx, uint8_t addr7, uint8_t reg,
                         const uint8_t *data, uint16_t len)
{
    I2C_HandleTypeDef *h = (I2C_HandleTypeDef *)ctx;
    HAL_StatusTypeDef st;
    uint16_t dev = (uint16_t)((uint16_t)addr7 << 1);

    if (h == NULL || data == NULL || len == 0u) {
        return -1;
    }

    st = HAL_I2C_Mem_Write(h, dev, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t *)data, len, IMU_STM32_I2C_TIMEOUT_MS);
    if (st == HAL_OK) {
        return 0;
    }

    hal_i2c_recover(h);
    st = HAL_I2C_Mem_Write(h, dev, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t *)data, len, IMU_STM32_I2C_TIMEOUT_MS);
    return (st == HAL_OK) ? 0 : -1;
}

static int hal_i2c_read(void *ctx, uint8_t addr7, uint8_t reg,
                        uint8_t *data, uint16_t len)
{
    I2C_HandleTypeDef *h = (I2C_HandleTypeDef *)ctx;
    HAL_StatusTypeDef st;
    uint16_t dev = (uint16_t)((uint16_t)addr7 << 1);

    if (h == NULL || data == NULL || len == 0u) {
        return -1;
    }

    st = HAL_I2C_Mem_Read(h, dev, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                          data, len, IMU_STM32_I2C_TIMEOUT_MS);
    if (st == HAL_OK) {
        return 0;
    }

    hal_i2c_recover(h);
    st = HAL_I2C_Mem_Read(h, dev, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                          data, len, IMU_STM32_I2C_TIMEOUT_MS);
    return (st == HAL_OK) ? 0 : -1;
}

/* ========================================================================= */
/* 方案 B：软件 I2C                                                          */
/* ========================================================================= */
#else

/* ---------------------------- DWT 微秒延时 ------------------------------ */

static bool s_dwt_ok = false;

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_dwt_ok = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u);
}

static void dwt_delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    if (!s_dwt_ok) {
        /* CYCCNT 不可用时退化为粗略空循环，仅用于应急 */
        volatile uint32_t n = us * 100u;
        while (n-- > 0u) {
        }
        return;
    }

    start = DWT->CYCCNT;
    ticks = us * (SystemCoreClock / 1000000u);
    if (ticks == 0u) {
        ticks = 1u;
    }
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

#ifndef IMU_STM32_UDELAY
#define IMU_STM32_UDELAY(us)    dwt_delay_us(us)
#endif

#define SOFT_DLY()              IMU_STM32_UDELAY(IMU_STM32_SOFT_HALF_US)

/** 供 imu_port_t.delay_us 使用的包装（去掉 ctx 参数） */
static void port_delay_us(void *ctx, uint32_t us)
{
    (void)ctx;
    IMU_STM32_UDELAY(us);
}

/* ---------------------------- GPIO 操作 -------------------------------- */

/* SDA 配成开漏输出：写 1 释放（由上拉拉高），写 0 拉低；读 IDR 得到真实电平，
 * 这样就不需要在输入/输出模式之间来回切换，也不需要更改 MODER。 */
static void scl_write(uint8_t level)
{
    HAL_GPIO_WritePin(IMU_STM32_SCL_PORT, IMU_STM32_SCL_PIN,
                      (level != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void sda_write(uint8_t level)
{
    HAL_GPIO_WritePin(IMU_STM32_SDA_PORT, IMU_STM32_SDA_PIN,
                      (level != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t sda_read(void)
{
    return (HAL_GPIO_ReadPin(IMU_STM32_SDA_PORT, IMU_STM32_SDA_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}

static void soft_i2c_gpio_init(void)
{
    GPIO_InitTypeDef g;

    IMU_STM32_SOFT_GPIO_CLK_ENABLE();

    g.Mode      = GPIO_MODE_OUTPUT_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = 0;

    g.Pin = IMU_STM32_SCL_PIN;
    HAL_GPIO_Init(IMU_STM32_SCL_PORT, &g);

    g.Pin = IMU_STM32_SDA_PIN;
    HAL_GPIO_Init(IMU_STM32_SDA_PORT, &g);

    scl_write(1u);
    sda_write(1u);
    SOFT_DLY();
}

static void soft_bus_recover(void)
{
    uint8_t i;

    sda_write(1u);
    for (i = 0; i < 9u; i++) {
        scl_write(0u);
        SOFT_DLY();
        scl_write(1u);
        SOFT_DLY();
    }
    /* 补一个 STOP */
    scl_write(0u);
    sda_write(0u);
    SOFT_DLY();
    scl_write(1u);
    SOFT_DLY();
    sda_write(1u);
    SOFT_DLY();
}

/* ---------------------------- 时序 -------------------------------------- */

static void soft_start(void)
{
    sda_write(1u);
    scl_write(1u);
    SOFT_DLY();
    sda_write(0u);          /* SCL 高电平期间 SDA 下降 */
    SOFT_DLY();
    scl_write(0u);
}

static void soft_stop(void)
{
    scl_write(0u);
    sda_write(0u);
    SOFT_DLY();
    scl_write(1u);
    SOFT_DLY();
    sda_write(1u);          /* SCL 高电平期间 SDA 上升 */
    SOFT_DLY();
}

static uint8_t soft_write_byte(uint8_t b)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8u; i++) {
        sda_write((b & 0x80u) ? 1u : 0u);
        b = (uint8_t)(b << 1);
        SOFT_DLY();
        scl_write(1u);
        SOFT_DLY();
        scl_write(0u);
        SOFT_DLY();
    }

    sda_write(1u);          /* 释放 SDA，等从机应答 */
    SOFT_DLY();
    scl_write(1u);
    SOFT_DLY();
    ack = (sda_read() == 0u) ? 1u : 0u;
    scl_write(0u);
    SOFT_DLY();
    return ack;
}

static uint8_t soft_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t v = 0;

    sda_write(1u);          /* 释放 SDA */
    for (i = 0; i < 8u; i++) {
        SOFT_DLY();
        scl_write(1u);
        SOFT_DLY();
        v = (uint8_t)((v << 1) | sda_read());
        scl_write(0u);
    }

    sda_write(ack ? 0u : 1u);
    SOFT_DLY();
    scl_write(1u);
    SOFT_DLY();
    scl_write(0u);
    SOFT_DLY();
    return v;
}

static int soft_i2c_write(void *ctx, uint8_t addr7, uint8_t reg,
                          const uint8_t *data, uint16_t len)
{
    uint16_t i;

    (void)ctx;
    if (data == NULL || len == 0u) {
        return -1;
    }

    soft_start();
    if (!soft_write_byte((uint8_t)(addr7 << 1))) {
        goto fail;
    }
    if (!soft_write_byte(reg)) {
        goto fail;
    }
    for (i = 0; i < len; i++) {
        if (!soft_write_byte(data[i])) {
            goto fail;
        }
    }
    soft_stop();
    return 0;

fail:
    soft_stop();
    return -1;
}

static int soft_i2c_read(void *ctx, uint8_t addr7, uint8_t reg,
                         uint8_t *data, uint16_t len)
{
    uint16_t i;

    (void)ctx;
    if (data == NULL || len == 0u) {
        return -1;
    }

    soft_start();
    if (!soft_write_byte((uint8_t)(addr7 << 1))) {
        goto fail;
    }
    if (!soft_write_byte(reg)) {
        goto fail;
    }

    soft_start();                                   /* 重复起始 */
    if (!soft_write_byte((uint8_t)((addr7 << 1) | 0x01u))) {
        goto fail;
    }
    for (i = 0; i < len; i++) {
        data[i] = soft_read_byte((uint8_t)((i + 1u) < len));
    }
    soft_stop();
    return 0;

fail:
    soft_stop();
    return -1;
}

#endif /* IMU_STM32_USE_SOFT_I2C */

/* ========================================================================= */
/* 对外接口                                                                  */
/* ========================================================================= */

static imu_port_t s_port = {
    NULL, NULL, NULL, NULL, port_delay_ms, NULL, port_millis, NULL
};

void imu_port_stm32_init(void *i2c_handle)
{
#if IMU_STM32_USE_SOFT_I2C
    (void)i2c_handle;
    soft_i2c_gpio_init();
    dwt_init();
    soft_bus_recover();

    s_port.ctx       = NULL;
    s_port.i2c_write = soft_i2c_write;
    s_port.i2c_read  = soft_i2c_read;
    s_port.spi_xfer  = NULL;
    s_port.delay_ms  = port_delay_ms;
    s_port.delay_us  = port_delay_us;
    s_port.millis    = port_millis;
    s_port.log       = NULL;
#else
    s_port.ctx       = i2c_handle;
    s_port.i2c_write = hal_i2c_write;
    s_port.i2c_read  = hal_i2c_read;
    s_port.spi_xfer  = NULL;
    s_port.delay_ms  = port_delay_ms;
    s_port.delay_us  = NULL;
    s_port.millis    = port_millis;
    s_port.log       = NULL;
#endif
}

const imu_port_t *imu_port_stm32_get(void)
{
    return &s_port;
}

void imu_port_stm32_bus_recover(void)
{
#if IMU_STM32_USE_SOFT_I2C
    soft_bus_recover();
#else
    /* 硬件 I2C：复位外设即可清除 AF/BUSY（也规避 F4 的 BUSY 卡死勘误）；
     * 若 SDA 被从机拉死，需要先把引脚切到 GPIO 手动打 9 个时钟，
     * 具体实现取决于板级引脚定义，可参考软件 I2C 部分。 */
    if (s_port.ctx != NULL) {
        hal_i2c_recover((I2C_HandleTypeDef *)s_port.ctx);
    }
#endif
}
