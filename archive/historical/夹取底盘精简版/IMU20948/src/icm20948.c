/**
 * @file    icm20948.c
 * @brief   ICM-20948 九轴 IMU 驱动实现（可移植）
 *
 * 参考：WHEELTEC L150Pro 工程 HARDWARE/ICM20948/ICM20948.c
 *      ICM-20948 Datasheet / AK09916 Datasheet
 *
 * 本文件不包含任何 MCU 相关的头文件，所有硬件动作都通过 imu_port_t 完成。
 */
#include "icm20948.h"

#include <string.h>

/* SPI 一次传输的最大长度：1 字节命令 + N 字节数据 */
#define ICM20948_SPI_BUF_MAX 24u

/* 复位后等待时间（数据手册要求 >= 100ms） */
#define ICM20948_RESET_DELAY_MS 100u
/* 唤醒/时钟稳定时间 */
#define ICM20948_WAKE_DELAY_MS  10u

/* 量程换算系数 */
static const float s_accel_lsb_per_g[4]   = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float s_gyro_lsb_per_dps[4]  = { 131.0f,  65.5f,   32.8f,   16.4f  };

/* ------------------------------------------------------------------------- */
/* 底层总线                                                                  */
/* ------------------------------------------------------------------------- */

static int bus_write(icm20948_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    const imu_port_t *p = dev->port;

    if (dev->cfg.bus == ICM20948_BUS_I2C) {
        if (p->i2c_write == NULL) {
            return ICM20948_ERR_PORT;
        }
        return (p->i2c_write(p->ctx, dev->cfg.i2c_addr7, reg, data, len) == 0)
                   ? ICM20948_OK : ICM20948_ERR_BUS;
    }

    if (p->spi_xfer == NULL) {
        return ICM20948_ERR_PORT;
    }
    if ((uint32_t)len + 1u > ICM20948_SPI_BUF_MAX) {
        return ICM20948_ERR_PARAM;
    }
    {
        uint8_t tx[ICM20948_SPI_BUF_MAX];
        tx[0] = (uint8_t)(reg & 0x7Fu);          /* 0 = 写 */
        if (len > 0u) {
            memcpy(&tx[1], data, len);
        }
        return (p->spi_xfer(p->ctx, tx, NULL, (uint16_t)(len + 1u)) == 0)
                   ? ICM20948_OK : ICM20948_ERR_BUS;
    }
}

static int bus_read(icm20948_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    const imu_port_t *p = dev->port;

    if (dev->cfg.bus == ICM20948_BUS_I2C) {
        if (p->i2c_read == NULL) {
            return ICM20948_ERR_PORT;
        }
        return (p->i2c_read(p->ctx, dev->cfg.i2c_addr7, reg, data, len) == 0)
                   ? ICM20948_OK : ICM20948_ERR_BUS;
    }

    if (p->spi_xfer == NULL) {
        return ICM20948_ERR_PORT;
    }
    if ((uint32_t)len + 1u > ICM20948_SPI_BUF_MAX) {
        return ICM20948_ERR_PARAM;
    }
    {
        uint8_t tx[ICM20948_SPI_BUF_MAX];
        uint8_t rx[ICM20948_SPI_BUF_MAX];
        memset(tx, 0, sizeof(tx));
        memset(rx, 0, sizeof(rx));
        tx[0] = (uint8_t)(reg | 0x80u);          /* bit7 = 1 表示读 */
        if (p->spi_xfer(p->ctx, tx, rx, (uint16_t)(len + 1u)) != 0) {
            return ICM20948_ERR_BUS;
        }
        memcpy(data, &rx[1], len);
        return ICM20948_OK;
    }
}

/* ------------------------------------------------------------------------- */
/* 寄存器访问（带 bank 缓存）                                                */
/* ------------------------------------------------------------------------- */

static int write_bank_sel(icm20948_t *dev, uint8_t bank)
{
    uint8_t v = (uint8_t)(bank << 4);
    int rc = bus_write(dev, ICM20948_REG_BANK_SEL, &v, 1u);
    if (rc == ICM20948_OK) {
        dev->bank = bank;
    }
    return rc;
}

static int select_bank(icm20948_t *dev, uint8_t bank)
{
    if (dev->bank == bank) {
        return ICM20948_OK;
    }
    return write_bank_sel(dev, bank);
}

static int reg_write(icm20948_t *dev, uint8_t bank, uint8_t reg, uint8_t val)
{
    int rc = select_bank(dev, bank);
    if (rc != ICM20948_OK) {
        return rc;
    }
    return bus_write(dev, reg, &val, 1u);
}

static int reg_read(icm20948_t *dev, uint8_t bank, uint8_t reg, uint8_t *val)
{
    int rc = select_bank(dev, bank);
    if (rc != ICM20948_OK) {
        return rc;
    }
    return bus_read(dev, reg, val, 1u);
}

static int reg_update(icm20948_t *dev, uint8_t bank, uint8_t reg,
                      uint8_t mask, uint8_t val)
{
    uint8_t v = 0;
    int rc = reg_read(dev, bank, reg, &v);
    if (rc != ICM20948_OK) {
        return rc;
    }
    v = (uint8_t)((v & (uint8_t)~mask) | (val & mask));
    return reg_write(dev, bank, reg, v);
}

/* ------------------------------------------------------------------------- */
/* 配置校验                                                                  */
/* ------------------------------------------------------------------------- */

static bool cfg_is_valid(const icm20948_cfg_t *c)
{
    uint8_t i, j;

    if (c->bus == ICM20948_BUS_I2C) {
        if (c->i2c_addr7 > 0x7Fu) {
            return false;
        }
    } else if (c->bus != ICM20948_BUS_SPI) {
        return false;
    }

    if ((uint32_t)c->accel_fs > (uint32_t)ICM20948_ACCEL_FS_16G) {
        return false;
    }
    if ((uint32_t)c->gyro_fs > (uint32_t)ICM20948_GYRO_FS_2000DPS) {
        return false;
    }
    if (c->accel_dlpf > ICM20948_DLPF_CFG_MAX ||
        c->gyro_dlpf  > ICM20948_DLPF_CFG_MAX) {
        return false;
    }
    if (c->avg_samples > ICM20948_MAX_AVG_SAMPLES) {
        return false;
    }
    if (c->gyro_calib_samples > ICM20948_GYRO_CALIB_MAX) {
        return false;
    }

    /* 轴系映射：绝对值必须正好是 {1,2,3} 的一个排列 */
    for (i = 0; i < 3u; i++) {
        int8_t v = c->axis_map[i];
        if (v == 0 || v == -128) {
            return false;
        }
        if ((v < 0 ? -v : v) > 3) {
            return false;
        }
        for (j = (uint8_t)(i + 1u); j < 3u; j++) {
            int8_t w = c->axis_map[j];
            if (w == -128) {
                return false;
            }
            if ((v < 0 ? -v : v) == (w < 0 ? -w : w)) {
                return false;
            }
        }
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* 初始化序列                                                                */
/* ------------------------------------------------------------------------- */

void icm20948_default_cfg(icm20948_cfg_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->bus                 = ICM20948_BUS_I2C;
    cfg->i2c_addr7           = ICM20948_I2C_ADDR_AD0_LOW;   /* 0x68 == 原工程 0xD0 >> 1 */
    cfg->accel_fs            = ICM20948_ACCEL_FS_2G;
    cfg->gyro_fs             = ICM20948_GYRO_FS_500DPS;
    cfg->accel_dlpf          = 6u;    /* 5.7Hz，与原工程 REG_VAL_BIT_ACCEL_DLPCFG_6 一致 */
    cfg->gyro_dlpf           = 6u;    /* 5.7Hz，与原工程 REG_VAL_BIT_GYRO_DLPCFG_6 一致 */
    cfg->accel_smplrt_div    = 7u;    /* ODR ≈ 140Hz，与原工程 0x07 一致 */
    cfg->gyro_smplrt_div     = 7u;
    cfg->enable_mag          = true;
    cfg->avg_samples         = 8u;    /* 与原工程 8 点滑动平均一致；追求低延迟可改为 0 */
    cfg->enable_drdy_int     = false; /* 需要 INT 引脚触发时再打开 */
    cfg->int_active_low      = false; /* 默认高有效推挽，对应 EXTI 上升沿 */
    cfg->auto_gyro_calib     = true;
    cfg->gyro_calib_samples  = 200u;
    cfg->gyro_calib_period_ms= 5u;
    cfg->axis_map[0]         = 1;
    cfg->axis_map[1]         = 2;
    cfg->axis_map[2]         = 3;
}

int icm20948_attach(icm20948_t *dev, const imu_port_t *port, const icm20948_cfg_t *cfg)
{
    icm20948_cfg_t tmp;

    if (dev == NULL || port == NULL) {
        return ICM20948_ERR_PARAM;
    }
    if (port->delay_ms == NULL) {
        return ICM20948_ERR_PORT;
    }
    if (port->i2c_read == NULL && port->i2c_write == NULL &&
        port->spi_xfer == NULL) {
        return ICM20948_ERR_PORT;
    }

    if (cfg != NULL) {
        tmp = *cfg;
    } else {
        icm20948_default_cfg(&tmp);
    }
    if (!cfg_is_valid(&tmp)) {
        return ICM20948_ERR_PARAM;
    }
    if (tmp.bus == ICM20948_BUS_I2C &&
        (port->i2c_read == NULL || port->i2c_write == NULL)) {
        return ICM20948_ERR_PORT;
    }
    if (tmp.bus == ICM20948_BUS_SPI && port->spi_xfer == NULL) {
        return ICM20948_ERR_PORT;
    }

    memset(dev, 0, sizeof(*dev));
    dev->port = port;
    dev->cfg  = tmp;
    dev->bank = 0xFFu;          /* bank 未知，首次访问强制写 BANK_SEL */
    return ICM20948_OK;
}

int icm20948_read_id(icm20948_t *dev, uint8_t *id)
{
    if (dev == NULL || dev->port == NULL || id == NULL) {
        return ICM20948_ERR_PARAM;
    }
    dev->bank = 0xFFu;          /* 强制重新选择 bank0 */
    return reg_read(dev, ICM20948_BANK_0, ICM20948_REG_WHO_AM_I, id);
}

int icm20948_probe(icm20948_t *dev, uint8_t *id)
{
    uint8_t v = 0;
    int rc;

    if (dev == NULL || dev->port == NULL) {
        return ICM20948_ERR_PARAM;
    }
    rc = icm20948_read_id(dev, &v);
    if (rc != ICM20948_OK) {
        return rc;
    }
    if (id != NULL) {
        *id = v;
    }
    if (v != ICM20948_WHO_AM_I_VAL) {
        return ICM20948_ERR_WHO_AM_I;
    }
    return ICM20948_OK;
}

int icm20948_reset(icm20948_t *dev)
{
    int rc;

    if (dev == NULL || dev->port == NULL) {
        return ICM20948_ERR_PARAM;
    }

    rc = reg_write(dev, ICM20948_BANK_0, ICM20948_REG_PWR_MGMT_1,
                   ICM20948_BIT_DEVICE_RESET);
    if (rc != ICM20948_OK) {
        return rc;
    }

    /* 复位后器件内部会重启，bank 选择清零，需要重新建立 */
    dev->bank = 0xFFu;
    dev->port->delay_ms(dev->port->ctx, ICM20948_RESET_DELAY_MS);

    /* 唤醒并选择自动时钟源 */
    rc = reg_write(dev, ICM20948_BANK_0, ICM20948_REG_PWR_MGMT_1,
                   ICM20948_BIT_CLKSEL_AUTO);
    if (rc != ICM20948_OK) {
        return rc;
    }
    dev->port->delay_ms(dev->port->ctx, ICM20948_WAKE_DELAY_MS);
    return ICM20948_OK;
}

int icm20948_sleep(icm20948_t *dev, bool enable)
{
    int rc;

    if (dev == NULL || dev->port == NULL) {
        return ICM20948_ERR_PARAM;
    }
    rc = reg_update(dev, ICM20948_BANK_0, ICM20948_REG_PWR_MGMT_1,
                    ICM20948_BIT_SLEEP,
                    enable ? ICM20948_BIT_SLEEP : 0u);
    if (rc == ICM20948_OK && !enable) {
        dev->port->delay_ms(dev->port->ctx, ICM20948_WAKE_DELAY_MS);
    }
    return rc;
}

/**
 * @brief 通过 ICM20948 的 I2C 主机（SLV0）读外部从机（AK09916）
 * @note  仅用于初始化阶段的探测，运行期由连续采样模式自动搬运数据
 */
static int read_secondary(icm20948_t *dev, uint8_t addr7, uint8_t reg,
                          uint8_t len, uint8_t *buf)
{
    uint8_t len_mask = (uint8_t)(len & ICM20948_BIT_SLV_LEN_MASK);
    int rc;

    if (len_mask == 0u) {
        return ICM20948_ERR_PARAM;
    }

    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_ADDR,
                   (uint8_t)(addr7 | ICM20948_BIT_SLV_RNW));
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_REG, reg);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_CTRL,
                   (uint8_t)(ICM20948_BIT_SLV_EN | len_mask));
    if (rc != ICM20948_OK) { return rc; }

    /* 打开 I2C 主机，等待一次搬运完成后关闭 */
    rc = reg_update(dev, ICM20948_BANK_0, ICM20948_REG_USER_CTRL,
                    ICM20948_BIT_I2C_MST_EN, ICM20948_BIT_I2C_MST_EN);
    if (rc != ICM20948_OK) { return rc; }
    dev->port->delay_ms(dev->port->ctx, 5u);

    rc = reg_update(dev, ICM20948_BANK_0, ICM20948_REG_USER_CTRL,
                    ICM20948_BIT_I2C_MST_EN, 0u);
    if (rc != ICM20948_OK) { return rc; }

    rc = select_bank(dev, ICM20948_BANK_0);
    if (rc != ICM20948_OK) { return rc; }
    rc = bus_read(dev, ICM20948_REG_EXT_SENS_DATA_00, buf, len);
    if (rc != ICM20948_OK) { return rc; }

    /* 关闭 SLV0，保持总线干净 */
    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_CTRL, 0x00u);
    if (rc != ICM20948_OK) { return rc; }
    return select_bank(dev, ICM20948_BANK_0);
}

/**
 * @brief 通过 ICM20948 的 I2C 主机（SLV1）写外部从机（AK09916）单个寄存器
 */
static int write_secondary(icm20948_t *dev, uint8_t addr7, uint8_t reg, uint8_t data)
{
    int rc;

    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV1_ADDR,
                   (uint8_t)(addr7 & ~ICM20948_BIT_SLV_RNW));
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV1_REG, reg);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV1_DO, data);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV1_CTRL,
                   (uint8_t)(ICM20948_BIT_SLV_EN | 0x01u));
    if (rc != ICM20948_OK) { return rc; }

    rc = reg_update(dev, ICM20948_BANK_0, ICM20948_REG_USER_CTRL,
                    ICM20948_BIT_I2C_MST_EN, ICM20948_BIT_I2C_MST_EN);
    if (rc != ICM20948_OK) { return rc; }
    dev->port->delay_ms(dev->port->ctx, 5u);

    rc = reg_update(dev, ICM20948_BANK_0, ICM20948_REG_USER_CTRL,
                    ICM20948_BIT_I2C_MST_EN, 0u);
    if (rc != ICM20948_OK) { return rc; }

    /* 清除 SLV1 使能（原工程此处误写了 SLV0，已修正） */
    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV1_CTRL, 0x00u);
    if (rc != ICM20948_OK) { return rc; }
    return select_bank(dev, ICM20948_BANK_0);
}

/**
 * @brief 初始化 AK09916：校验 ID -> 软复位 -> 连续 100Hz -> 配置 SLV0 自动搬运
 */
static int mag_init(icm20948_t *dev)
{
    uint8_t wia[2] = { 0, 0 };
    int rc;

    rc = read_secondary(dev, AK09916_I2C_ADDR, AK09916_REG_WIA1, 2u, wia);
    if (rc != ICM20948_OK) {
        return rc;
    }
    if (wia[0] != AK09916_WIA1_VAL || wia[1] != AK09916_WIA2_VAL) {
        return ICM20948_ERR_MAG;
    }

    rc = write_secondary(dev, AK09916_I2C_ADDR, AK09916_REG_CNTL3,
                         AK09916_SOFT_RESET);
    if (rc != ICM20948_OK) {
        return rc;
    }
    dev->port->delay_ms(dev->port->ctx, 1u);

    rc = write_secondary(dev, AK09916_I2C_ADDR, AK09916_REG_CNTL2,
                         AK09916_MODE_CONT_100HZ);
    if (rc != ICM20948_OK) {
        return rc;
    }
    dev->port->delay_ms(dev->port->ctx, 1u);

    /* SLV0 连续读取：ST1 + 6 字节磁场 + TMPS + ST2，共 9 字节 */
    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_ADDR,
                   (uint8_t)(AK09916_I2C_ADDR | ICM20948_BIT_SLV_RNW));
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_REG,
                   AK09916_REG_ST1);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_CTRL,
                   (uint8_t)(ICM20948_BIT_SLV_EN | 9u));
    if (rc != ICM20948_OK) { return rc; }

    /* 常开 I2C 主机，让 SLV0 每个采样周期自动搬运 */
    rc = reg_update(dev, ICM20948_BANK_0, ICM20948_REG_USER_CTRL,
                    ICM20948_BIT_I2C_MST_EN, ICM20948_BIT_I2C_MST_EN);
    if (rc != ICM20948_OK) { return rc; }

    dev->port->delay_ms(dev->port->ctx, 10u);
    dev->mag_ok = true;
    return ICM20948_OK;
}

/** 配置加速度计 / 陀螺仪（bank2） */
static int sensors_config(icm20948_t *dev)
{
    const icm20948_cfg_t *c = &dev->cfg;
    uint8_t accel_cfg, gyro_cfg;
    int rc;

    gyro_cfg = (uint8_t)((c->gyro_dlpf  << ICM20948_DLPFCFG_SHIFT) |
                         ((uint8_t)c->gyro_fs  << ICM20948_FS_SHIFT) |
                         ICM20948_BIT_DLPF_ENABLE);
    accel_cfg = (uint8_t)((c->accel_dlpf << ICM20948_DLPFCFG_SHIFT) |
                          ((uint8_t)c->accel_fs << ICM20948_FS_SHIFT) |
                          ICM20948_BIT_DLPF_ENABLE);

    rc = reg_write(dev, ICM20948_BANK_2, ICM20948_REG_GYRO_SMPLRT_DIV,
                   c->gyro_smplrt_div);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_2, ICM20948_REG_GYRO_CONFIG_1, gyro_cfg);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_2, ICM20948_REG_GYRO_CONFIG_2, 0x00u);
    if (rc != ICM20948_OK) { return rc; }

    rc = reg_write(dev, ICM20948_BANK_2, ICM20948_REG_ACCEL_SMPLRT_DIV_1, 0x00u);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_2, ICM20948_REG_ACCEL_SMPLRT_DIV_2,
                   c->accel_smplrt_div);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_2, ICM20948_REG_ACCEL_CONFIG, accel_cfg);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_2, ICM20948_REG_ACCEL_CONFIG_2, 0x00u);
    if (rc != ICM20948_OK) { return rc; }

    /* 使能 ODR 对齐，保证陀螺/加计采样在同一时刻，减小姿态解算的相位误差 */
    rc = reg_write(dev, ICM20948_BANK_2, ICM20948_REG_ODR_ALIGN_EN, 0x01u);
    if (rc != ICM20948_OK) { return rc; }

    return select_bank(dev, ICM20948_BANK_0);
}

int icm20948_init(icm20948_t *dev, const imu_port_t *port, const icm20948_cfg_t *cfg)
{
    uint8_t id = 0;
    int rc;

    rc = icm20948_attach(dev, port, cfg);
    if (rc != ICM20948_OK) {
        return rc;
    }

    /* 上电后至少等待 10ms 再访问总线 */
    dev->port->delay_ms(dev->port->ctx, 10u);

    /* 1. 先确认器件在位，避免对错误的芯片执行复位 */
    rc = icm20948_probe(dev, &id);
    if (rc != ICM20948_OK) {
        return rc;
    }

    /* 2. 软复位 + 唤醒 */
    rc = icm20948_reset(dev);
    if (rc != ICM20948_OK) {
        return rc;
    }

    /* 3. 复位后再确认一次（掉电复位、总线干扰等） */
    rc = icm20948_probe(dev, &id);
    if (rc != ICM20948_OK) {
        return rc;
    }

    /* 4. 关闭 FIFO / DMP，仅使用直接寄存器读取 */
    rc = reg_write(dev, ICM20948_BANK_0, ICM20948_REG_USER_CTRL, 0x00u);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_0, ICM20948_REG_FIFO_EN_1, 0x00u);
    if (rc != ICM20948_OK) { return rc; }
    rc = reg_write(dev, ICM20948_BANK_0, ICM20948_REG_FIFO_EN_2, 0x00u);
    if (rc != ICM20948_OK) { return rc; }

    /* 5. 量程 / 滤波 / 采样率 */
    rc = sensors_config(dev);
    if (rc != ICM20948_OK) {
        return rc;
    }

    /* 5.5 数据就绪中断（可选）：把 RAW_DATA_RDY 输出到 INT1 引脚 */
    if (dev->cfg.enable_drdy_int) {
        (void)icm20948_enable_drdy_int(dev, true);
    }

    /* 6. 磁力计（可选） */
    dev->mag_ok = false;
    if (dev->cfg.enable_mag) {
        rc = mag_init(dev);
        if (rc != ICM20948_OK) {
            /* 磁力计失败不阻断六轴功能，交由上层决定是否降级使用 */
            dev->mag_ok = false;
        }
    }

    dev->initialized = true;

    /* 7. 陀螺零偏标定（此时必须静止） */
    if (dev->cfg.auto_gyro_calib && dev->cfg.gyro_calib_samples > 0u) {
        (void)icm20948_calibrate_gyro(dev, dev->cfg.gyro_calib_samples,
                                      dev->cfg.gyro_calib_period_ms);
    }

    return ICM20948_OK;
}

/* ------------------------------------------------------------------------- */
/* 数据读取                                                                  */
/* ------------------------------------------------------------------------- */

static int16_t raw_avg_push(icm20948_t *dev, uint8_t axis, int16_t val)
{
    uint8_t n = dev->cfg.avg_samples;
    uint8_t i;
    int32_t sum = 0;

    if (n < 2u) {
        return val;
    }
    if (n > ICM20948_MAX_AVG_SAMPLES) {
        n = ICM20948_MAX_AVG_SAMPLES;
    }

    dev->avg_buf[axis][dev->avg_idx] = val;
    for (i = 0; i < n; i++) {
        sum += dev->avg_buf[axis][i];
    }

    /* 尚未填满时按实际点数求平均，避免启动阶段的假数据 */
    if (dev->avg_fill < n) {
        return (int16_t)(sum / (int32_t)(dev->avg_fill + 1u));
    }
    return (int16_t)((sum + (int32_t)(n / 2u)) / (int32_t)n);
}

int icm20948_read_raw(icm20948_t *dev, icm20948_sample_t *out)
{
    uint8_t b[12];
    uint8_t t[2];
    int16_t acc[3], gyr[3];
    int rc;

    if (dev == NULL || out == NULL) {
        return ICM20948_ERR_PARAM;
    }
    if (dev->port == NULL) {
        return ICM20948_ERR_PARAM;
    }

    memset(out, 0, sizeof(*out));

    /* 一次突发读 12 字节：ACCEL XYZ + GYRO XYZ（寄存器自动递增） */
    rc = select_bank(dev, ICM20948_BANK_0);
    if (rc != ICM20948_OK) {
        dev->error_count++;
        return rc;
    }
    rc = bus_read(dev, ICM20948_REG_ACCEL_XOUT_H, b, 12u);
    if (rc != ICM20948_OK) {
        dev->error_count++;
        return rc;
    }

    acc[0] = (int16_t)(((uint16_t)b[0]  << 8) | b[1]);
    acc[1] = (int16_t)(((uint16_t)b[2]  << 8) | b[3]);
    acc[2] = (int16_t)(((uint16_t)b[4]  << 8) | b[5]);
    gyr[0] = (int16_t)(((uint16_t)b[6]  << 8) | b[7]);
    gyr[1] = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
    gyr[2] = (int16_t)(((uint16_t)b[10] << 8) | b[11]);

    rc = bus_read(dev, ICM20948_REG_TEMP_OUT_H, t, 2u);
    if (rc != ICM20948_OK) {
        dev->error_count++;
        return rc;
    }
    out->temp_raw = (int16_t)(((uint16_t)t[0] << 8) | t[1]);

    out->accel_raw[0] = raw_avg_push(dev, 0u, acc[0]);
    out->accel_raw[1] = raw_avg_push(dev, 1u, acc[1]);
    out->accel_raw[2] = raw_avg_push(dev, 2u, acc[2]);
    out->gyro_raw[0]  = raw_avg_push(dev, 3u, gyr[0]);
    out->gyro_raw[1]  = raw_avg_push(dev, 4u, gyr[1]);
    out->gyro_raw[2]  = raw_avg_push(dev, 5u, gyr[2]);

    if (dev->cfg.avg_samples >= 2u) {
        uint8_t n = dev->cfg.avg_samples;
        if (n > ICM20948_MAX_AVG_SAMPLES) {
            n = ICM20948_MAX_AVG_SAMPLES;
        }
        if (dev->avg_fill < n) {
            dev->avg_fill++;
        }
        dev->avg_idx = (uint8_t)((dev->avg_idx + 1u) % n);
    }

    dev->sample_count++;
    if (dev->port->millis != NULL) {
        out->timestamp_ms = dev->port->millis(dev->port->ctx);
    }
    return ICM20948_OK;
}

int icm20948_read(icm20948_t *dev, icm20948_sample_t *out)
{
    int rc;

    if (dev == NULL || out == NULL) {
        return ICM20948_ERR_PARAM;
    }

    rc = icm20948_read_raw(dev, out);
    if (rc != ICM20948_OK) {
        return rc;
    }

    {
        uint8_t i;
        const float a_lsb = s_accel_lsb_per_g[(uint32_t)dev->cfg.accel_fs];
        const float g_lsb = s_gyro_lsb_per_dps[(uint32_t)dev->cfg.gyro_fs];

        for (i = 0; i < 3u; i++) {
            int8_t map = dev->cfg.axis_map[i];
            uint8_t src = (uint8_t)((map < 0 ? -map : map) - 1);
            float sign = (map < 0) ? -1.0f : 1.0f;
            float a = (float)out->accel_raw[src] / a_lsb;
            float g = (float)out->gyro_raw[src] / g_lsb - dev->gyro_bias_dps[src];
            out->accel_g[i]  = sign * a;
            out->gyro_dps[i] = sign * g;
        }
        out->temp_c = ((float)out->temp_raw / 326.8f) + 25.0f;
    }
    return ICM20948_OK;
}

int icm20948_read_mag(icm20948_t *dev, float mag_uT[3])
{
    uint8_t b[9];
    int16_t mx, my, mz;
    int rc;

    if (dev == NULL || mag_uT == NULL) {
        return ICM20948_ERR_PARAM;
    }
    if (!dev->mag_ok) {
        return ICM20948_ERR_UNSUPPORTED;
    }

    rc = select_bank(dev, ICM20948_BANK_0);
    if (rc != ICM20948_OK) {
        return rc;
    }
    rc = bus_read(dev, ICM20948_REG_EXT_SENS_DATA_00, b, 9u);
    if (rc != ICM20948_OK) {
        dev->error_count++;
        return rc;
    }

    /* b[0]=ST1 b[1..6]=HXL..HZH b[7]=TMPS b[8]=ST2 */
    if ((b[0] & AK09916_BIT_DRDY) == 0u) {
        return ICM20948_ERR_NO_DATA;
    }

    /* AK09916 数据为小端 */
    mx = (int16_t)(((uint16_t)b[2] << 8) | b[1]);
    my = (int16_t)(((uint16_t)b[4] << 8) | b[3]);
    mz = (int16_t)(((uint16_t)b[6] << 8) | b[5]);

    mag_uT[0] = (float)mx * AK09916_UT_PER_LSB - dev->mag_offset_uT[0];
    mag_uT[1] = (float)my * AK09916_UT_PER_LSB - dev->mag_offset_uT[1];
    mag_uT[2] = (float)mz * AK09916_UT_PER_LSB - dev->mag_offset_uT[2];
    return ICM20948_OK;
}

bool icm20948_mag_available(const icm20948_t *dev)
{
    return (dev != NULL) && dev->mag_ok;
}

int icm20948_enable_drdy_int(icm20948_t *dev, bool enable)
{
    uint8_t pin_cfg;
    int rc;

    if (dev == NULL || dev->port == NULL) {
        return ICM20948_ERR_PARAM;
    }

    /* 任意寄存器读即清中断：保证 INT 是短脉冲，适合 EXTI 边沿触发 */
    pin_cfg = ICM20948_BIT_INT_ANYRD_2CLEAR;
    if (dev->cfg.int_active_low) {
        pin_cfg |= (uint8_t)(ICM20948_BIT_INT1_ACTL | ICM20948_BIT_INT1_OPEN);
    }

    rc = reg_write(dev, ICM20948_BANK_0, ICM20948_REG_INT_PIN_CFG, pin_cfg);
    if (rc != ICM20948_OK) {
        return rc;
    }
    return reg_write(dev, ICM20948_BANK_0, ICM20948_REG_INT_ENABLE_1,
                     enable ? ICM20948_BIT_RAW_DATA_RDY_EN : 0x00u);
}

int icm20948_data_ready(icm20948_t *dev, bool *ready)
{
    uint8_t v = 0;
    int rc;

    if (dev == NULL || ready == NULL) {
        return ICM20948_ERR_PARAM;
    }
    rc = reg_read(dev, ICM20948_BANK_0, ICM20948_REG_INT_STATUS_1, &v);
    if (rc != ICM20948_OK) {
        return rc;
    }
    *ready = ((v & ICM20948_BIT_RAW_DATA_RDY) != 0u);
    return ICM20948_OK;
}

/* ------------------------------------------------------------------------- */
/* 标定与参数                                                                */
/* ------------------------------------------------------------------------- */

int icm20948_calibrate_gyro(icm20948_t *dev, uint16_t samples, uint32_t period_ms)
{
    float sum[3] = { 0.0f, 0.0f, 0.0f };
    float lsb;
    uint16_t i;
    int rc;

    if (dev == NULL || dev->port == NULL) {
        return ICM20948_ERR_PARAM;
    }
    if (samples == 0u) {
        return ICM20948_ERR_PARAM;
    }
    if (samples > ICM20948_GYRO_CALIB_MAX) {
        samples = ICM20948_GYRO_CALIB_MAX;
    }

    lsb = s_gyro_lsb_per_dps[(uint32_t)dev->cfg.gyro_fs];

    /* 用原始值累加，避免标定过程中叠加旧的零偏 */
    for (i = 0; i < samples; i++) {
        icm20948_sample_t s;
        rc = icm20948_read_raw(dev, &s);
        if (rc != ICM20948_OK) {
            return rc;
        }
        sum[0] += (float)s.gyro_raw[0];
        sum[1] += (float)s.gyro_raw[1];
        sum[2] += (float)s.gyro_raw[2];
        if (period_ms > 0u) {
            dev->port->delay_ms(dev->port->ctx, period_ms);
        }
    }

    dev->gyro_bias_dps[0] = sum[0] / (float)samples / lsb;
    dev->gyro_bias_dps[1] = sum[1] / (float)samples / lsb;
    dev->gyro_bias_dps[2] = sum[2] / (float)samples / lsb;
    return ICM20948_OK;
}

void icm20948_set_gyro_bias(icm20948_t *dev, const float bias_dps[3])
{
    if (dev == NULL || bias_dps == NULL) {
        return;
    }
    dev->gyro_bias_dps[0] = bias_dps[0];
    dev->gyro_bias_dps[1] = bias_dps[1];
    dev->gyro_bias_dps[2] = bias_dps[2];
}

void icm20948_get_gyro_bias(const icm20948_t *dev, float bias_dps[3])
{
    if (dev == NULL || bias_dps == NULL) {
        return;
    }
    bias_dps[0] = dev->gyro_bias_dps[0];
    bias_dps[1] = dev->gyro_bias_dps[1];
    bias_dps[2] = dev->gyro_bias_dps[2];
}

void icm20948_set_mag_offset(icm20948_t *dev, const float offset_uT[3])
{
    if (dev == NULL || offset_uT == NULL) {
        return;
    }
    dev->mag_offset_uT[0] = offset_uT[0];
    dev->mag_offset_uT[1] = offset_uT[1];
    dev->mag_offset_uT[2] = offset_uT[2];
}

const char *icm20948_strerror(int err)
{
    switch (err) {
    case ICM20948_OK:              return "ok";
    case ICM20948_ERR_PARAM:       return "invalid parameter";
    case ICM20948_ERR_PORT:        return "port callback missing";
    case ICM20948_ERR_WHO_AM_I:    return "who_am_i mismatch";
    case ICM20948_ERR_BUS:         return "bus transfer failed";
    case ICM20948_ERR_MAG:         return "ak09916 not found";
    case ICM20948_ERR_NO_DATA:     return "no magnetic data";
    case ICM20948_ERR_UNSUPPORTED: return "feature not enabled";
    default:                       return "unknown error";
    }
}
