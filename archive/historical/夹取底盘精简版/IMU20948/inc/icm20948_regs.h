/**
 * @file    icm20948_regs.h
 * @brief   ICM-20948（含 AK09916 磁力计）寄存器定义
 *
 * 数据手册：ICM-20948 Datasheet (TDK InvenSense)
 * 寄存器分为 4 个 bank，通过 BANK_SEL(0x7F) 切换：
 *      BANK_SEL[5:4] = 0 -> bank0, 1 -> bank1, 2 -> bank2, 3 -> bank3
 */
#ifndef ICM20948_REGS_H
#define ICM20948_REGS_H

/* ---------- 器件地址 ---------- */
#define ICM20948_WHO_AM_I_VAL      0xEAu   /* WHO_AM_I 固定返回值 */
#define ICM20948_I2C_ADDR_AD0_LOW  0x68u   /* AD0 = 0（7 位地址） */
#define ICM20948_I2C_ADDR_AD0_HIGH 0x69u   /* AD0 = 1（7 位地址） */

#define AK09916_I2C_ADDR           0x0Cu   /* AK09916 7 位地址 */
#define AK09916_WIA1_VAL           0x48u
#define AK09916_WIA2_VAL           0x09u

/* ---------- 通用 ---------- */
#define ICM20948_REG_BANK_SEL      0x7Fu
#define ICM20948_BANK_0            0x00u
#define ICM20948_BANK_1            0x01u
#define ICM20948_BANK_2            0x02u
#define ICM20948_BANK_3            0x03u

/* ==================== BANK 0 ==================== */
#define ICM20948_REG_WHO_AM_I           0x00u
#define ICM20948_REG_USER_CTRL          0x03u
#define ICM20948_REG_LP_CONFIG          0x05u
#define ICM20948_REG_PWR_MGMT_1         0x06u
#define ICM20948_REG_PWR_MGMT_2         0x07u
#define ICM20948_REG_INT_PIN_CFG        0x0Fu
#define ICM20948_REG_INT_ENABLE         0x10u
#define ICM20948_REG_INT_ENABLE_1       0x11u
#define ICM20948_REG_INT_ENABLE_2       0x12u
#define ICM20948_REG_INT_ENABLE_3       0x13u
#define ICM20948_REG_I2C_MST_STATUS     0x17u
#define ICM20948_REG_INT_STATUS         0x19u
#define ICM20948_REG_INT_STATUS_1       0x1Au
#define ICM20948_REG_INT_STATUS_2       0x1Bu
#define ICM20948_REG_INT_STATUS_3       0x1Cu
#define ICM20948_REG_DELAY_TIMEH        0x28u
#define ICM20948_REG_DELAY_TIMEL        0x29u
#define ICM20948_REG_ACCEL_XOUT_H       0x2Du
#define ICM20948_REG_ACCEL_XOUT_L       0x2Eu
#define ICM20948_REG_ACCEL_YOUT_H       0x2Fu
#define ICM20948_REG_ACCEL_YOUT_L       0x30u
#define ICM20948_REG_ACCEL_ZOUT_H       0x31u
#define ICM20948_REG_ACCEL_ZOUT_L       0x32u
#define ICM20948_REG_GYRO_XOUT_H        0x33u
#define ICM20948_REG_GYRO_XOUT_L        0x34u
#define ICM20948_REG_GYRO_YOUT_H        0x35u
#define ICM20948_REG_GYRO_YOUT_L        0x36u
#define ICM20948_REG_GYRO_ZOUT_H        0x37u
#define ICM20948_REG_GYRO_ZOUT_L        0x38u
#define ICM20948_REG_TEMP_OUT_H         0x39u
#define ICM20948_REG_TEMP_OUT_L         0x3Au
#define ICM20948_REG_EXT_SENS_DATA_00   0x3Bu
#define ICM20948_REG_EXT_SENS_DATA_01   0x3Cu
#define ICM20948_REG_EXT_SENS_DATA_08   0x43u
#define ICM20948_REG_FIFO_EN_1          0x66u
#define ICM20948_REG_FIFO_EN_2          0x67u
#define ICM20948_REG_FIFO_RST           0x68u
#define ICM20948_REG_FIFO_MODE          0x69u
#define ICM20948_REG_FIFO_COUNTH        0x70u
#define ICM20948_REG_FIFO_COUNTL        0x71u
#define ICM20948_REG_FIFO_R_W           0x72u
#define ICM20948_REG_DATA_RDY_STATUS    0x74u

/* USER_CTRL 位定义 */
#define ICM20948_BIT_DMP_EN         0x80u
#define ICM20948_BIT_FIFO_EN        0x40u
#define ICM20948_BIT_I2C_MST_EN     0x20u
#define ICM20948_BIT_I2C_IF_DIS     0x10u
#define ICM20948_BIT_DMP_RST        0x08u
#define ICM20948_BIT_SRAM_RST       0x04u
#define ICM20948_BIT_I2C_MST_RST    0x02u

/* PWR_MGMT_1 位定义 */
#define ICM20948_BIT_DEVICE_RESET   0x80u
#define ICM20948_BIT_SLEEP          0x40u
#define ICM20948_BIT_LP             0x20u
#define ICM20948_BIT_TEMP_DIS       0x08u
#define ICM20948_BIT_CLKSEL_AUTO    0x01u

/* INT_STATUS_1 位定义 */
#define ICM20948_BIT_RAW_DATA_RDY   0x01u

/* INT_PIN_CFG 位定义（bank0 0x0F） */
#define ICM20948_BIT_INT1_ACTL          0x80u   /* 1 = 低电平有效 */
#define ICM20948_BIT_INT1_OPEN          0x40u   /* 1 = 开漏输出 */
#define ICM20948_BIT_INT1_LATCH_EN      0x20u   /* 1 = 锁存直到读状态 */
#define ICM20948_BIT_INT_ANYRD_2CLEAR   0x10u   /* 1 = 任意寄存器读清中断 */
#define ICM20948_BIT_BYPASS_EN          0x02u

/* INT_ENABLE_1 位定义（bank0 0x11） */
#define ICM20948_BIT_RAW_DATA_RDY_EN    0x01u

/* ==================== BANK 1 ==================== */
#define ICM20948_REG_XA_OFFS_H      0x00u
#define ICM20948_REG_XA_OFFS_L      0x01u
#define ICM20948_REG_YA_OFFS_H      0x03u
#define ICM20948_REG_YA_OFFS_L      0x04u
#define ICM20948_REG_ZA_OFFS_H      0x06u
#define ICM20948_REG_ZA_OFFS_L      0x07u

/* ==================== BANK 2 ==================== */
#define ICM20948_REG_GYRO_SMPLRT_DIV    0x00u
#define ICM20948_REG_GYRO_CONFIG_1      0x01u
#define ICM20948_REG_GYRO_CONFIG_2      0x02u
#define ICM20948_REG_XG_OFFS_USRH       0x03u
#define ICM20948_REG_XG_OFFS_USRL       0x04u
#define ICM20948_REG_YG_OFFS_USRH       0x05u
#define ICM20948_REG_YG_OFFS_USRL       0x06u
#define ICM20948_REG_ZG_OFFS_USRH       0x07u
#define ICM20948_REG_ZG_OFFS_USRL       0x08u
#define ICM20948_REG_ODR_ALIGN_EN       0x09u
#define ICM20948_REG_ACCEL_SMPLRT_DIV_1 0x10u
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2 0x11u
#define ICM20948_REG_ACCEL_INTEL_CTRL   0x12u
#define ICM20948_REG_ACCEL_WOM_THR      0x13u
#define ICM20948_REG_ACCEL_CONFIG       0x14u
#define ICM20948_REG_ACCEL_CONFIG_2     0x15u
#define ICM20948_REG_FSYNC_CONFIG       0x52u
#define ICM20948_REG_TEMP_CONFIG        0x53u

/* GYRO_CONFIG_1 / ACCEL_CONFIG 位定义 */
#define ICM20948_DLPFCFG_SHIFT      3u      /* bit[5:3] */
#define ICM20948_FS_SHIFT           1u      /* bit[2:1]  */
#define ICM20948_BIT_DLPF_ENABLE    0x01u   /* bit[0]    */

/* ==================== BANK 3 ==================== */
#define ICM20948_REG_I2C_MST_ODR_CONFIG 0x00u
#define ICM20948_REG_I2C_MST_CTRL       0x01u
#define ICM20948_REG_I2C_MST_DELAY_CTRL 0x02u
#define ICM20948_REG_I2C_SLV0_ADDR      0x03u
#define ICM20948_REG_I2C_SLV0_REG       0x04u
#define ICM20948_REG_I2C_SLV0_CTRL      0x05u
#define ICM20948_REG_I2C_SLV0_DO        0x06u
#define ICM20948_REG_I2C_SLV1_ADDR      0x07u
#define ICM20948_REG_I2C_SLV1_REG       0x08u
#define ICM20948_REG_I2C_SLV1_CTRL      0x09u
#define ICM20948_REG_I2C_SLV1_DO        0x0Au
#define ICM20948_REG_I2C_SLV2_ADDR      0x0Bu
#define ICM20948_REG_I2C_SLV2_REG       0x0Cu
#define ICM20948_REG_I2C_SLV2_CTRL      0x0Du
#define ICM20948_REG_I2C_SLV2_DO        0x0Eu
#define ICM20948_REG_I2C_SLV3_ADDR      0x0Fu
#define ICM20948_REG_I2C_SLV3_REG       0x10u
#define ICM20948_REG_I2C_SLV3_CTRL      0x11u
#define ICM20948_REG_I2C_SLV3_DO        0x12u
#define ICM20948_REG_I2C_SLV4_CTRL      0x13u
#define ICM20948_REG_I2C_SLV4_ADDR      0x14u
#define ICM20948_REG_I2C_SLV4_REG       0x15u
#define ICM20948_REG_I2C_SLV4_DO        0x16u
#define ICM20948_REG_I2C_SLV4_DI        0x17u

/* I2C_SLVx_CTRL 位定义 */
#define ICM20948_BIT_SLV_EN         0x80u   /* bit7 使能 */
#define ICM20948_BIT_SLV_BYTE_SW    0x40u   /* bit6 字节交换 */
#define ICM20948_BIT_SLV_REG_DIS    0x20u   /* bit5 不写寄存器地址 */
#define ICM20948_BIT_SLV_GRP        0x10u   /* bit4 分组 */
#define ICM20948_BIT_SLV_LEN_MASK   0x0Fu   /* bit[3:0] 传输长度 */

/* I2C_SLVx_ADDR 位定义 */
#define ICM20948_BIT_SLV_RNW        0x80u   /* bit7 1=读 0=写 */

/* ==================== AK09916 磁力计 ==================== */
#define AK09916_REG_WIA1        0x00u
#define AK09916_REG_WIA2        0x01u
#define AK09916_REG_ST1         0x10u
#define AK09916_REG_HXL         0x11u
#define AK09916_REG_HXH         0x12u
#define AK09916_REG_HYL         0x13u
#define AK09916_REG_HYH         0x14u
#define AK09916_REG_HZL         0x15u
#define AK09916_REG_HZH         0x16u
#define AK09916_REG_TMPS        0x17u
#define AK09916_REG_ST2         0x18u
#define AK09916_REG_CNTL2       0x31u
#define AK09916_REG_CNTL3       0x32u

#define AK09916_BIT_DRDY        0x01u   /* ST1 bit0 数据就绪 */
#define AK09916_BIT_HOFL        0x08u   /* ST2 bit3 磁场溢出 */

#define AK09916_MODE_POWER_DOWN 0x00u
#define AK09916_MODE_SINGLE     0x01u
#define AK09916_MODE_CONT_10HZ  0x02u
#define AK09916_MODE_CONT_20HZ  0x04u
#define AK09916_MODE_CONT_50HZ  0x05u
#define AK09916_MODE_CONT_100HZ 0x08u
#define AK09916_MODE_SELF_TEST  0x10u

#define AK09916_SOFT_RESET      0x01u

/* AK09916 灵敏度 0.15 uT/LSB */
#define AK09916_UT_PER_LSB      0.15f

#endif /* ICM20948_REGS_H */
