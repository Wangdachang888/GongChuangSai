/**
 * @file    imu_attitude.h
 * @brief   姿态解算模块（与具体 IMU 芯片无关，可单独使用）
 *
 * 由原工程 BALANCE/filter.c 的 Kalman_Filter() / Yijielvbo() 提取并扩展：
 *   - IMU_ALGO_COMPLEMENTARY : 一阶互补滤波（对应原 Yijielvbo）
 *   - IMU_ALGO_KALMAN        : 二状态卡尔曼滤波（对应原 Kalman_Filter）
 *   - IMU_ALGO_MAHONY        : Mahony 四元数互补滤波（六轴/九轴，推荐）
 *
 * 坐标与角度约定（全模块统一，非常重要）：
 *   - 载体轴：+X 向前、+Y 向右、+Z 向上（右手系）
 *   - 静止水平放置时加速度计读数约为 (0, 0, +1g)
 *   - 若实际模块安装方向不同，用 icm20948_cfg_t.axis_map 重映射，
 *     保证送入本模块的 accel_g / gyro_dps 符合上述约定
 *   - roll  = 绕 +X 轴旋转角（右手定则），单位 度，范围 (-180, 180]
 *   - pitch = 绕 +Y 轴旋转角，单位 度，范围 [-90, 90]
 *   - yaw   = 绕 +Z 轴旋转角（右手定则，向上看逆时针为正），单位 度
 *   - yaw 的零点可以任意设定：调用 imu_attitude_reset_yaw()
 *
 * 单位：加速度 g、角速度 deg/s、磁力计 uT。
 */
#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_DEG_TO_RAD  0.017453292519943295f
#define IMU_RAD_TO_DEG  57.29577951308232f

/* ------------------------------------------------------------------------- */
/* 通用一维二状态卡尔曼滤波（原工程 Kalman_Filter 的等价实现）               */
/* ------------------------------------------------------------------------- */

/**
 * @brief 一维卡尔曼滤波器（状态：角度 + 陀螺零偏）
 *
 * 原工程 filter.c 中的 Kalman_Filter() 只能处理一路角度（平衡车的倾角），
 * 且 dt 被硬编码为 0.005s。本实现把状态封装成结构体，dt 由调用者传入，
 * 因此可以直接用于 roll / pitch 两轴，也可以用于任意角度融合。
 */
typedef struct {
    float angle;      /* 后验角度估计（度，连续不卷绕） */
    float bias;       /* 陀螺零偏估计（度/秒） */
    float rate;       /* 去偏后的角速度（度/秒） */
    float q_angle;    /* 角度过程噪声协方差，原工程 0.001 */
    float q_bias;     /* 零偏过程噪声协方差，原工程 0.003 */
    float r_measure;  /* 观测噪声协方差，原工程 0.5 */
    float p[2][2];    /* 误差协方差矩阵 */
    bool  started;
} imu_kalman1d_t;

void  imu_kalman1d_init(imu_kalman1d_t *k, float q_angle, float q_bias, float r_measure);
void  imu_kalman1d_reset(imu_kalman1d_t *k, float angle_deg, float bias_dps);
float imu_kalman1d_update(imu_kalman1d_t *k, float measured_deg,
                          float gyro_rate_dps, float dt_s);

/* ------------------------------------------------------------------------- */
/* 姿态解算器                                                                */
/* ------------------------------------------------------------------------- */

typedef enum {
    IMU_ALGO_COMPLEMENTARY = 0,  /* 一阶互补滤波：计算量最小，适合 8 位机 */
    IMU_ALGO_KALMAN        = 1,  /* 二状态卡尔曼：平滑，滞后略大 */
    IMU_ALGO_MAHONY        = 2   /* 四元数互补：全姿态，动态性能最好（推荐） */
} imu_algo_t;

typedef struct {
    imu_algo_t      algo;
    float           alpha;        /* 互补滤波系数，原工程 K1 = 0.02 */
    float           mag_gain;     /* 磁力计纠偏增益（1/s），0 = 不使用磁力计 */
    float           kp;           /* Mahony 比例增益 */
    float           ki;           /* Mahony 积分增益 */

    imu_kalman1d_t  kal_roll;
    imu_kalman1d_t  kal_pitch;

    float           roll;         /* 输出：横滚角（度） */
    float           pitch;        /* 输出：俯仰角（度） */
    float           yaw;          /* 内部：连续 yaw（度，不卷绕） */
    float           yaw_offset;   /* 输出零点偏移 */

    float           q[4];         /* Mahony 姿态四元数 w,x,y,z */
    float           integral_fb[3];

    bool            use_mag;      /* 是否已收到过有效磁力计数据 */
    bool            initialized;  /* 是否完成首帧对齐 */
    float           sample_dt;    /* 缺省采样周期（s） */
} imu_attitude_t;

/**
 * @brief 初始化姿态解算器
 * @param algo 解算算法
 * @param sample_dt 名义采样周期（秒），例如 200Hz 填 0.005f
 */
void imu_attitude_init(imu_attitude_t *att, imu_algo_t algo, float sample_dt);

/** 设置 Mahony 增益（kp 越大收敛越快但噪声越大，典型 0.5~5；ki 典型 0~0.05） */
void imu_attitude_set_mahony_gains(imu_attitude_t *att, float kp, float ki);

/** 设置磁力计纠偏增益（1/s）。0 表示不使用磁力计，2.0 表示约 0.5s 时间常数 */
void imu_attitude_set_mag_gain(imu_attitude_t *att, float gain);

/** 设置互补滤波系数 alpha（0.001~0.2，越小越信任陀螺） */
void imu_attitude_set_alpha(imu_attitude_t *att, float alpha);

/**
 * @brief 姿态更新（每读到一帧 IMU 数据调用一次）
 *
 * @param acc_g     加速度，单位 g，顺序 XYZ，不可为 NULL
 * @param gyro_dps  角速度，单位 deg/s，顺序 XYZ，不可为 NULL
 * @param mag_uT    磁力计，单位 uT，顺序 XYZ；为 NULL 时仅用六轴
 * @param dt_s      距上一次调用的时间，单位秒；<= 0 时使用 sample_dt
 * @return 0 成功，-1 入参非法
 */
int imu_attitude_update(imu_attitude_t *att,
                        const float acc_g[3],
                        const float gyro_dps[3],
                        const float mag_uT[3],
                        float dt_s);

/** 读取欧拉角（度），指针可为 NULL */
void imu_attitude_get_euler(const imu_attitude_t *att,
                            float *roll_deg, float *pitch_deg, float *yaw_deg);

/** 读取四元数 w,x,y,z（仅 Mahony 有意义；其余算法由欧拉角合成） */
void imu_attitude_get_quaternion(const imu_attitude_t *att, float q[4]);

/**
 * @brief 把当前航向设为指定角度（通常填 0）
 * @note  只是平移输出零点，不改变内部融合状态；有磁力计时零点也不会被拉回
 */
void imu_attitude_reset_yaw(imu_attitude_t *att, float yaw_deg);

/** 由 roll/pitch + 磁力计计算倾斜补偿后的磁航向（度，绕 +Z 逆时针为正） */
float imu_attitude_mag_heading(float roll_deg, float pitch_deg, const float mag_uT[3]);

/** 角度归一化到 (-180, 180] */
float imu_attitude_wrap180(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ATTITUDE_H */
