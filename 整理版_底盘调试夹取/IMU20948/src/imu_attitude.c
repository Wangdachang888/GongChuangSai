/**
 * @file    imu_attitude.c
 * @brief   姿态解算实现（互补 / 卡尔曼 / Mahony 四元数）
 *
 * 原工程 BALANCE/filter.c 的 Kalman_Filter() 与 Yijielvbo() 已按等价数学
 * 关系移植到本文件，并扩展为三轴、支持可变采样周期。
 */
#include "imu_attitude.h"

#include <math.h>
#include <string.h>

/*
 * 兼容性补丁：部分 C90 编译器（例如 Keil MDK 的 ARMCC5 默认模式）的 math.h
 * 不声明 float 版本数学函数，直接调用会被当成返回 int 的隐式声明，结果错误。
 * 这里显式声明一次；在 C99 及以上的编译器上不会生效。
 */
#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 199901L)
extern float sqrtf(float x);
extern float atan2f(float y, float x);
extern float asinf(float x);
extern float sinf(float x);
extern float cosf(float x);
extern float fmodf(float x, float y);
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------------- */
/* 工具函数                                                                  */
/* ------------------------------------------------------------------------- */

float imu_attitude_wrap180(float angle_deg)
{
    float a;

    if (!(angle_deg == angle_deg)) {   /* NaN 保护 */
        return 0.0f;
    }
    /* 先放大再取模，避免 -0.0 与环境差异 */
    a = fmodf(angle_deg + 180.0f, 360.0f);
    if (a < 0.0f) {
        a += 360.0f;
    }
    return a - 180.0f;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/**
 * @brief 由加速度计算 roll / pitch
 *
 * 约定 +Z 向上（水平静止时 accel = (0,0,+1g)）：
 *   roll  = atan2(ay, az)
 *   pitch = atan2(-ax, sqrt(ay2 + az2))
 */
static bool accel_to_roll_pitch(const float a[3], float *roll_deg, float *pitch_deg)
{
    float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

    if (n < 0.05f) {          /* 失重 / 数据异常，放弃本次观测 */
        return false;
    }
    *roll_deg  = atan2f(a[1], a[2]) * IMU_RAD_TO_DEG;
    *pitch_deg = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2])) * IMU_RAD_TO_DEG;
    return true;
}

/** 判断磁力计数据是否可用于航向：排除全零与磁场接近垂直的无效情况 */
static bool mag_is_valid(const float m[3])
{
    float h2 = m[0] * m[0] + m[1] * m[1];
    float n2 = h2 + m[2] * m[2];

    if (n2 < 1.0f) {             /* 未采到数据（全 0）或幅值异常 */
        return false;
    }
    if (h2 < 0.04f * n2) {       /* 水平分量 < 总场的 20%，航向不可信 */
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* 一维卡尔曼滤波                                                            */
/* ------------------------------------------------------------------------- */

void imu_kalman1d_init(imu_kalman1d_t *k, float q_angle, float q_bias, float r_measure)
{
    if (k == NULL) {
        return;
    }
    memset(k, 0, sizeof(*k));
    k->q_angle   = q_angle;
    k->q_bias    = q_bias;
    k->r_measure = r_measure;
    k->p[0][0]   = 1.0f;
    k->p[1][1]   = 1.0f;
}

void imu_kalman1d_reset(imu_kalman1d_t *k, float angle_deg, float bias_dps)
{
    if (k == NULL) {
        return;
    }
    k->angle   = angle_deg;
    k->bias    = bias_dps;
    k->rate    = 0.0f;
    k->p[0][0] = 1.0f;
    k->p[0][1] = 0.0f;
    k->p[1][0] = 0.0f;
    k->p[1][1] = 1.0f;
    k->started = true;
}

float imu_kalman1d_update(imu_kalman1d_t *k, float measured_deg,
                          float gyro_rate_dps, float dt_s)
{
    float p00, p01, p10, p11;
    float s, k0, k1, err;

    if (k == NULL) {
        return 0.0f;
    }
    if (dt_s <= 0.0f) {
        dt_s = 0.005f;
    }

    if (!k->started) {                 /* 首帧直接用观测值对齐，避免缓慢收敛 */
        imu_kalman1d_reset(k, measured_deg, 0.0f);
        return k->angle;
    }

    /* 观测角与估计角的差值卷绕到 (-180,180]，支持跨越 ±180 的情况 */
    measured_deg = k->angle + imu_attitude_wrap180(measured_deg - k->angle);

    /* ---- 先验估计 ---- */
    k->angle += (gyro_rate_dps - k->bias) * dt_s;

    p00 = k->p[0][0] + dt_s * (k->q_angle - k->p[0][1] - k->p[1][0]);
    p01 = k->p[0][1] + dt_s * (-k->p[1][1]);
    p10 = k->p[1][0] + dt_s * (-k->p[1][1]);
    p11 = k->p[1][1] + dt_s * k->q_bias;

    /* ---- 卡尔曼增益 ---- */
    s  = p00 + k->r_measure;
    if (s <= 1e-9f) {
        s = 1e-9f;
    }
    k0 = p00 / s;
    k1 = p10 / s;

    /* ---- 后验估计 ---- */
    err = measured_deg - k->angle;
    k->angle += k0 * err;
    k->bias  += k1 * err;

    k->p[0][0] = p00 - k0 * p00;
    k->p[0][1] = p01 - k0 * p01;
    k->p[1][0] = p10 - k1 * p00;
    k->p[1][1] = p11 - k1 * p01;

    k->rate = gyro_rate_dps - k->bias;
    return k->angle;
}

/* ------------------------------------------------------------------------- */
/* 四元数工具                                                                */
/* ------------------------------------------------------------------------- */

static void quat_normalize(float q[4])
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    if (n < 1e-6f) {
        q[0] = 1.0f;
        q[1] = q[2] = q[3] = 0.0f;
        return;
    }
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
}

/** 由 roll/pitch/yaw（弧度）构造 ZYX 顺序四元数 */
static void quat_from_euler(float q[4], float roll, float pitch, float yaw)
{
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

static void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                    1.0f - 2.0f * (q1 * q1 + q2 * q2)) * IMU_RAD_TO_DEG;
    *pitch = asinf(clampf(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * IMU_RAD_TO_DEG;
    *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                    1.0f - 2.0f * (q2 * q2 + q3 * q3)) * IMU_RAD_TO_DEG;
}

/* ------------------------------------------------------------------------- */
/* Mahony 互补滤波                                                           */
/* ------------------------------------------------------------------------- */

static void mahony_update(imu_attitude_t *att, const float acc[3],
                          const float gyr[3], const float mag[3], float dt)
{
    float q0 = att->q[0], q1 = att->q[1], q2 = att->q[2], q3 = att->q[3];
    float gx = gyr[0] * IMU_DEG_TO_RAD;
    float gy = gyr[1] * IMU_DEG_TO_RAD;
    float gz = gyr[2] * IMU_DEG_TO_RAD;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    float n;
    float dq0, dq1, dq2, dq3;

    /* ---- 加速度计：估计重力方向（世界 +Z 在机体系的投影）与观测的叉积 ---- */
    n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (n > 0.05f) {
        float ax = acc[0] / n, ay = acc[1] / n, az = acc[2] / n;
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        ex += ay * vz - az * vy;
        ey += az * vx - ax * vz;
        ez += ax * vy - ay * vx;
    }

    /* ---- 磁力计：参考磁场方向与观测的叉积 ---- */
    if (mag != NULL) {
        n = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
        if (n > 1.0f) {
            float mx = mag[0] / n, my = mag[1] / n, mz = mag[2] / n;
            float hx, hy, bx, bz, wx, wy, wz;

            hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) +
                         my * (q1 * q2 - q0 * q3) +
                         mz * (q1 * q3 + q0 * q2));
            hy = 2.0f * (mx * (q1 * q2 + q0 * q3) +
                         my * (0.5f - q1 * q1 - q3 * q3) +
                         mz * (q2 * q3 - q0 * q1));
            bx = sqrtf(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1 * q3 - q0 * q2) +
                         my * (q2 * q3 + q0 * q1) +
                         mz * (0.5f - q1 * q1 - q2 * q2));

            wx = 2.0f * (bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2));
            wy = 2.0f * (bx * (q1 * q2 - q0 * q3) + bz * (q2 * q3 + q0 * q1));
            wz = 2.0f * (bx * (q1 * q3 + q0 * q2) + bz * (0.5f - q1 * q1 - q2 * q2));

            ex += my * wz - mz * wy;
            ey += mz * wx - mx * wz;
            ez += mx * wy - my * wx;
        }
    }

    /* ---- PI 反馈：积分项补偿陀螺零偏 ---- */
    if (att->ki > 0.0f) {
        att->integral_fb[0] += att->ki * ex * dt;
        att->integral_fb[1] += att->ki * ey * dt;
        att->integral_fb[2] += att->ki * ez * dt;
        gx += att->integral_fb[0];
        gy += att->integral_fb[1];
        gz += att->integral_fb[2];
    }
    gx += att->kp * ex;
    gy += att->kp * ey;
    gz += att->kp * ez;

    /* ---- 一阶龙格库塔积分四元数 ---- */
    dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    att->q[0] = q0 + dq0 * dt;
    att->q[1] = q1 + dq1 * dt;
    att->q[2] = q2 + dq2 * dt;
    att->q[3] = q3 + dq3 * dt;
    quat_normalize(att->q);
}

/* ------------------------------------------------------------------------- */
/* 磁航向                                                                    */
/* ------------------------------------------------------------------------- */

float imu_attitude_mag_heading(float roll_deg, float pitch_deg, const float mag_uT[3])
{
    float r, p, sr, cr, sp, cp, vx, vy;

    if (mag_uT == NULL) {
        return 0.0f;
    }
    r = roll_deg  * IMU_DEG_TO_RAD;
    p = pitch_deg * IMU_DEG_TO_RAD;
    sr = sinf(r); cr = cosf(r);
    sp = sinf(p); cp = cosf(p);

    /* v = Ry(pitch) * Rx(roll) * m，去掉倾斜后的水平分量 */
    vx = cp * mag_uT[0] + sp * (sr * mag_uT[1] + cr * mag_uT[2]);
    vy = cr * mag_uT[1] - sr * mag_uT[2];

    /* 世界磁场水平分量沿 +X 时 yaw = 0 */
    return atan2f(-vy, vx) * IMU_RAD_TO_DEG;
}

/* ------------------------------------------------------------------------- */
/* 对外接口                                                                  */
/* ------------------------------------------------------------------------- */

void imu_attitude_init(imu_attitude_t *att, imu_algo_t algo, float sample_dt)
{
    if (att == NULL) {
        return;
    }
    memset(att, 0, sizeof(*att));
    att->algo     = algo;
    att->alpha    = 0.02f;             /* 原工程 K1 = 0.02 */
    att->mag_gain = 0.5f;
    att->kp       = 1.0f;
    att->ki       = 0.0f;
    att->q[0]     = 1.0f;
    att->sample_dt = (sample_dt > 0.0f) ? sample_dt : 0.005f;

    imu_kalman1d_init(&att->kal_roll,  0.001f, 0.003f, 0.5f);
    imu_kalman1d_init(&att->kal_pitch, 0.001f, 0.003f, 0.5f);
}

void imu_attitude_set_mahony_gains(imu_attitude_t *att, float kp, float ki)
{
    if (att == NULL) {
        return;
    }
    att->kp = (kp >= 0.0f) ? kp : 0.0f;
    att->ki = (ki >= 0.0f) ? ki : 0.0f;
}

void imu_attitude_set_mag_gain(imu_attitude_t *att, float gain)
{
    if (att == NULL) {
        return;
    }
    att->mag_gain = (gain >= 0.0f) ? gain : 0.0f;
}

void imu_attitude_set_alpha(imu_attitude_t *att, float alpha)
{
    if (att == NULL) {
        return;
    }
    att->alpha = clampf(alpha, 0.0001f, 1.0f);
}

int imu_attitude_update(imu_attitude_t *att,
                        const float acc_g[3],
                        const float gyro_dps[3],
                        const float mag_uT[3],
                        float dt_s)
{
    float roll_m = 0.0f, pitch_m = 0.0f;
    bool acc_ok;
    bool mag_ok;

    if (att == NULL || acc_g == NULL || gyro_dps == NULL) {
        return -1;
    }
    if (!(dt_s > 0.0f)) {
        dt_s = att->sample_dt;
    }
    if (dt_s > 0.2f) {                 /* 长时间停帧时限制积分步长 */
        dt_s = 0.2f;
    }

    acc_ok = accel_to_roll_pitch(acc_g, &roll_m, &pitch_m);

    /* 磁力计数据有效性：全 0 / 幅值过低 / 接近垂直都不参与融合 */
    mag_ok = (mag_uT != NULL) && mag_is_valid(mag_uT);
    if (mag_ok) {
        att->use_mag = true;
    }

    /* ---- 首帧对齐：直接采用加速度计的倾角，避免从 0 缓慢爬升 ---- */
    if (!att->initialized) {
        if (acc_ok) {
            att->roll  = roll_m;
            att->pitch = pitch_m;
        }
        if (att->algo == IMU_ALGO_MAHONY) {
            quat_from_euler(att->q, att->roll * IMU_DEG_TO_RAD,
                            att->pitch * IMU_DEG_TO_RAD, 0.0f);
        }
        att->initialized = true;
    }

    switch (att->algo) {
    case IMU_ALGO_COMPLEMENTARY: {
        if (acc_ok) {
            float pred = att->roll + gyro_dps[0] * dt_s;
            att->roll = imu_attitude_wrap180(
                pred + att->alpha * imu_attitude_wrap180(roll_m - pred));

            pred = att->pitch + gyro_dps[1] * dt_s;
            att->pitch = imu_attitude_wrap180(
                pred + att->alpha * imu_attitude_wrap180(pitch_m - pred));
        } else {
            att->roll  += gyro_dps[0] * dt_s;
            att->pitch += gyro_dps[1] * dt_s;
        }
        att->yaw += gyro_dps[2] * dt_s;
        break;
    }

    case IMU_ALGO_KALMAN: {
        float m_roll  = acc_ok ? roll_m  : att->kal_roll.angle;
        float m_pitch = acc_ok ? pitch_m : att->kal_pitch.angle;

        att->roll  = imu_kalman1d_update(&att->kal_roll,  m_roll,  gyro_dps[0], dt_s);
        att->pitch = imu_kalman1d_update(&att->kal_pitch, m_pitch, gyro_dps[1], dt_s);
        att->yaw  += gyro_dps[2] * dt_s;
        break;
    }

    case IMU_ALGO_MAHONY:
    default: {
        float q_roll, q_pitch, q_yaw;

        mahony_update(att, acc_g, gyro_dps, mag_ok ? mag_uT : NULL, dt_s);

        quat_to_euler(att->q, &q_roll, &q_pitch, &q_yaw);
        att->roll  = q_roll;
        att->pitch = q_pitch;

        /* 把 [-180,180] 的 yaw 解卷绕成连续值 */
        att->yaw += imu_attitude_wrap180(q_yaw - imu_attitude_wrap180(att->yaw));
        break;
    }
    }

    /* ---- 互补/卡尔曼使用磁力计做航向纠偏（Mahony 已在四元数内部融合） ---- */
    if (mag_ok && att->algo != IMU_ALGO_MAHONY && att->mag_gain > 0.0f) {
        float heading = imu_attitude_mag_heading(att->roll, att->pitch, mag_uT);
        float err = imu_attitude_wrap180(heading - imu_attitude_wrap180(att->yaw));

        att->yaw += err * clampf(att->mag_gain * dt_s, 0.0f, 1.0f);
    }

    /* 俯仰角物理上限制在 ±90°，避免长时间纯陀螺积分越界 */
    if (att->algo != IMU_ALGO_MAHONY) {
        att->pitch = clampf(att->pitch, -90.0f, 90.0f);
    }

    return 0;
}

void imu_attitude_get_euler(const imu_attitude_t *att,
                            float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (att == NULL) {
        return;
    }
    if (roll_deg != NULL) {
        *roll_deg = imu_attitude_wrap180(att->roll);
    }
    if (pitch_deg != NULL) {
        *pitch_deg = att->pitch;
    }
    if (yaw_deg != NULL) {
        *yaw_deg = imu_attitude_wrap180(att->yaw - att->yaw_offset);
    }
}

void imu_attitude_get_quaternion(const imu_attitude_t *att, float q[4])
{
    if (att == NULL || q == NULL) {
        return;
    }
    if (att->algo == IMU_ALGO_MAHONY) {
        q[0] = att->q[0];
        q[1] = att->q[1];
        q[2] = att->q[2];
        q[3] = att->q[3];
    } else {
        quat_from_euler(q, att->roll * IMU_DEG_TO_RAD,
                        att->pitch * IMU_DEG_TO_RAD,
                        imu_attitude_wrap180(att->yaw) * IMU_DEG_TO_RAD);
    }
}

void imu_attitude_reset_yaw(imu_attitude_t *att, float yaw_deg)
{
    if (att == NULL) {
        return;
    }
    att->yaw_offset = att->yaw - yaw_deg;
}
