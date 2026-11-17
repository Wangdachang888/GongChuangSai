/**
 * @file    nav.h
 * @brief   麦轮底盘惯性/里程计组合导航（二维位姿估计）
 *
 * 思路（地面机器人最实用的组合方式）：
 *   - 航向 yaw：IMU 为主（陀螺 + 磁力计），轮式角速度为辅（抑制陀螺漂移）
 *   - 平移 vx/vy：轮式里程计为主（IMU 加速度二次积分会很快发散，不用）
 *   - 零速修正 ZUPT：判定静止时把速度清零、冻结航向，抑制漂移
 *   - 打滑检测：轮式角速度与 IMU 角速度差异过大时，判定轮子打滑，
 *     此时只信 IMU 航向，并衰减由轮速推算的平移量
 *
 * 坐标约定（与 mecanum_chassis.h 一致）：
 *   X 向前，Y 向左，yaw 逆时针为正，单位 m / rad
 *
 * 用法（在 10ms 控制周期里）：
 * @code
 *   Nav_Init(&nav, 0.0f);
 *   ...
 *   Nav_Update(&nav, dt, chassis->vx_actual, chassis->vy_actual, chassis->omega_actual);
 *   Nav_GetPose(&nav, &x, &y, &yaw);
 * @endcode
 */
#ifndef __NAV_H
#define __NAV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* ---------------- 输出：世界系位姿 ---------------- */
    float x;                 /* m，前进方向 */
    float y;                 /* m，左方向 */
    float yaw;               /* rad，逆时针为正，范围 (-π, π] */

    /* ---------------- 输出：速度 ---------------- */
    float vx_body;           /* m/s，车体系 */
    float vy_body;
    float vx_world;          /* m/s，世界系 */
    float vy_world;
    float omega_wheel;       /* rad/s，轮式角速度（输入） */
    float omega_imu;         /* rad/s，IMU 角速度（由 yaw 差分得到） */

    /* ---------------- 状态标志 ---------------- */
    uint8_t stationary;      /* 1 = 判定静止（ZUPT 生效） */
    uint8_t slipping;        /* 1 = 检测到打滑 */
    uint8_t started;         /* 内部：是否已完成首帧对齐 */
    uint32_t slip_count;     /* 累计打滑次数 */
    uint32_t zupt_count;     /* 累计静止周期数 */

    /* ---------------- 可调参数 ---------------- */
    float zupt_v;            /* 静止判定的线速度阈值，m/s，默认 0.015 */
    float zupt_w;            /* 静止判定的角速度阈值，rad/s，默认 0.05 */
    float slip_thr;          /* 打滑判定的角速度差阈值，rad/s，默认 0.35 */
    float slip_v_scale;      /* 打滑时轮速换算平移的衰减系数，默认 0.25 */
    float wheel_weight;      /* 轮式航向权重 0~1，默认 0.10；用磁力计时建议 0 */

    /* ---------------- 内部状态 ---------------- */
    float yaw_prev_imu;
    float slip_ratio;        /* 当前角速度差 / 阈值，>1 即打滑 */
} Nav_t;

/** IMU posture snapshot supplied by the application runtime. */
typedef struct {
    float yaw_rad;
    uint8_t valid;
} NavImuSnapshot_t;

/** 初始化并清零位姿；yaw0 为初始航向（rad） */
void Nav_Init(Nav_t *nav, float yaw0);

/** 重新设定位姿（到位、重启任务、手动对位时调用） */
void Nav_Reset(Nav_t *nav, float x, float y, float yaw);

/**
 * @brief 导航更新，建议 10ms 调用一次（与底盘位置环同频）
 * @param dt         距上次调用的时间（s）
 * @param vx_body    车体系前进速度（m/s，来自轮速正运动学）
 * @param vy_body    车体系左移速度（m/s）
 * @param omega_wheel 车体系角速度（rad/s，来自轮速正运动学）
 */
void Nav_Update(Nav_t *nav, float dt, float vx_body, float vy_body,
                float omega_wheel, const NavImuSnapshot_t *imu);

/** 读取位姿（指针可为 NULL） */
void Nav_GetPose(const Nav_t *nav, float *x, float *y, float *yaw);

/** 读取世界系速度（指针可为 NULL） */
void Nav_GetVelocity(const Nav_t *nav, float *vx_world, float *vy_world);

/** 读取健康状态：静止 / 打滑 / 打滑计数 */
void Nav_GetHealth(const Nav_t *nav, uint8_t *stationary, uint8_t *slipping,
                   uint32_t *slip_count);

/**
 * @brief 调整参数（传 0 或负数表示保持默认）
 * @param zupt_v      静止线速度阈值 m/s
 * @param zupt_w      静止角速度阈值 rad/s
 * @param slip_thr    打滑角速度差阈值 rad/s
 * @param wheel_weight 轮式航向权重 0~1
 */
void Nav_SetParams(Nav_t *nav, float zupt_v, float zupt_w,
                   float slip_thr, float wheel_weight);

#ifdef __cplusplus
}
#endif

#endif /* __NAV_H */
