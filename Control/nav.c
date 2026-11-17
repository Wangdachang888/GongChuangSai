/**
 * @file    nav.c
 * @brief   麦轮底盘惯性/里程计组合导航实现
 *
 * 数据流：
 *   ① app_imu（200Hz）→ yaw（绝对航向，陀螺 + 磁力计融合）
 *   ② 轮式正运动学（10ms）→ vx/vy/omega（车体系）
 *   ③ 本模块（10ms）：航向融合 + 打滑检测 + 零速修正 → 世界系位姿
 *
 * 这里刻意不做“加速度二次积分”：
 *   地面机器人上，加速度计积分出的速度 1~2 秒就会漂到 0.1 m/s 量级，
 *   位置几分钟就能漂出几十米。轮速 + IMU 航向才是可用的组合。
 */
#include "nav.h"

#include <math.h>
#include <string.h>

/*
 * 兼容性补丁：Keil AC5 默认按 C90 编译，math.h 里可能不声明 float 版本函数，
 * 直接调用会变成返回 int 的隐式声明，结果错误。这里显式声明一次。
 */
#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 199901L)
extern float sqrtf(float x);
extern float sinf(float x);
extern float cosf(float x);
extern float fmodf(float x, float y);
#endif

#define NAV_PI          3.14159265358979323846f
#define NAV_TWO_PI      6.28318530717958647692f

/** 归一化到 (-π, π] */
static float nav_wrap_pi(float a)
{
    a = fmodf(a + NAV_PI, NAV_TWO_PI);
    if (a < 0.0f) {
        a += NAV_TWO_PI;
    }
    return a - NAV_PI;
}

void Nav_Init(Nav_t *nav, float yaw0)
{
    if (nav == NULL) {
        return;
    }
    memset(nav, 0, sizeof(*nav));

    nav->yaw           = nav_wrap_pi(yaw0);
    nav->zupt_v        = 0.015f;   /* 1.5 cm/s，编码器噪声以下 */
    nav->zupt_w        = 0.05f;    /* 约 3 deg/s */
    nav->slip_thr      = 0.35f;    /* 约 20 deg/s 的角速度差 */
    nav->slip_v_scale  = 0.25f;
    nav->wheel_weight  = 0.10f;
    nav->started       = 0u;
}

void Nav_Reset(Nav_t *nav, float x, float y, float yaw)
{
    if (nav == NULL) {
        return;
    }
    nav->x          = x;
    nav->y          = y;
    nav->yaw        = nav_wrap_pi(yaw);
    nav->vx_body    = 0.0f;
    nav->vy_body    = 0.0f;
    nav->vx_world   = 0.0f;
    nav->vy_world   = 0.0f;
    nav->stationary = 0u;
    nav->slipping   = 0u;
    nav->started    = 0u;          /* 下一帧重新与 IMU 对齐 */
}

void Nav_Update(Nav_t *nav, float dt, float vx_body, float vy_body,
                float omega_wheel, const NavImuSnapshot_t *imu)
{
    float imu_yaw;
    float d_imu;
    float err;
    float c, s, scale;
    uint8_t imu_ok;

    if (nav == NULL) {
        return;
    }
    if (!(dt > 0.0f)) {
        return;
    }
    if (dt > 0.1f) {
        dt = 0.1f;                 /* 单帧超过 100ms 认为调度异常，限制积分步长 */
    }

    /* ---------------- ① 取 IMU 航向 ---------------- */
    imu_ok  = (imu != NULL && imu->valid) ? 1u : 0u;
    imu_yaw = imu_ok ? imu->yaw_rad : nav->yaw;

    if (!nav->started) {
        nav->yaw          = imu_yaw;   /* 首帧直接对齐，避免从 0 慢慢爬 */
        nav->yaw_prev_imu = imu_yaw;
        nav->started      = 1u;
        d_imu             = 0.0f;
    } else {
        d_imu             = nav_wrap_pi(imu_yaw - nav->yaw_prev_imu);
        nav->yaw_prev_imu = imu_yaw;
    }
    nav->omega_imu = d_imu / dt;

    /* ---------------- ② 打滑检测 ---------------- */
    /* 轮子说转得比车快 / 慢很多 -> 打滑；原地被推着转也算 */
    if (imu_ok) {
        err = nav->omega_imu - omega_wheel;
        nav->slip_ratio = fabsf(err) / nav->slip_thr;
        nav->slipping   = (nav->slip_ratio > 1.0f) ? 1u : 0u;
        if (nav->slipping) {
            nav->slip_count++;
        }
    } else {
        nav->slip_ratio = 0.0f;    /* IMU 掉线时无从判断，不误报打滑 */
        nav->slipping   = 0u;
    }

    /* ---------------- ③ 零速修正（ZUPT） ---------------- */
    /* 轮子不动 + 陀螺不动 = 真的静止：速度清零、航向冻结 */
    if (!nav->slipping &&
        fabsf(vx_body) < nav->zupt_v &&
        fabsf(vy_body) < nav->zupt_v &&
        fabsf(omega_wheel) < nav->zupt_w &&
        fabsf(nav->omega_imu) < nav->zupt_w) {

        nav->stationary = 1u;
        nav->zupt_count++;
        nav->vx_body    = 0.0f;
        nav->vy_body    = 0.0f;
        nav->vx_world   = 0.0f;
        nav->vy_world   = 0.0f;
        nav->omega_wheel = omega_wheel;
        return;                    /* 位姿和航向都不积分 */
    }
    nav->stationary = 0u;

    /* ---------------- ④ 航向融合 ---------------- */
    if (imu_ok) {
        /* IMU 为主；轮式角速度只在没打滑时以低权重参与，用来抑制陀螺零漂 */
        float w = nav->slipping ? 0.0f : nav->wheel_weight;

        nav->yaw = nav_wrap_pi(nav->yaw +
                               (1.0f - w) * d_imu +
                               w * (omega_wheel * dt));
    } else {
        /* IMU 掉线：退化为纯轮式航向 */
        nav->yaw = nav_wrap_pi(nav->yaw + omega_wheel * dt);
    }

    /* ---------------- ⑤ 车体系速度 → 世界系 → 积分 ---------------- */
    nav->vx_body     = vx_body;
    nav->vy_body     = vy_body;
    nav->omega_wheel = omega_wheel;

    /* 打滑时轮速不可信，按系数衰减，避免“轮子空转、位置飞走” */
    scale = nav->slipping ? nav->slip_v_scale : 1.0f;

    c = cosf(nav->yaw);
    s = sinf(nav->yaw);
    nav->vx_world = (vx_body * c - vy_body * s) * scale;
    nav->vy_world = (vx_body * s + vy_body * c) * scale;

    nav->x += nav->vx_world * dt;
    nav->y += nav->vy_world * dt;
}

void Nav_GetPose(const Nav_t *nav, float *x, float *y, float *yaw)
{
    if (nav == NULL) {
        return;
    }
    if (x != NULL) {
        *x = nav->x;
    }
    if (y != NULL) {
        *y = nav->y;
    }
    if (yaw != NULL) {
        *yaw = nav->yaw;
    }
}

void Nav_GetVelocity(const Nav_t *nav, float *vx_world, float *vy_world)
{
    if (nav == NULL) {
        return;
    }
    if (vx_world != NULL) {
        *vx_world = nav->vx_world;
    }
    if (vy_world != NULL) {
        *vy_world = nav->vy_world;
    }
}

void Nav_GetHealth(const Nav_t *nav, uint8_t *stationary, uint8_t *slipping,
                   uint32_t *slip_count)
{
    if (nav == NULL) {
        return;
    }
    if (stationary != NULL) {
        *stationary = nav->stationary;
    }
    if (slipping != NULL) {
        *slipping = nav->slipping;
    }
    if (slip_count != NULL) {
        *slip_count = nav->slip_count;
    }
}

void Nav_SetParams(Nav_t *nav, float zupt_v, float zupt_w,
                   float slip_thr, float wheel_weight)
{
    if (nav == NULL) {
        return;
    }
    if (zupt_v > 0.0f) {
        nav->zupt_v = zupt_v;
    }
    if (zupt_w > 0.0f) {
        nav->zupt_w = zupt_w;
    }
    if (slip_thr > 0.0f) {
        nav->slip_thr = slip_thr;
    }
    if (wheel_weight >= 0.0f && wheel_weight <= 1.0f) {
        nav->wheel_weight = wheel_weight;
    }
}
