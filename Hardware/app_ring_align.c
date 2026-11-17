#include "app_ring_align.h"
#include <math.h>
#include <string.h>

#define RING_ALIGN_MAX_CORRECTIONS 20U
#define RING_ALIGN_MIN_SPEED_MPS   0.03f
#define RING_ALIGN_MIN_OMEGA_RADPS 0.08f

static void restore_limits(AppRingAlign *align)
{
    if (align == 0 || align->chassis == 0 || !align->speed_limited) return;
    Mecanum_SetSpeedLimit(align->chassis, align->saved_max_speed,
                          align->saved_max_omega);
    align->speed_limited = 0U;
}

static void fail(AppRingAlign *align, const char *reason)
{
    if (align == 0) return;
    Vision_RingFail(align->vision, reason);
    restore_limits(align);
    align->active = 0U;
    align->failed = 1U;
}

static uint8_t compute_step(const AppRingAlign *align, float *body_x,
                            float *body_y)
{
    float determinant;
    float x;
    float y;
    float length;
    float scale;
    if (align == 0 || body_x == 0 || body_y == 0 ||
        !align->calibration_valid) return 0U;
    determinant = align->config.matrix_00 * align->config.matrix_11 -
                  align->config.matrix_01 * align->config.matrix_10;
    if (fabsf(determinant) <= 0.000001f) return 0U;
    x = -align->config.gain *
        (align->config.matrix_11 * (float)align->vision->pos.dx -
         align->config.matrix_01 * (float)align->vision->pos.dy) / determinant;
    y = -align->config.gain *
        (-align->config.matrix_10 * (float)align->vision->pos.dx +
         align->config.matrix_00 * (float)align->vision->pos.dy) / determinant;
    length = sqrtf(x * x + y * y);
    if (length > align->config.max_step_m) {
        scale = align->config.max_step_m / length;
        x *= scale;
        y *= scale;
    } else if (length < align->config.min_step_m && length > 0.0f) {
        scale = align->config.min_step_m / length;
        x *= scale;
        y *= scale;
    }
    *body_x = x;
    *body_y = y;
    return 1U;
}

static void update_limits(AppRingAlign *align)
{
    if (!align->speed_limited) {
        align->saved_max_speed = align->chassis->max_speed;
        align->saved_max_omega = align->chassis->max_omega;
        align->speed_limited = 1U;
    }
    Mecanum_SetSpeedLimit(align->chassis,
                          fminf(align->saved_max_speed,
                                align->config.max_speed_mps),
                          fminf(align->saved_max_omega,
                                align->config.max_omega_radps));
}

void AppRingAlign_Init(AppRingAlign *align, VisionContext *vision,
                       MecanumChassis_t *chassis,
                       const RingAlignConfig *config)
{
    float determinant;
    if (align == 0) return;
    memset(align, 0, sizeof(*align));
    align->vision = vision;
    align->chassis = chassis;
    if (config != 0) align->config = *config;
    determinant = align->config.matrix_00 * align->config.matrix_11 -
                  align->config.matrix_01 * align->config.matrix_10;
    align->calibration_valid = (uint8_t)(fabsf(determinant) > 0.000001f &&
        align->config.gain > 0.0f && align->config.max_step_m > 0.0f &&
        align->config.min_step_m >= 0.0f &&
        align->config.max_speed_mps > 0.0f &&
        align->config.max_omega_radps > 0.0f);
}

uint8_t AppRingAlign_Start(AppRingAlign *align, uint8_t target, uint32_t now_ms)
{
    if (align == 0 || align->vision == 0 || align->chassis == 0 ||
        align->active) return 0U;
    if (!Vision_StartRingAlign(align->vision, target, now_ms)) return 0U;
    align->active = 1U;
    align->succeeded = 0U;
    align->failed = 0U;
    align->correction_count = 0U;
    align->stable_started = 0U;
    return 1U;
}

void AppRingAlign_Poll(AppRingAlign *align, uint32_t now_ms)
{
    const VisionPosition *position;
    float body_x;
    float body_y;
    float x;
    float y;
    float theta;
    float world_x;
    float world_y;
    float vx;
    float vy;
    float omega;
    float cos_theta;
    float sin_theta;
    if (align == 0 || !align->active || align->vision == 0 ||
        align->chassis == 0) return;
    if (Vision_RingSucceeded(align->vision)) {
        restore_limits(align);
        align->active = 0U;
        align->succeeded = 1U;
        return;
    }
    if (Vision_RingFailed(align->vision)) {
        restore_limits(align);
        align->active = 0U;
        align->failed = 1U;
        return;
    }
    if ((uint32_t)(now_ms - Vision_RingStartedAt(align->vision)) >
        VISION_RING_TOTAL_TIMEOUT_MS) {
        fail(align, "RING_TOTAL_TIMEOUT");
        return;
    }
    if (Vision_GetRingState(align->vision) == VISION_RING_WAIT_ACK &&
        (uint32_t)(now_ms - Vision_RingStartedAt(align->vision)) >
        VISION_ACK_TIMEOUT_MS) {
        fail(align, "RING_ACK_TIMEOUT");
        (void)Vision_SendBody(align->vision, "CANCEL");
        return;
    }
    if (Vision_GetRingState(align->vision) == VISION_RING_WAIT_POS &&
        (uint32_t)(now_ms - Vision_RingStepStartedAt(align->vision)) >
        VISION_RING_STEP_TIMEOUT_MS) {
        fail(align, "RING_POS_TIMEOUT");
        (void)Vision_SendBody(align->vision, "CANCEL");
        return;
    }
    if (Vision_GetRingState(align->vision) == VISION_RING_CALCULATE) {
        position = Vision_GetPosition(align->vision, now_ms);
        if (position == 0) {
            fail(align, "RING_POS_STALE");
            return;
        }
        if (fabsf((float)position->dx) <= align->config.tolerance_x_px &&
            fabsf((float)position->dy) <= align->config.tolerance_y_px) {
            if (!Vision_RingMarkStable(align->vision, now_ms))
                align->active = 0U;
            return;
        }
        align->stable_started = 0U;
        if (++align->correction_count > RING_ALIGN_MAX_CORRECTIONS ||
            !compute_step(align, &body_x, &body_y)) {
            fail(align, align->calibration_valid ?
                 "RING_NOT_CONVERGING" : "RING_CALIBRATION_INVALID");
            return;
        }
        Mecanum_GetPosition(align->chassis, &x, &y, &theta);
        cos_theta = cosf(theta);
        sin_theta = sinf(theta);
        world_x = body_x * cos_theta - body_y * sin_theta;
        world_y = body_x * sin_theta + body_y * cos_theta;
        update_limits(align);
        align->chassis->body_frame = 0U;
        Mecanum_SetPositionMode(align->chassis, x + world_x, y + world_y, theta);
        Vision_RingMarkMoving(align->vision, now_ms);
        return;
    }
    if (Vision_GetRingState(align->vision) != VISION_RING_MOVING) return;
    Mecanum_GetVelocity(align->chassis, &vx, &vy, &omega);
    if (!Mecanum_IsTargetReached(align->chassis) ||
        sqrtf(vx * vx + vy * vy) > RING_ALIGN_MIN_SPEED_MPS ||
        fabsf(omega) > RING_ALIGN_MIN_OMEGA_RADPS) {
        align->stable_started = 0U;
        return;
    }
    if (!align->stable_started) {
        align->stable_started = 1U;
        align->stable_since_ms = now_ms;
        return;
    }
    if ((uint32_t)(now_ms - align->stable_since_ms) < VISION_RING_STABLE_MS)
        return;
    align->stable_started = 0U;
    (void)Vision_RingMarkStepDone(align->vision, now_ms);
}

void AppRingAlign_Cancel(AppRingAlign *align)
{
    if (align == 0 || !align->active) return;
    Vision_RingFail(align->vision, "CANCELLED");
    restore_limits(align);
    Mecanum_Stop(align->chassis);
    align->active = 0U;
    align->failed = 1U;
}

uint8_t AppRingAlign_IsActive(const AppRingAlign *align)
{
    return align != 0 && align->active;
}

uint8_t AppRingAlign_Succeeded(const AppRingAlign *align)
{
    return align != 0 && align->succeeded;
}

uint8_t AppRingAlign_Failed(const AppRingAlign *align)
{
    return align != 0 && align->failed;
}
