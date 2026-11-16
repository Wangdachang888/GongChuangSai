/**
 * @file    bsp_servo.c
 * @brief   PB14/PB15 dual servo + trapezoidal speed profile
 *
 * TIM12: 84 MHz / 84 = 1 MHz, ARR=19999 => 50 Hz
 * SetAngle/Nudge only change target; Servo_Poll ramps cmd with accel/brake.
 */
#include "bsp_servo.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim12;

typedef struct {
    uint32_t channel;
    float cmd_deg;
    float target_deg;
    float vel_dps;
    float max_speed_dps;
    float accel_dps2;
    uint16_t pulse_us;
    uint8_t enabled;
    uint8_t busy;
} ServoSlot_t;

static ServoSlot_t s_servo[SERVO_COUNT] = {
    { TIM_CHANNEL_1, SERVO_BOOT_ANGLE_DEG, SERVO_BOOT_ANGLE_DEG, 0.0f, SERVO_MAX_SPEED_DPS, SERVO_ACCEL_DPS2, SERVO_PULSE_CENTER_US, 0u, 0u },
    { TIM_CHANNEL_2, SERVO_BOOT_ANGLE_DEG, SERVO_BOOT_ANGLE_DEG, 0.0f, SERVO_MAX_SPEED_DPS, SERVO_ACCEL_DPS2, SERVO_PULSE_CENTER_US, 0u, 0u },
};

static uint8_t s_inited = 0u;
static uint32_t s_last_ms = 0u;
static uint16_t clampu16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float fabsf_local(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float fminf_local(float a, float b)
{
    return (a < b) ? a : b;
}

static float center_angle_deg(void)
{
    float span_us = (float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    float span_ang = SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG;
    if (span_us <= 0.0f) {
        return SERVO_BOOT_ANGLE_DEG;
    }
    return SERVO_ANGLE_MIN_DEG +
           ((float)(SERVO_PULSE_CENTER_US - SERVO_PULSE_MIN_US) / span_us) * span_ang;
}

static uint16_t angle_to_pulse_us(float angle_deg)
{
    float a = clampf(angle_deg, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    float span_ang = SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG;
    float span_us = (float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    float us;
    if (span_ang <= 0.0f) {
        return SERVO_PULSE_CENTER_US;
    }
    us = (float)SERVO_PULSE_MIN_US + ((a - SERVO_ANGLE_MIN_DEG) / span_ang) * span_us;
    if (us < 0.0f) us = 0.0f;
    return clampu16((uint16_t)(us + 0.5f), SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
}

static void apply_cmd(uint8_t id)
{
    uint16_t pulse;
    if (id >= SERVO_COUNT || s_inited == 0u) {
        return;
    }
    s_servo[id].cmd_deg = clampf(s_servo[id].cmd_deg,
                                  SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    pulse = angle_to_pulse_us(s_servo[id].cmd_deg);
    s_servo[id].pulse_us = pulse;
    __HAL_TIM_SET_COMPARE(&htim12, s_servo[id].channel, pulse);
}

static void ensure_enabled(uint8_t id)
{
    if (s_servo[id].enabled == 0u) {
        (void)HAL_TIM_PWM_Start(&htim12, s_servo[id].channel);
        s_servo[id].enabled = 1u;
    }
}

static void set_target(uint8_t id, float angle_deg)
{
    if (id >= SERVO_COUNT) return;
    ensure_enabled(id);
    s_servo[id].target_deg = clampf(angle_deg,
                                    SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    s_servo[id].busy = 1u;
}

static void profile_step(uint8_t id, float dt)
{
    ServoSlot_t *s;
    float err;
    float dir;
    float v;
    float v_max;
    float a;
    float brake_dist;
    float v_abs;
    float step;

    if (id >= SERVO_COUNT) return;
    s = &s_servo[id];
    if (s->enabled == 0u) return;

    err = s->target_deg - s->cmd_deg;
    if (fabsf_local(err) <= SERVO_STOP_EPS_DEG &&
        fabsf_local(s->vel_dps) <= SERVO_VEL_EPS_DPS) {
        s->cmd_deg = s->target_deg;
        s->vel_dps = 0.0f;
        s->busy = 0u;
        apply_cmd(id);
        return;
    }

    dir = (err >= 0.0f) ? 1.0f : -1.0f;
    v = s->vel_dps;
    v_max = s->max_speed_dps;
    a = s->accel_dps2;
    if (a < 1.0f) a = 1.0f;
    if (v_max < 1.0f) v_max = 1.0f;

    v_abs = fabsf_local(v);
    brake_dist = (v_abs * v_abs) / (2.0f * a);

    if ((v * dir) < 0.0f) {
        float dv = a * dt;
        if (v > 0.0f) {
            v -= dv;
            if (v < 0.0f) v = 0.0f;
        } else {
            v += dv;
            if (v > 0.0f) v = 0.0f;
        }
    } else if (fabsf_local(err) <= brake_dist) {
        float dv = a * dt;
        if (v > 0.0f) {
            v -= dv;
            if (v < 0.0f) v = 0.0f;
        } else if (v < 0.0f) {
            v += dv;
            if (v > 0.0f) v = 0.0f;
        } else {
            v = dir * fminf_local(v_max, fabsf_local(err) / dt);
        }
    } else {
        float dv = a * dt;
        v += dir * dv;
        if (v > v_max) v = v_max;
        if (v < -v_max) v = -v_max;
    }

    step = v * dt;
    if (dir > 0.0f && step > err) {
        step = err;
        v = 0.0f;
    } else if (dir < 0.0f && step < err) {
        step = err;
        v = 0.0f;
    }

    s->vel_dps = v;
    s->cmd_deg += step;
    s->cmd_deg = clampf(s->cmd_deg, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    s->busy = 1u;
    apply_cmd(id);
}

void Servo_Init(void)
{
    (void)HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);

    __HAL_TIM_DISABLE(&htim12);
    __HAL_TIM_SET_PRESCALER(&htim12, 84u - 1u);
    __HAL_TIM_SET_AUTORELOAD(&htim12, 20000u - 1u);
    htim12.Init.Prescaler = 84u - 1u;
    htim12.Init.Period = 20000u - 1u;
    htim12.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE(&htim12);

    s_inited = 1u;
    s_last_ms = HAL_GetTick();

    s_servo[SERVO_1].cmd_deg = SERVO_BOOT_ANGLE_DEG;
    s_servo[SERVO_1].target_deg = SERVO_BOOT_ANGLE_DEG;
    s_servo[SERVO_1].vel_dps = 0.0f;
    s_servo[SERVO_1].max_speed_dps = SERVO_MAX_SPEED_DPS;
    s_servo[SERVO_1].accel_dps2 = SERVO_ACCEL_DPS2;
    s_servo[SERVO_1].busy = 0u;
    s_servo[SERVO_2].cmd_deg = SERVO_BOOT_ANGLE_DEG;
    s_servo[SERVO_2].target_deg = SERVO_BOOT_ANGLE_DEG;
    s_servo[SERVO_2].vel_dps = 0.0f;
    s_servo[SERVO_2].max_speed_dps = SERVO_MAX_SPEED_DPS;
    s_servo[SERVO_2].accel_dps2 = SERVO_ACCEL_DPS2;
    s_servo[SERVO_2].busy = 0u;

    apply_cmd(SERVO_1);
    apply_cmd(SERVO_2);
    (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    s_servo[SERVO_1].enabled = 1u;
    s_servo[SERVO_2].enabled = 1u;
}

void Servo_Poll(void)
{
    uint32_t now;
    uint32_t dt_ms;
    float dt;
    uint8_t i;

    if (s_inited == 0u) return;

    now = HAL_GetTick();
    dt_ms = now - s_last_ms;
    if (dt_ms == 0u) {
        return;
    }
    s_last_ms = now;

    if (dt_ms > 50u) {
        dt_ms = 50u;
    }
    dt = (float)dt_ms * 0.001f;

    for (i = 0u; i < SERVO_COUNT; ++i) {
        profile_step(i, dt);
    }
}

void Servo_SetAngle(uint8_t id, float angle_deg)
{
    set_target(id, angle_deg);
}

void Servo_SetBothAngle(float angle1_deg, float angle2_deg)
{
    set_target(SERVO_1, angle1_deg);
    set_target(SERVO_2, angle2_deg);
}

void Servo_Nudge(uint8_t id, float delta_deg)
{
    if (id >= SERVO_COUNT) return;
    set_target(id, s_servo[id].target_deg + delta_deg);
}

void Servo_NudgeAll(float delta_deg)
{
    Servo_Nudge(SERVO_1, delta_deg);
    Servo_Nudge(SERVO_2, delta_deg);
}

void Servo_SetAngleImmediate(uint8_t id, float angle_deg)
{
    if (id >= SERVO_COUNT) return;
    ensure_enabled(id);
    angle_deg = clampf(angle_deg, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    s_servo[id].cmd_deg = angle_deg;
    s_servo[id].target_deg = angle_deg;
    s_servo[id].vel_dps = 0.0f;
    s_servo[id].busy = 0u;
    apply_cmd(id);
}

void Servo_SetPulseUs(uint8_t id, uint16_t pulse_us)
{
    float span_us, span_ang, ang;
    if (id >= SERVO_COUNT) return;
    pulse_us = clampu16(pulse_us, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    span_us = (float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    span_ang = SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG;
    if (span_us <= 0.0f) {
        ang = SERVO_BOOT_ANGLE_DEG;
    } else {
        ang = SERVO_ANGLE_MIN_DEG +
              ((float)(pulse_us - SERVO_PULSE_MIN_US) / span_us) * span_ang;
    }
    Servo_SetAngleImmediate(id, ang);
}

void Servo_Center(uint8_t id)
{
    set_target(id, center_angle_deg());
}

void Servo_CenterAll(void)
{
    float mid = center_angle_deg();
    set_target(SERVO_1, mid);
    set_target(SERVO_2, mid);
}

void Servo_Disable(uint8_t id)
{
    if (id >= SERVO_COUNT || s_inited == 0u) return;
    (void)HAL_TIM_PWM_Stop(&htim12, s_servo[id].channel);
    s_servo[id].enabled = 0u;
    s_servo[id].vel_dps = 0.0f;
    s_servo[id].busy = 0u;
}

void Servo_DisableAll(void)
{
    Servo_Disable(SERVO_1);
    Servo_Disable(SERVO_2);
}

void Servo_EnableAll(void)
{
    if (s_inited == 0u) {
        Servo_Init();
        return;
    }
    ensure_enabled(SERVO_1);
    ensure_enabled(SERVO_2);
    apply_cmd(SERVO_1);
    apply_cmd(SERVO_2);
}

float Servo_GetAngle(uint8_t id)
{
    if (id >= SERVO_COUNT) return 0.0f;
    return s_servo[id].cmd_deg;
}

float Servo_GetTarget(uint8_t id)
{
    if (id >= SERVO_COUNT) return 0.0f;
    return s_servo[id].target_deg;
}

uint16_t Servo_GetPulseUs(uint8_t id)
{
    if (id >= SERVO_COUNT) return 0u;
    return s_servo[id].pulse_us;
}

uint8_t Servo_IsBusy(uint8_t id)
{
    if (id >= SERVO_COUNT) return 0u;
    return s_servo[id].busy;
}

uint8_t Servo_IsBusyAny(void)
{
    return (Servo_IsBusy(SERVO_1) != 0u || Servo_IsBusy(SERVO_2) != 0u) ? 1u : 0u;
}

void Servo_SetSpeedLimit(float max_speed_dps)
{
    uint8_t i;
    if (max_speed_dps < 1.0f) max_speed_dps = 1.0f;
    if (max_speed_dps > 720.0f) max_speed_dps = 720.0f;
    for (i = 0u; i < SERVO_COUNT; ++i) s_servo[i].max_speed_dps = max_speed_dps;
}

void Servo_SetAccel(float accel_dps2)
{
    uint8_t i;
    if (accel_dps2 < 1.0f) accel_dps2 = 1.0f;
    if (accel_dps2 > 2000.0f) accel_dps2 = 2000.0f;
    for (i = 0u; i < SERVO_COUNT; ++i) s_servo[i].accel_dps2 = accel_dps2;
}

void Servo_SetProfile(uint8_t id, float max_speed_dps, float accel_dps2)
{
    if (id >= SERVO_COUNT) return;
    if (max_speed_dps < 1.0f) max_speed_dps = 1.0f;
    if (max_speed_dps > 720.0f) max_speed_dps = 720.0f;
    if (accel_dps2 < 1.0f) accel_dps2 = 1.0f;
    if (accel_dps2 > 2000.0f) accel_dps2 = 2000.0f;
    s_servo[id].max_speed_dps = max_speed_dps;
    s_servo[id].accel_dps2 = accel_dps2;
}
