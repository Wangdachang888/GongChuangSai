/**
 * @file    app_stepper_drive.c
 * @brief   Independent jerk-limited planner for the D36A BSP.
 *
 * The profile is maintained in pulse units. Each tick decides whether the
 * remaining distance requires braking, changes acceleration by a bounded
 * jerk, integrates speed, and feeds only that tick's pulses to the BSP.
 * This keeps the S curve independent from the selected microstep setting.
 */
#include "app_stepper_drive.h"
#include "stepper_config.h"
#include "main.h"
#include "bsp_stepper.h"

typedef struct
{
    uint8_t active;
    uint8_t spin;
    uint8_t waiting_hardware;
    uint8_t trap_brake;
    uint32_t direction_ready_ms;
    int8_t direction;
    float target_sps;
    float speed_sps;
    float accel_sps2;
    float peak_accel_sps2;
    float jerk_sps3;
    int32_t steps_to_queue;
    float step_fraction;
} DriveAxis_t;

static DriveAxis_t s_axis[2];
static uint8_t s_next_trap_brake[2];
static uint8_t s_inited;
static uint32_t s_last_tick;
static uint8_t s_demo_enable;
static uint8_t s_demo_state;
static uint32_t s_demo_ms;

static float clampf_local(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static uint32_t drive_ppr(uint8_t axis)
{
    return (axis == (uint8_t)STEPPER_AXIS_1) ?
           APP_STEPPER_LIFT_PULSES_PER_REV : DRIVE_PPR;
}

static GPIO_PinState direction_level(int8_t direction)
{
    return (direction < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static float rpm_to_sps(uint8_t axis, float rpm)
{
    return rpm * (float)drive_ppr(axis) / 60.0f;
}

static float sps_to_rpm(uint8_t axis, float sps)
{
    return sps * 60.0f / (float)drive_ppr(axis);
}

static float stopping_distance(const DriveAxis_t *axis)
{
    float speed = axis->speed_sps;
    float accel = axis->peak_accel_sps2;

    if (speed <= 0.0f || accel < 1.0f) return 0.0f;
    if (axis->trap_brake != 0U) {
        return (speed * speed) / (2.0f * accel);
    }

    /* First term is the constant-acceleration distance. The second term is
       the extra distance used while negative acceleration ramps in. */
    return (speed * speed) / (2.0f * accel) +
           (speed * accel) / (2.0f * axis->jerk_sps3);
}

static void move_acceleration(DriveAxis_t *axis,
                              float requested,
                              float dt)
{
    float delta = requested - axis->accel_sps2;
    float change = axis->jerk_sps3 * dt;

    if (delta > change) delta = change;
    if (delta < -change) delta = -change;
    axis->accel_sps2 += delta;
    axis->accel_sps2 = clampf_local(axis->accel_sps2,
                                    -axis->peak_accel_sps2,
                                    axis->peak_accel_sps2);
}

static void drive_finish_if_empty(uint8_t axis)
{
    DriveAxis_t *profile = &s_axis[axis];

    if (profile->waiting_hardware != 0U &&
        Stepper_IsBusy((StepperAxis_t)axis) == 0U) {
        profile->active = 0U;
        profile->waiting_hardware = 0U;
        profile->speed_sps = 0.0f;
        profile->accel_sps2 = 0.0f;
        profile->step_fraction = 0.0f;
    }
}

static void drive_start(uint8_t axis,
                        int8_t direction,
                        int32_t steps,
                        uint8_t spin,
                        float rpm,
                        float accel)
{
    DriveAxis_t *profile;
    float target_sps;
    float peak_accel;

    if (axis > 1U || s_inited == 0U) return;
    if (direction == 0) direction = 1;
    if (steps < 0) steps = -steps;
    if (steps == 0 && spin == 0U) {
        StepperDrive_Stop(axis);
        return;
    }

    rpm = clampf_local(rpm, DRIVE_RPM_MIN, DRIVE_RPM_MAX);
    if (accel < 1.0f) accel = 1.0f;
    target_sps = rpm_to_sps(axis, rpm);
    target_sps = clampf_local(target_sps,
                              rpm_to_sps(axis, DRIVE_RPM_MIN),
                              (float)STEPPER_MAX_FREQ_HZ);
    peak_accel = rpm_to_sps(axis, accel);
    if (peak_accel < 1.0f) peak_accel = 1.0f;

    Stepper_Stop((StepperAxis_t)axis);
    if (Stepper_SetDirection((StepperAxis_t)axis,
                             direction_level(direction)) != HAL_OK) return;

    profile = &s_axis[axis];
    profile->active = 1U;
    profile->spin = spin;
    profile->waiting_hardware = 0U;
    profile->trap_brake = s_next_trap_brake[axis];
    s_next_trap_brake[axis] = 0U;
    profile->direction = direction;
    profile->target_sps = target_sps;
    profile->speed_sps = 0.0f;
    profile->accel_sps2 = 0.0f;
    profile->peak_accel_sps2 = peak_accel;
    /* Acceleration reaches its peak over about 0.20 s. */
    profile->jerk_sps3 = peak_accel / 0.20f;
    if (profile->jerk_sps3 < 1.0f) profile->jerk_sps3 = 1.0f;
    profile->steps_to_queue = (spin != 0U) ? 0 : steps;
    profile->step_fraction = 0.0f;
    profile->direction_ready_ms = HAL_GetTick() + DRIVE_DIRECTION_SETTLE_MS;
}

void StepperDrive_Init(void)
{
    uint8_t axis;

    Stepper_Init();
    Stepper_SetEnable(STEPPER_AXIS_1, 1U);
    Stepper_SetEnable(STEPPER_AXIS_2, 0U);
    for (axis = 0U; axis < 2U; axis++) {
        s_axis[axis].active = 0U;
        s_axis[axis].spin = 0U;
        s_axis[axis].waiting_hardware = 0U;
        s_axis[axis].trap_brake = 0U;
        s_axis[axis].direction_ready_ms = 0U;
        s_axis[axis].direction = 1;
        s_axis[axis].target_sps = 0.0f;
        s_axis[axis].speed_sps = 0.0f;
        s_axis[axis].accel_sps2 = 0.0f;
        s_axis[axis].peak_accel_sps2 = 1.0f;
        s_axis[axis].jerk_sps3 = 1.0f;
        s_axis[axis].steps_to_queue = 0;
        s_axis[axis].step_fraction = 0.0f;
        s_next_trap_brake[axis] = 0U;
    }
    s_last_tick = HAL_GetTick();
    s_demo_enable = 0U;
    s_demo_state = 0U;
    s_demo_ms = 0U;
    s_inited = 1U;
}

void StepperDrive_Spin(uint8_t axis, int8_t direction, float rpm, float accel)
{
    drive_start(axis, direction, 0, 1U, rpm, accel);
}

void StepperDrive_MoveRevs(uint8_t axis, int8_t direction,
                           float revs, float rpm, float accel)
{
    int32_t steps;
    if (revs < 0.0f) {
        revs = -revs;
        direction = (int8_t)-direction;
    }
    steps = (int32_t)(revs * (float)drive_ppr(axis) + 0.5f);
    drive_start(axis, direction, steps, 0U, rpm, accel);
}

void StepperDrive_MoveSteps(uint8_t axis, int8_t direction,
                            int32_t steps, float rpm, float accel)
{
    if (steps < 0) {
        steps = -steps;
        direction = (int8_t)-direction;
    }
    drive_start(axis, direction, steps, 0U, rpm, accel);
}

void StepperDrive_SetAccel(uint8_t axis, float accel)
{
    if (axis > 1U || s_inited == 0U || accel < 1.0f) return;
    s_axis[axis].peak_accel_sps2 = rpm_to_sps(axis, accel);
    if (s_axis[axis].peak_accel_sps2 < 1.0f) {
        s_axis[axis].peak_accel_sps2 = 1.0f;
    }
    s_axis[axis].jerk_sps3 = s_axis[axis].peak_accel_sps2 / 0.20f;
}

void StepperDrive_SetBrakeMode(uint8_t axis, uint8_t trap_brake)
{
    if (axis > 1U || s_inited == 0U) return;
    if (s_axis[axis].active != 0U) {
        s_axis[axis].trap_brake = (trap_brake != 0U) ? 1U : 0U;
    } else {
        s_next_trap_brake[axis] = (trap_brake != 0U) ? 1U : 0U;
    }
}

void StepperDrive_Stop(uint8_t axis)
{
    if (axis > 1U) return;
    s_axis[axis].active = 0U;
    s_axis[axis].waiting_hardware = 0U;
    s_axis[axis].steps_to_queue = 0;
    s_axis[axis].speed_sps = 0.0f;
    s_axis[axis].accel_sps2 = 0.0f;
    s_axis[axis].step_fraction = 0.0f;
    s_axis[axis].direction_ready_ms = 0U;
    Stepper_Stop((StepperAxis_t)axis);
}

void StepperDrive_StopAll(void)
{
    StepperDrive_Stop(0U);
    StepperDrive_Stop(1U);
}

static void drive_tick(uint8_t axis, float dt)
{
    DriveAxis_t *profile = &s_axis[axis];
    float requested_accel;
    float stop_distance;
    float pulse_count;
    int32_t feed_steps;
    uint32_t frequency_hz;

    if (profile->active == 0U) return;
    drive_finish_if_empty(axis);
    if (profile->active == 0U || profile->waiting_hardware != 0U) return;
    if ((int32_t)(HAL_GetTick() - profile->direction_ready_ms) < 0) return;

    stop_distance = stopping_distance(profile);
    if (profile->spin == 0U && profile->steps_to_queue <= 0) {
        profile->waiting_hardware = 1U;
        return;
    }

    if (profile->spin != 0U) {
        requested_accel = (profile->speed_sps < profile->target_sps) ?
                          profile->peak_accel_sps2 : 0.0f;
    } else if ((float)profile->steps_to_queue <= stop_distance + 2.0f) {
        requested_accel = -profile->peak_accel_sps2;
    } else if (profile->speed_sps < profile->target_sps) {
        requested_accel = profile->peak_accel_sps2;
    } else {
        requested_accel = 0.0f;
    }

    if (profile->trap_brake != 0U && requested_accel < 0.0f) {
        /* Final return moves use direct trapezoidal braking by design. */
        profile->accel_sps2 = -profile->peak_accel_sps2;
    } else {
        move_acceleration(profile, requested_accel, dt);
    }

    profile->speed_sps += profile->accel_sps2 * dt;
    if (profile->speed_sps < 0.0f) profile->speed_sps = 0.0f;
    if (profile->speed_sps > profile->target_sps) {
        profile->speed_sps = profile->target_sps;
        if (profile->accel_sps2 > 0.0f) profile->accel_sps2 = 0.0f;
    }

    /* Do not let the last few pulses turn into seconds of isolated 1 Hz
       pulses. Keep a short, deterministic terminal segment and then stop
       the hardware queue. */
    if (requested_accel < 0.0f) {
        float terminal_sps = (axis == (uint8_t)STEPPER_AXIS_2) ?
                             (float)DRIVE_TURNTABLE_TERMINAL_FREQ_HZ :
                             (float)DRIVE_TERMINAL_FREQ_HZ;
        if (terminal_sps > profile->target_sps) terminal_sps = profile->target_sps;
        if (profile->speed_sps < terminal_sps) {
            profile->speed_sps = terminal_sps;
            profile->accel_sps2 = 0.0f;
        }
    }

    pulse_count = profile->step_fraction + profile->speed_sps * dt;
    feed_steps = (int32_t)pulse_count;
    profile->step_fraction = pulse_count - (float)feed_steps;
    if (profile->spin == 0U && feed_steps > profile->steps_to_queue) {
        feed_steps = profile->steps_to_queue;
        profile->step_fraction = 0.0f;
    }
    if (feed_steps <= 0) return;

    if (profile->spin == 0U) {
        profile->steps_to_queue -= feed_steps;
        if (profile->steps_to_queue < 0) profile->steps_to_queue = 0;
    }
    frequency_hz = (uint32_t)(profile->speed_sps + 0.5f);
    if (frequency_hz < STEPPER_MIN_FREQ_HZ) frequency_hz = STEPPER_MIN_FREQ_HZ;
    if (frequency_hz > STEPPER_MAX_FREQ_HZ) frequency_hz = STEPPER_MAX_FREQ_HZ;
    if (Stepper_Feed((StepperAxis_t)axis, (uint32_t)feed_steps,
                     frequency_hz) != HAL_OK) {
        StepperDrive_Stop(axis);
        return;
    }
    if (profile->spin == 0U && profile->steps_to_queue == 0) {
        profile->waiting_hardware = 1U;
    }
}

static void demo_tick(void)
{
    if (s_demo_enable == 0U) return;
#if (DRIVE_DEMO_SPIN != 0)
    if (s_demo_state == 0U) {
        StepperDrive_Spin(0U, 1, DRIVE_DEMO_RPM, DRIVE_DEMO_ACCEL);
        s_demo_state = 1U;
    }
#else
    if (s_demo_state == 0U) {
        StepperDrive_MoveRevs(0U, 1, DRIVE_DEMO_REVS,
                              DRIVE_DEMO_RPM, DRIVE_DEMO_ACCEL);
        s_demo_state = 1U;
    } else if (s_demo_state == 1U && !StepperDrive_IsBusy(0U)) {
        s_demo_ms = HAL_GetTick();
        s_demo_state = 2U;
    } else if (s_demo_state == 2U &&
               (uint32_t)(HAL_GetTick() - s_demo_ms) >= DRIVE_DEMO_PAUSE_MS) {
        StepperDrive_MoveRevs(0U, -1, DRIVE_DEMO_REVS,
                              DRIVE_DEMO_RPM, DRIVE_DEMO_ACCEL);
        s_demo_state = 3U;
    } else if (s_demo_state == 3U && !StepperDrive_IsBusy(0U)) {
        s_demo_enable = 0U;
    }
#endif
}

void StepperDrive_DemoStart(void)
{
    s_demo_enable = 1U;
    s_demo_state = 0U;
    s_demo_ms = HAL_GetTick();
}

void StepperDrive_Poll(void)
{
    uint32_t now;
    uint32_t elapsed_ms;

    if (s_inited == 0U) return;
    now = HAL_GetTick();
    elapsed_ms = (uint32_t)(now - s_last_tick);
    if (elapsed_ms < DRIVE_TICK_MS) return;
    if (elapsed_ms > 50U) elapsed_ms = 50U;
    s_last_tick = now;
    demo_tick();
    drive_tick(0U, (float)elapsed_ms * 0.001f);
    drive_tick(1U, (float)elapsed_ms * 0.001f);
}

uint8_t StepperDrive_IsBusy(uint8_t axis)
{
    if (axis > 1U) return 0U;
    return (s_axis[axis].active != 0U ||
            Stepper_IsBusy((StepperAxis_t)axis) != 0U) ? 1U : 0U;
}

float StepperDrive_GetRPM(uint8_t axis)
{
    if (axis > 1U) return 0.0f;
    return sps_to_rpm(axis, s_axis[axis].speed_sps);
}

uint32_t StepperDrive_GetStepsLeft(uint8_t axis)
{
    uint32_t pending;
    if (axis > 1U) return 0U;
    pending = (s_axis[axis].steps_to_queue > 0) ?
              (uint32_t)s_axis[axis].steps_to_queue : 0U;
    return pending + Stepper_GetRemainingSteps((StepperAxis_t)axis);
}
