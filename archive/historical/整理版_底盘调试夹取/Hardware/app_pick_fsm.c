/**
 * @file    app_pick_fsm.c
 * @brief   Pick directions using PB14, PB15, lift axis 1 and turntable axis 2.
 *
 * PB14/SERVO_1 is the gripper. PB15/SERVO_2 rotates the complete holder.
 * Lift coordinates are open-loop coordinates relative to the manually
 * established zero; no false software homing is claimed.
 */
#include "app_pick_fsm.h"
#include "app_stepper_openloop.h"
#include "bsp_servo.h"
#include <math.h>

#define PICK_GRIPPER_SERVO SERVO_1
#define PICK_ROTATE_SERVO  SERVO_2
#define PICK_LIFT_AXIS     STEPPER_AXIS_1
#define PICK_TURNTABLE_AXIS STEPPER_AXIS_2

static float absf_local(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t valid_config(const PickConfig *config)
{
    if (config == 0) return 0U;
    return (config->lift_frequency_hz >= 1U &&
            config->lift_frequency_hz <= STEPPER_MAX_FREQ_HZ &&
            config->forward_approach_steps != 0 &&
            config->forward_to_grab_steps != 0 &&
            config->forward_raise_steps != 0 &&
            config->forward_reset_steps != 0 &&
            config->reverse_raise_steps != 0 &&
            config->reverse_to_grab_steps != 0 &&
            config->reverse_raise_after_close_steps != 0 &&
            config->reverse_to_ground_steps != 0 &&
            config->reverse_reset_steps != 0 &&
            config->lift_accel_rpms >= 1.0f &&
            config->rotate_speed_dps >= 1.0f &&
            config->rotate_accel_dps2 >= 1.0f &&
            config->gripper_speed_dps >= 1.0f &&
            config->gripper_accel_dps2 >= 1.0f &&
            config->servo_timeout_ms > 0U &&
            config->lift_timeout_ms > 0U &&
            config->settle_ms < config->servo_timeout_ms &&
            config->forward_rotate_trigger_steps > 0U &&
            config->reverse_rotate_trigger_steps > 0U &&
            config->reverse_home_rotate_trigger_steps > 0U &&
            config->forward_cycle_count > 0U &&
            config->turntable_ccw_sign != 0 &&
            config->turntable_first_ccw_steps != 0 &&
            config->turntable_cw_steps != 0 &&
            config->turntable_return_ccw_steps != 0 &&
            config->turntable_frequency_hz >= 1U &&
            config->turntable_frequency_hz <= STEPPER_MAX_FREQ_HZ &&
            config->turntable_accel_rpms >= 1.0f) ? 1U : 0U;
}

static uint8_t timed_out(const PickFsm *fsm, uint32_t now, uint32_t timeout)
{
    return ((uint32_t)(now - fsm->state_since_ms) >= timeout) ? 1U : 0U;
}

static void enter_state(PickFsm *fsm, PickState state, uint32_t now)
{
    fsm->state = state;
    fsm->state_since_ms = now;
    fsm->rotate_started_ms = 0U;
    fsm->lift_started = 0U;
    fsm->rotate_started = 0U;
    fsm->turntable_started = 0U;
}

static void fail(PickFsm *fsm, PickError error)
{
    AppStepper_Stop(PICK_LIFT_AXIS);
    AppStepper_Stop(PICK_TURNTABLE_AXIS);
    AppStepper_SetEnable(PICK_TURNTABLE_AXIS, 0U);
    fsm->error = error;
    fsm->state = PICK_FAULT;
}

static void set_gripper(const PickFsm *fsm, float angle)
{
    Servo_SetProfile(PICK_GRIPPER_SERVO,
                     fsm->config.gripper_speed_dps,
                     fsm->config.gripper_accel_dps2);
    Servo_SetAngle(PICK_GRIPPER_SERVO, angle);
}

static void set_rotate(const PickFsm *fsm, float angle)
{
    Servo_SetProfile(PICK_ROTATE_SERVO,
                     fsm->config.rotate_speed_dps,
                     fsm->config.rotate_accel_dps2);
    Servo_SetAngle(PICK_ROTATE_SERVO, angle);
}

static uint8_t start_lift(PickFsm *fsm, int32_t steps,
                          uint32_t now, uint8_t trap_brake)
{
    AppStepper_SetBrakeMode(PICK_LIFT_AXIS, trap_brake);
    if (AppStepper_MoveStepsProfile(PICK_LIFT_AXIS, steps,
                                    fsm->config.lift_frequency_hz,
                                    fsm->config.lift_accel_rpms) != HAL_OK) {
        fail(fsm, PICK_ERROR_COMMAND);
        return 0U;
    }
    fsm->state_since_ms = now;
    return 1U;
}

static uint8_t start_lift_once(PickFsm *fsm, int32_t steps,
                               uint32_t now, uint8_t trap_brake)
{
    if (fsm->lift_started != 0U) return 1U;
    if (!start_lift(fsm, steps, now, trap_brake)) return 0U;
    fsm->lift_started = 1U;
    return 1U;
}

static uint8_t start_turntable(PickFsm *fsm, int32_t steps, uint32_t now)
{
    if (steps == 0) return 0U;
    AppStepper_SetEnable(PICK_TURNTABLE_AXIS, 1U);
    AppStepper_SetBrakeMode(PICK_TURNTABLE_AXIS, 1U);
    if (AppStepper_MoveStepsProfile(PICK_TURNTABLE_AXIS,
                                    steps,
                                    fsm->config.turntable_frequency_hz,
                                    fsm->config.turntable_accel_rpms) != HAL_OK) {
        fail(fsm, PICK_ERROR_COMMAND);
        return 0U;
    }
    fsm->state_since_ms = now;
    return 1U;
}

static uint8_t direction_settled(const PickFsm *fsm, uint32_t now)
{
    return ((uint32_t)(now - fsm->state_since_ms) >=
            fsm->config.direction_settle_ms) ? 1U : 0U;
}

static void trigger_rotate(PickFsm *fsm, float angle, uint32_t remaining)
{
    if (fsm->rotate_started == 0U &&
        AppStepper_GetRemainingSteps(PICK_LIFT_AXIS) <= remaining) {
        set_rotate(fsm, angle);
        fsm->rotate_started = 1U;
    }
}

static uint32_t rotate_duration_ms(const PickFsm *fsm)
{
    float distance = absf_local(fsm->config.rotate_place_deg -
                                fsm->config.rotate_home_deg);
    float speed = fsm->config.rotate_speed_dps;
    float accel = fsm->config.rotate_accel_dps2;
    float accel_distance;
    float seconds;

    if (speed < 1.0f) speed = 1.0f;
    if (accel < 1.0f) accel = 1.0f;
    accel_distance = speed * speed / accel;
    if (distance <= accel_distance) {
        seconds = 2.0f * sqrtf(distance / accel);
    } else {
        seconds = 2.0f * speed / accel +
                  (distance - accel_distance) / speed;
    }
    return (uint32_t)(seconds * 1000.0f + 0.5f);
}

void PickFsm_Init(PickFsm *fsm, const PickConfig *config)
{
    if (fsm == 0) return;
    fsm->state = PICK_IDLE;
    fsm->error = PICK_ERROR_NONE;
    fsm->state_since_ms = 0U;
    fsm->rotate_started_ms = 0U;
    fsm->lift_started = 0U;
    fsm->rotate_started = 0U;
    fsm->turntable_started = 0U;
    fsm->forward_cycle_index = 0U;
    if (config != 0) fsm->config = *config;
}

void PickFsm_Reset(PickFsm *fsm, uint32_t now)
{
    if (fsm == 0) return;
    AppStepper_Stop(PICK_LIFT_AXIS);
    AppStepper_Stop(PICK_TURNTABLE_AXIS);
    AppStepper_SetEnable(PICK_TURNTABLE_AXIS, 0U);
    set_gripper(fsm, fsm->config.gripper_open_deg);
    set_rotate(fsm, fsm->config.rotate_home_deg);
    fsm->error = PICK_ERROR_NONE;
    enter_state(fsm, PICK_RESETTING, now);
}

uint8_t PickFsm_Start(PickFsm *fsm, uint32_t now)
{
    if (fsm == 0 || fsm->state != PICK_IDLE ||
        !valid_config(&fsm->config)) {
        if (fsm != 0) {
            fsm->error = PICK_ERROR_CONFIG;
            fsm->state = PICK_FAULT;
        }
        return 0U;
    }
    fsm->error = PICK_ERROR_NONE;
    fsm->forward_cycle_index = 0U;
    set_gripper(fsm, fsm->config.gripper_open_deg);
    set_rotate(fsm, fsm->config.rotate_home_deg);
    enter_state(fsm, PICK_FORWARD_HOME, now);
    return 1U;
}

uint8_t PickFsm_StartReverse(PickFsm *fsm, uint32_t now)
{
    if (fsm == 0 || fsm->state != PICK_IDLE ||
        !valid_config(&fsm->config)) {
        if (fsm != 0) {
            fsm->error = PICK_ERROR_CONFIG;
            fsm->state = PICK_FAULT;
        }
        return 0U;
    }
    fsm->error = PICK_ERROR_NONE;
    fsm->forward_cycle_index = 0U;
    AppStepper_Stop(PICK_TURNTABLE_AXIS);
    AppStepper_SetEnable(PICK_TURNTABLE_AXIS, 0U);
    set_gripper(fsm, fsm->config.gripper_open_deg);
    set_rotate(fsm, fsm->config.rotate_home_deg);
    enter_state(fsm, PICK_REVERSE_HOME, now);
    return 1U;
}

void PickFsm_Poll(PickFsm *fsm, uint32_t now)
{
    uint32_t rotation_ms;
    uint32_t lift_start_ms;

    if (fsm == 0 || fsm->state == PICK_IDLE ||
        fsm->state == PICK_DONE || fsm->state == PICK_FAULT ||
        fsm->state == PICK_ABORTED) return;

    Servo_Poll();
    AppStepper_Poll();

    switch (fsm->state) {
    case PICK_RESETTING:
        if (!Servo_IsBusy(PICK_GRIPPER_SERVO) &&
            !Servo_IsBusy(PICK_ROTATE_SERVO)) {
            enter_state(fsm, PICK_IDLE, now);
        } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms)) {
            fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
        }
        break;

    case PICK_FORWARD_HOME:
        if (!Servo_IsBusy(PICK_GRIPPER_SERVO) &&
            !Servo_IsBusy(PICK_ROTATE_SERVO)) {
            enter_state(fsm, PICK_FORWARD_TO_APPROACH, now);
        } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms)) {
            fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
        }
        break;

    case PICK_FORWARD_TO_APPROACH:
        if (!fsm->lift_started) {
            if (direction_settled(fsm, now)) {
                (void)start_lift_once(fsm, fsm->config.forward_approach_steps,
                                      now, 0U);
            }
        } else if (!AppStepper_IsBusy(PICK_LIFT_AXIS)) {
            enter_state(fsm, PICK_FORWARD_TO_GRAB, now);
        } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
            fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
        }
        break;

    case PICK_FORWARD_TO_GRAB:
        if (!fsm->lift_started) {
            if (direction_settled(fsm, now)) {
                (void)start_lift_once(fsm, fsm->config.forward_to_grab_steps,
                                      now, 0U);
            }
        } else if (!AppStepper_IsBusy(PICK_LIFT_AXIS)) {
            enter_state(fsm, PICK_FORWARD_CLOSE, now);
        } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
            fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
        }
        break;

    case PICK_FORWARD_CLOSE:
        if (!fsm->rotate_started) {
            set_gripper(fsm, fsm->config.gripper_close_deg);
            fsm->rotate_started = 1U;
            fsm->state_since_ms = now;
        } else if (!Servo_IsBusy(PICK_GRIPPER_SERVO) &&
                   timed_out(fsm, now, fsm->config.settle_ms)) {
            if (fsm->forward_cycle_index == 0U) {
                enter_state(fsm, PICK_FORWARD_TURNTABLE_FIRST, now);
            } else {
                enter_state(fsm, PICK_FORWARD_RAISE, now);
            }
        } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms)) {
            fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
        }
        break;

    case PICK_FORWARD_TURNTABLE_FIRST:
        if (!fsm->turntable_started) {
            if (start_turntable(fsm,
                                fsm->config.turntable_first_ccw_steps *
                                fsm->config.turntable_ccw_sign,
                                now)) {
                fsm->turntable_started = 1U;
            }
        } else if (!AppStepper_IsBusy(PICK_TURNTABLE_AXIS)) {
            enter_state(fsm, PICK_FORWARD_RAISE, now);
        } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
            fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
        }
        break;

    case PICK_FORWARD_RAISE:
        if (!fsm->lift_started) {
            if (direction_settled(fsm, now)) {
                (void)start_lift_once(fsm, fsm->config.forward_raise_steps,
                                      now, 0U);
            }
        } else {
            trigger_rotate(fsm, fsm->config.rotate_place_deg,
                           fsm->config.forward_rotate_trigger_steps);
            if (!AppStepper_IsBusy(PICK_LIFT_AXIS) &&
                fsm->rotate_started != 0U &&
                !Servo_IsBusy(PICK_ROTATE_SERVO)) {
                set_gripper(fsm, fsm->config.gripper_open_deg);
                enter_state(fsm, PICK_FORWARD_OPEN, now);
            } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
                fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
            } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms)) {
                fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
            }
        }
        break;

    case PICK_FORWARD_OPEN:
        if (!Servo_IsBusy(PICK_GRIPPER_SERVO)) {
            set_rotate(fsm, fsm->config.rotate_home_deg);
            fsm->rotate_started_ms = now;
            fsm->rotate_started = 1U;
            enter_state(fsm, PICK_FORWARD_RESET_ROTATE, now);
            fsm->rotate_started_ms = now;
        } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms)) {
            fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
        }
        break;

    case PICK_FORWARD_RESET_ROTATE:
        rotation_ms = rotate_duration_ms(fsm);
        lift_start_ms = (rotation_ms > fsm->config.lift_before_rotate_done_ms) ?
                        (rotation_ms - fsm->config.lift_before_rotate_done_ms) : 0U;
        if (fsm->lift_started == 0U &&
            (uint32_t)(now - fsm->rotate_started_ms) >= lift_start_ms) {
            (void)start_lift_once(fsm, fsm->config.forward_reset_steps,
                                  now, 1U);
        }
        if (fsm->lift_started != 0U &&
            !AppStepper_IsBusy(PICK_LIFT_AXIS) &&
            !Servo_IsBusy(PICK_ROTATE_SERVO)) {
            enter_state(fsm, PICK_FORWARD_WAIT_ZERO, now);
        } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
            fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
        } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms +
                             fsm->config.lift_before_rotate_done_ms)) {
            fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
        }
        break;

    case PICK_FORWARD_WAIT_ZERO:
        if (!AppStepper_IsBusy(PICK_LIFT_AXIS)) {
            if ((uint8_t)(fsm->forward_cycle_index + 1U) >=
                fsm->config.forward_cycle_count) {
                enter_state(fsm, PICK_FORWARD_TURNTABLE_RETURN, now);
            } else {
                enter_state(fsm, PICK_FORWARD_TURNTABLE_STEP, now);
            }
        }
        break;

    case PICK_FORWARD_TURNTABLE_STEP:
        if (!fsm->turntable_started) {
            if (start_turntable(fsm,
                                -fsm->config.turntable_cw_steps *
                                fsm->config.turntable_ccw_sign,
                                now)) {
                fsm->turntable_started = 1U;
            }
        } else if (!AppStepper_IsBusy(PICK_TURNTABLE_AXIS)) {
            fsm->forward_cycle_index++;
            enter_state(fsm, PICK_FORWARD_HOME, now);
        } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
            fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
        }
        break;

    case PICK_FORWARD_TURNTABLE_RETURN:
        if (!fsm->turntable_started) {
            if (start_turntable(fsm,
                                fsm->config.turntable_return_ccw_steps *
                                fsm->config.turntable_ccw_sign,
                                now)) {
                fsm->turntable_started = 1U;
            }
        } else if (!AppStepper_IsBusy(PICK_TURNTABLE_AXIS)) {
            AppStepper_SetEnable(PICK_TURNTABLE_AXIS, 0U);
            enter_state(fsm, PICK_DONE, now);
        } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
            fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
        }
        break;

    case PICK_REVERSE_HOME:
        if (!Servo_IsBusy(PICK_GRIPPER_SERVO) &&
            !Servo_IsBusy(PICK_ROTATE_SERVO)) {
            enter_state(fsm, PICK_REVERSE_RAISE, now);
        } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms)) {
            fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
        }
        break;

    case PICK_REVERSE_RAISE:
        if (!fsm->lift_started) {
            if (direction_settled(fsm, now)) {
                (void)start_lift_once(fsm, fsm->config.reverse_raise_steps,
                                      now, 0U);
            }
        } else {
            trigger_rotate(fsm, fsm->config.rotate_place_deg,
                           fsm->config.reverse_rotate_trigger_steps);
            if (!AppStepper_IsBusy(PICK_LIFT_AXIS) &&
                fsm->rotate_started != 0U &&
                !Servo_IsBusy(PICK_ROTATE_SERVO)) {
                enter_state(fsm, PICK_REVERSE_TURN, now);
            } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
                fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
            } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms)) {
                fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
            }
        }
        break;

    case PICK_REVERSE_TURN:
        if (!fsm->lift_started) {
            if (direction_settled(fsm, now)) {
                (void)start_lift_once(fsm, fsm->config.reverse_to_grab_steps,
                                      now, 0U);
            }
        } else if (!AppStepper_IsBusy(PICK_LIFT_AXIS)) {
            enter_state(fsm, PICK_REVERSE_CLOSE, now);
        } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
            fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
        }
        break;

    case PICK_REVERSE_CLOSE:
        if (!fsm->rotate_started) {
            set_gripper(fsm, fsm->config.gripper_close_deg);
            fsm->rotate_started = 1U;
            fsm->state_since_ms = now;
        } else if (!Servo_IsBusy(PICK_GRIPPER_SERVO) &&
                   timed_out(fsm, now, fsm->config.settle_ms)) {
            enter_state(fsm, PICK_REVERSE_RAISE_AFTER_CLOSE, now);
        } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms)) {
            fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
        }
        break;

    case PICK_REVERSE_RAISE_AFTER_CLOSE:
        if (!fsm->lift_started) {
            if (direction_settled(fsm, now)) {
                (void)start_lift_once(fsm,
                                      fsm->config.reverse_raise_after_close_steps,
                                      now, 0U);
            }
        } else {
            trigger_rotate(fsm, fsm->config.rotate_home_deg,
                           fsm->config.reverse_home_rotate_trigger_steps);
            if (!AppStepper_IsBusy(PICK_LIFT_AXIS) &&
                fsm->rotate_started != 0U &&
                !Servo_IsBusy(PICK_ROTATE_SERVO)) {
                enter_state(fsm, PICK_REVERSE_HOME_ROTATE, now);
            } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
                fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
            } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms)) {
                fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
            }
        }
        break;

    case PICK_REVERSE_HOME_ROTATE:
        if (!fsm->lift_started) {
            if (direction_settled(fsm, now)) {
                (void)start_lift_once(fsm, fsm->config.reverse_to_ground_steps,
                                      now, 0U);
            }
        } else if (!AppStepper_IsBusy(PICK_LIFT_AXIS)) {
            set_gripper(fsm, fsm->config.gripper_open_deg);
            enter_state(fsm, PICK_REVERSE_RELEASE, now);
        } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
            fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
        }
        break;

    case PICK_REVERSE_RELEASE:
        if (!Servo_IsBusy(PICK_GRIPPER_SERVO)) {
            enter_state(fsm, PICK_REVERSE_RESET_ZERO, now);
        } else if (timed_out(fsm, now, fsm->config.servo_timeout_ms)) {
            fail(fsm, PICK_ERROR_SERVO_TIMEOUT);
        }
        break;

    case PICK_REVERSE_RESET_ZERO:
        if (!fsm->lift_started) {
            (void)start_lift_once(fsm, fsm->config.reverse_reset_steps,
                                  now, 1U);
        } else if (!AppStepper_IsBusy(PICK_LIFT_AXIS)) {
            enter_state(fsm, PICK_REVERSE_WAIT_ZERO, now);
        } else if (timed_out(fsm, now, fsm->config.lift_timeout_ms)) {
            fail(fsm, PICK_ERROR_STEPPER_TIMEOUT);
        }
        break;

    case PICK_REVERSE_WAIT_ZERO:
        if (!AppStepper_IsBusy(PICK_LIFT_AXIS)) {
            enter_state(fsm, PICK_DONE, now);
        }
        break;

    default:
        fail(fsm, PICK_ERROR_CONFIG);
        break;
    }
}

void PickFsm_Abort(PickFsm *fsm)
{
    if (fsm == 0) return;
    AppStepper_Stop(PICK_LIFT_AXIS);
    AppStepper_Stop(PICK_TURNTABLE_AXIS);
    AppStepper_SetEnable(PICK_TURNTABLE_AXIS, 0U);
    fsm->state = PICK_ABORTED;
}

uint8_t PickFsm_IsActive(const PickFsm *fsm)
{
    return (fsm != 0 && fsm->state > PICK_IDLE &&
            fsm->state < PICK_DONE) ? 1U : 0U;
}

uint8_t PickFsm_IsDone(const PickFsm *fsm)
{
    return (fsm != 0 && fsm->state == PICK_DONE) ? 1U : 0U;
}

uint8_t PickFsm_IsReady(const PickFsm *fsm)
{
    return (fsm != 0 && fsm->state == PICK_IDLE &&
            !Servo_IsBusy(PICK_GRIPPER_SERVO) &&
            !Servo_IsBusy(PICK_ROTATE_SERVO) &&
            !AppStepper_IsBusy(PICK_LIFT_AXIS) &&
            !AppStepper_IsBusy(PICK_TURNTABLE_AXIS)) ? 1U : 0U;
}
