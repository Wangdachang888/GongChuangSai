#include "app_pick_control.h"
#include "app_pick_config.h"
#include "app_pick_fsm.h"
#include "app_stepper_openloop.h"
#include "bsp_servo.h"
#include "main.h"

#define PICK_KEY_PORT GPIOE
#define PICK_KEY_PIN  GPIO_PIN_0
#define PICK_KEY_DEBOUNCE_MS 20U
#define PICK_KEY_LONG_MS 700U

static PickFsm s_fsm;
static uint8_t s_raw_pressed;
static uint8_t s_stable_pressed;
static uint8_t s_armed;
static uint8_t s_ignore_until_release;
static uint8_t s_long_press;
static uint32_t s_edge_ms;
static uint32_t s_press_ms;

static const PickConfig s_config = {
    .rotate_home_deg = PICK_ROTATE_HOME_DEG,
    .rotate_place_deg = PICK_ROTATE_PLACE_DEG,
    .gripper_open_deg = PICK_GRIPPER_OPEN_DEG,
    .gripper_preclose_deg = PICK_GRIPPER_PRECLOSE_DEG,
    .gripper_close_deg = PICK_GRIPPER_CLOSE_DEG,
    .forward_approach_steps = PICK_LIFT_APPROACH_STEPS,
    .forward_to_grab_steps = PICK_LIFT_TO_GRAB_STEPS,
    .forward_raise_steps = PICK_LIFT_FORWARD_RAISE_STEPS,
    .forward_reset_steps = PICK_LIFT_FORWARD_RESET_STEPS,
    .reverse_raise_steps = PICK_LIFT_REVERSE_RAISE_STEPS,
    .reverse_to_grab_steps = PICK_LIFT_REVERSE_TO_GRAB_STEPS,
    .reverse_raise_after_close_steps = PICK_LIFT_REVERSE_RAISE_AFTER_CLOSE_STEPS,
    .reverse_to_ground_steps = PICK_LIFT_REVERSE_TO_GROUND_STEPS,
    .reverse_reset_steps = PICK_LIFT_REVERSE_RESET_STEPS,
    .lift_frequency_hz = PICK_LIFT_FREQUENCY_HZ,
    .lift_accel_rpms = PICK_LIFT_ACCEL_RPMS,
    .rotate_speed_dps = PICK_ROTATE_SPEED_DPS,
    .rotate_accel_dps2 = PICK_ROTATE_ACCEL_DPS2,
    .gripper_speed_dps = PICK_GRIPPER_SPEED_DPS,
    .gripper_accel_dps2 = PICK_GRIPPER_ACCEL_DPS2,
    .lift_before_rotate_done_ms = PICK_LIFT_BEFORE_ROTATE_DONE_MS,
    .servo_timeout_ms = PICK_SERVO_TIMEOUT_MS,
    .lift_timeout_ms = PICK_LIFT_TIMEOUT_MS,
    .settle_ms = PICK_SETTLE_MS,
    .direction_settle_ms = PICK_DIRECTION_SETTLE_MS,
    .forward_rotate_trigger_steps = PICK_FORWARD_ROTATE_TRIGGER_STEPS,
    .reverse_rotate_trigger_steps = PICK_REVERSE_ROTATE_TRIGGER_STEPS,
    .reverse_home_rotate_trigger_steps = PICK_REVERSE_HOME_ROTATE_TRIGGER_STEPS,
    .forward_cycle_count = PICK_FORWARD_CYCLE_COUNT,
    .turntable_servo_speed_dps = PICK_TURNTABLE_SPEED_DPS,
    .turntable_servo_accel_dps2 = PICK_TURNTABLE_ACCEL_DPS2
};

static uint8_t key_pressed(void)
{
    return HAL_GPIO_ReadPin(PICK_KEY_PORT, PICK_KEY_PIN) == GPIO_PIN_RESET;
}

void PickControl_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Pin = PICK_KEY_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PICK_KEY_PORT, &gpio);

    PickFsm_Init(&s_fsm, &s_config);
    (void)PickFsm_SetReverseTurntableAngle(
        &s_fsm, PICK_REVERSE_TURNTABLE_ANGLE_DEG);

    s_raw_pressed = key_pressed();
    s_stable_pressed = s_raw_pressed;
    s_ignore_until_release = s_raw_pressed;
    s_armed = 0U;
    s_long_press = 0U;
    s_edge_ms = HAL_GetTick();
    s_press_ms = 0U;
    PickFsm_Reset(&s_fsm, s_edge_ms);
}

void PickControl_Poll(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t raw_pressed = key_pressed();

    PickFsm_Poll(&s_fsm, now);
    if (PickFsm_IsDone(&s_fsm) || s_fsm.state == PICK_FAULT ||
        s_fsm.state == PICK_ABORTED) {
        if (!Servo_IsBusyAny() && !AppStepper_IsBusy(STEPPER_AXIS_1)) {
            PickFsm_Reset(&s_fsm, now);
        }
    }

    if (raw_pressed != s_raw_pressed) {
        s_raw_pressed = raw_pressed;
        s_edge_ms = now;
    }
    if ((uint32_t)(now - s_edge_ms) < PICK_KEY_DEBOUNCE_MS) return;

    if (s_stable_pressed == raw_pressed) {
        if (raw_pressed && !s_long_press && s_armed &&
            (uint32_t)(now - s_press_ms) >= PICK_KEY_LONG_MS) {
            s_long_press = 1U;
        }
        return;
    }

    s_stable_pressed = raw_pressed;
    if (raw_pressed) {
        s_press_ms = now;
        s_long_press = 0U;
        s_armed = (s_ignore_until_release == 0U) ? 1U : 0U;
    } else if (s_armed && PickFsm_IsReady(&s_fsm)) {
        if (s_long_press) {
            (void)PickFsm_StartReverse(&s_fsm, now);
        } else {
            (void)PickFsm_Start(&s_fsm, now);
        }
        s_armed = 0U;
    } else {
        s_armed = 0U;
        s_ignore_until_release = 0U;
    }
}

uint8_t PickControl_IsBusy(void)
{
    return PickFsm_IsActive(&s_fsm);
}
