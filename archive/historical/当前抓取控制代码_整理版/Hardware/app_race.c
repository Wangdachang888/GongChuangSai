#include "app_mode.h"
#include "app_race.h"
#include "race_config.h"
#include "race_vision.h"
#include "app_imu.h"
#include "bsp_oled.h"
#include "app_pick_fsm.h"
#include "app_stepper_openloop.h"
#include "bsp_servo.h"
#include "race_task_plan.h"
#include <stdio.h>
#include <string.h>

#if APP_RUN_MODE == APP_MODE_RACE
static RaceFsm s_race;
static MecanumChassis_t *s_chassis;
static uint32_t s_boot, s_frames, s_last_imu;
static volatile uint32_t s_last_poll;
static volatile uint8_t s_active, s_control, s_stalled;
static uint8_t s_key_raw, s_key_stable, s_key_armed, s_key_long, s_key_start;
static uint32_t s_key_changed, s_key_pressed;
static uint32_t s_screen_time, s_page_time;
static uint8_t s_page;
static PickFsm s_pick;
static RaceTaskPlan s_plan;
static uint8_t s_direct_cycle_done;
#if (RACE_PICK_DIRECT_KEY_TEST != 0U)
static void start_direct_pick(uint32_t now, uint8_t reverse)
{
    if (PickFsm_IsReady(&s_pick)) {
        if (reverse) (void)PickFsm_StartReverse(&s_pick, now);
        else (void)PickFsm_Start(&s_pick, now);
    }
}
#endif


static const PickConfig s_pick_config = {
    RACE_PICK_ROTATE_HOME_DEG,
    RACE_PICK_ROTATE_PLACE_DEG,
    RACE_PICK_GRIPPER_OPEN_DEG,
    RACE_PICK_GRIPPER_PRECLOSE_DEG,
    RACE_PICK_GRIPPER_CLOSE_DEG,
    RACE_PICK_LIFT_APPROACH_STEPS,
    RACE_PICK_LIFT_DROP_TO_GRAB_STEPS,
    RACE_PICK_FORWARD_TO_170_STEPS,
    RACE_PICK_FORWARD_RESET_LIFT_STEPS,
    RACE_PICK_REVERSE_TO_175_STEPS,
    RACE_PICK_REVERSE_TO_141_STEPS,
    RACE_PICK_REVERSE_TO_175_AFTER_CLOSE_STEPS,
    RACE_PICK_REVERSE_TO_GROUND_STEPS,
    RACE_PICK_REVERSE_RESET_LIFT_STEPS,
    RACE_PICK_LIFT_FREQUENCY_HZ,
    RACE_PICK_LIFT_ACCEL_RPMS,
    RACE_PICK_ROTATE_SPEED_DPS,
    RACE_PICK_ROTATE_ACCEL_DPS2,
    RACE_PICK_GRIPPER_SPEED_DPS,
    RACE_PICK_GRIPPER_ACCEL_DPS2,
    RACE_PICK_LIFT_BEFORE_ROTATE_DONE_MS,
    RACE_PICK_SERVO_TIMEOUT_MS,
    RACE_PICK_LIFT_TIMEOUT_MS,
    RACE_PICK_SETTLE_MS,
    RACE_PICK_DIRECTION_SETTLE_MS,
    RACE_PICK_ROTATE_TRIGGER_STEPS,
    RACE_PICK_REVERSE_ROTATE_TRIGGER_STEPS,
    RACE_PICK_REVERSE_HOME_ROTATE_TRIGGER_STEPS,
    RACE_PICK_FORWARD_CYCLES,
    RACE_PICK_TURNTABLE_SERVO_SPEED_DPS,
    RACE_PICK_TURNTABLE_SERVO_ACCEL_DPS2
};

static void stop_outputs(void)
{
    Mecanum_Stop(s_chassis);
    Motor_Stop(s_chassis->motor_lf);
    Motor_Stop(s_chassis->motor_rf);
    Motor_Stop(s_chassis->motor_lb);
    Motor_Stop(s_chassis->motor_rb);
}

static RaceFeedback feedback(uint32_t now)
{
    RaceFeedback result;
    uint32_t saved_irq = __get_PRIMASK();
    __disable_irq();
    result.x = s_chassis->x_pos;
    result.y = s_chassis->y_pos;
    result.yaw = s_chassis->theta_pos;
    result.vx = s_chassis->vx_actual;
    result.vy = s_chassis->vy_actual;
    result.omega = s_chassis->omega_actual;
    __set_PRIMASK(saved_irq);
    result.imu_valid = app_imu_ok() && s_frames != 0U &&
                       (uint32_t)(now - s_last_imu) < RACE_IMU_STALE_MS;
    return result;
}

static uint8_t warmed_up(uint32_t now)
{
    return app_imu_bias_ready() && (uint32_t)(now - s_boot) >= RACE_WARMUP_MS;
}

static void poll_key(const RaceFeedback *current, uint32_t now)
{
    uint8_t raw = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET;
#if (RACE_PICK_DIRECT_KEY_TEST != 0U)
    (void)current;
#endif
    if (raw != s_key_raw) {
        s_key_raw = raw;
        s_key_changed = now;
    }
    if ((uint32_t)(now - s_key_changed) < 20U) return;
    if (!s_key_armed) {
        s_key_stable = raw;
        if (!raw) s_key_armed = 1U;
        return;
    }
    if (raw != s_key_stable) {
        s_key_stable = raw;
        if (raw) {
            s_key_pressed = now;
            s_key_long = 0U;
#if (RACE_PICK_DIRECT_KEY_TEST != 0U)
            s_key_start = PickFsm_IsReady(&s_pick);
#else
            s_key_start = s_race.state == RACE_READY && warmed_up(now) && current->imu_valid;
#endif
            if (s_race.state == RACE_RUNNING) Race_Abort(&s_race);
        }
#if (RACE_PICK_DIRECT_KEY_TEST != 0U)
        else if (s_key_start) {
#else
        else if (!s_key_long && s_key_start) {
#endif
#if (RACE_PICK_DIRECT_KEY_TEST != 0U)
            start_direct_pick(now, s_key_long);
#else
            if (s_race.state == RACE_READY) {
            (void)Race_StartQrRoute(&s_race, current, now);
            }
#endif
        }
    }
    if (raw && !s_key_long && (uint32_t)(now - s_key_pressed) >= 700U) {
        s_key_long = 1U;
        if (s_race.state == RACE_READY) {
            (void)Race_SelectZone(&s_race, s_race.zone == 1U ? 2U : 1U);
        }
    }
}

static void text_line(uint8_t row, const char *text)
{
    uint8_t column = 0U;
    while (*text && column < 21U) {
        OLED_ShowChar((uint8_t)(column * 6U), row, (uint8_t)*text++, 12U, 1U);
        column++;
    }
}

static void poll_display(uint32_t now)
{
    static const char *const states[] = {"READY", "RUNNING", "MATERIAL READY", "FAULT", "STOPPED"};
    static const char *const actions[] = {"MOVE", "TURN", "SETTLE", "SCAN"};
    char line[32];
    uint8_t column;
    if (s_page == 0U && (uint32_t)(now - s_screen_time) >= 200U) {
        s_screen_time = now;
        memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
        (void)snprintf(line, sizeof(line), "RACE Z%u %s", s_race.zone, states[s_race.state]);
        text_line(0U, line);
        if (s_race.state == RACE_READY) {
            text_line(12U, warmed_up(now) ? "SHORT:START" : "IMU WARMUP: KEEP STILL");
            text_line(24U, "LONG:ZONE 1/2");
        } else if (s_race.state == RACE_RUNNING) {
            (void)snprintf(line, sizeof(line), "%u/%u %s", s_race.action_index + 1U,
                           s_race.action_count, actions[s_race.actions[s_race.action_index].kind]);
            text_line(12U, line);
            text_line(24U, "PRESS:STOP");
        } else {
            (void)snprintf(line, sizeof(line), "ERROR:%u RESET TO ARM", (unsigned)s_race.error);
            text_line(12U, line);
        }
        text_line(36U, s_race.task_code);
        text_line(48U, RaceVision_Healthy() ? "USART3:RX READY" : "USART3:FAULT");
    }
    if ((uint32_t)(now - s_page_time) < 2U) return;
    s_page_time = now;
    OLED_WR_Byte((uint8_t)(0xb0U + s_page), OLED_CMD);
    OLED_WR_Byte(0x00U, OLED_CMD);
    OLED_WR_Byte(0x10U, OLED_CMD);
    for (column = 0U; column < 128U; column++) OLED_WR_Byte(OLED_GRAM[column][s_page], OLED_DATA);
    s_page = (uint8_t)((s_page + 1U) & 7U);
}

void AppRace_Init(MecanumChassis_t *chassis)
{
    s_chassis = chassis;
    s_direct_cycle_done = 0U;
    Race_Init(&s_race);
    s_boot = HAL_GetTick();
    s_last_poll = s_boot;
    s_last_imu = s_boot;
    s_frames = app_imu_frames();
    s_active = 0U;
    s_stalled = 0U;
    s_control = 1U;
    s_key_armed = 0U;
    s_key_raw = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET;
    s_key_stable = s_key_raw;
    s_key_changed = s_boot;
    if (RACE_PICK_ENABLE != 0U) {
        AppStepper_Init();
        Servo_Init();
#if (RACE_PICK_TURNTABLE_ENABLE == 0U)
        Servo_Disable(SERVO_3);
#else
        /* Hold PC7 output off at boot; the first tray command enables it. */
        Servo_Disable(SERVO_3);
#endif
    }
    PickFsm_Init(&s_pick, &s_pick_config);
    (void)PickFsm_SetReverseTurntableAngle(
        &s_pick, RACE_PICK_REVERSE_TURNTABLE_ANGLE_DEG);
    if (RACE_PICK_DIRECT_KEY_TEST != 0U) {
        PickFsm_Reset(&s_pick, s_boot);
    }
    stop_outputs();
    Mecanum_SetSpeedLimit(chassis, RACE_MOVE_SPEED_MPS, RACE_TURN_SPEED_RADPS);
    Mecanum_SetTolerance(chassis, RACE_POSITION_TOL_M * 0.5f, RACE_YAW_TOL_RAD * 0.5f);
    if (!RaceVision_Init()) Race_Fail(&s_race, RACE_ERROR_LINK);
    if (!app_imu_ok()) Race_Fail(&s_race, RACE_ERROR_IMU);
}

uint8_t AppRace_ControlTick(void)
{
    if (s_chassis == NULL) return 0U;
    if (s_active && (uint32_t)(HAL_GetTick() - s_last_poll) >= RACE_IMU_STALE_MS) {
        s_stalled = 1U;
        s_control = 0U;
        s_active = 0U;
        stop_outputs();
        PickFsm_Abort(&s_pick);
    }
    return s_control;
}

void AppRace_Poll(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t frames = app_imu_frames();
    RaceState previous_state = s_race.state;
    uint8_t previous_index = s_race.action_index;
    uint32_t saved_irq;
    RaceFeedback current;
    char code[RACE_TASK_CODE_SIZE];
    if (frames != s_frames) {
        s_frames = frames;
        s_last_imu = now;
    }
    current = feedback(now);
    if (s_stalled) Race_Fail(&s_race, RACE_ERROR_LOOP_STALL);
    if (!RaceVision_Healthy() && (s_race.state == RACE_READY || s_race.state == RACE_RUNNING)) {
        Race_Fail(&s_race, RACE_ERROR_LINK);
    }
    if (RaceVision_Poll(code) && Race_SubmitQr(&s_race, s_race.scan_session, code)) {
        if (!RaceTaskPlan_Build(&s_plan,
                                (const uint8_t (*)[3])s_race.colors,
                                (const uint8_t (*)[3])s_race.positions) ||
            !RaceTaskPlan_Start(&s_plan)) {
            Race_Fail(&s_race, RACE_ERROR_CONFIG);
        }
    }
    poll_key(&current, now);
    Race_Tick(&s_race, &current, now);
    if (previous_state == RACE_RUNNING && s_race.state == RACE_DONE &&
        RaceTaskPlan_Current(&s_plan) != 0 &&
        RaceTaskPlan_Current(&s_plan)->kind == TASK_ACTION_PICK) {
        if ((RACE_PICK_ENABLE != 0U) && !PickFsm_Start(&s_pick, now)) {
            s_race.error = RACE_ERROR_CONFIG;
            s_race.state = RACE_FAULT;
        }
    }
    if (PickFsm_IsActive(&s_pick) || s_pick.state == PICK_RESETTING) PickFsm_Poll(&s_pick, now);
#if (RACE_PICK_DIRECT_KEY_TEST != 0U)
    if (PickFsm_IsDone(&s_pick)) {
        s_direct_cycle_done = 1U;
        PickFsm_Reset(&s_pick, now);
    }
#endif
    if (PickFsm_IsDone(&s_pick) && RaceTaskPlan_Current(&s_plan) != 0 &&
        RaceTaskPlan_Current(&s_plan)->kind == TASK_ACTION_PICK) {
        (void)RaceTaskPlan_Advance(&s_plan);
    }
    if (s_race.state == RACE_RUNNING &&
        (previous_state != RACE_RUNNING || previous_index != s_race.action_index)) {
        const RaceAction *action = &s_race.actions[s_race.action_index];
        saved_irq = __get_PRIMASK();
        __disable_irq();
        if (!s_stalled && (action->kind == RACE_ACTION_MOVE || action->kind == RACE_ACTION_TURN)) {
            s_chassis->body_frame = 0U;
            Mecanum_SetPositionMode(s_chassis, s_race.target_x, s_race.target_y, s_race.target_yaw);
        }
        __set_PRIMASK(saved_irq);
        if (action->kind == RACE_ACTION_SCAN && !RaceVision_BeginScan()) {
            Race_Fail(&s_race, RACE_ERROR_LINK);
        }
    }
    saved_irq = __get_PRIMASK();
    __disable_irq();
    if (s_stalled) Race_Fail(&s_race, RACE_ERROR_LOOP_STALL);
    s_active = (s_race.state == RACE_RUNNING) || PickFsm_IsActive(&s_pick);
    s_last_poll = now;
    if (s_race.state != RACE_READY && s_race.state != RACE_RUNNING &&
        !PickFsm_IsActive(&s_pick)) {
        if (s_control) stop_outputs();
        s_control = 0U;
    }
    __set_PRIMASK(saved_irq);
    if (!s_active) RaceVision_EndScan();
    poll_display(now);
}

const RaceFsm *AppRace_Status(void)
{
    return &s_race;
}

const RaceTaskPlan *AppRace_TaskPlan(void)
{
    return &s_plan;
}

uint8_t AppRace_LogFinished(void)
{
    if (s_race.state == RACE_FAULT || s_race.state == RACE_ABORTED ||
        s_pick.state == PICK_FAULT || s_pick.state == PICK_ABORTED) return 1U;
    if (PickFsm_IsActive(&s_pick) ||
        AppStepper_IsBusy(STEPPER_AXIS_1) ||
        Servo_IsBusy(SERVO_1) || Servo_IsBusy(SERVO_2) ||
        Servo_IsBusy(SERVO_3)) return 0U;
#if (RACE_PICK_DIRECT_KEY_TEST != 0U)
    /* Boot reset is not a completed test. Wait for the post-test reset. */
    return s_direct_cycle_done && PickFsm_IsReady(&s_pick);
#else
    /* The current executor parks after the material route and optional pick.
     * This is NOT proof that the entire planned competition is complete.
     * When remaining actions are connected, use RaceTaskPlan_Done(&s_plan)
     * here instead of the material-route end state. */
    return s_race.state == RACE_DONE;
#endif
}
#endif
