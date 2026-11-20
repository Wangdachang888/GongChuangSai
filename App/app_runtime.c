#include "app_runtime.h"
#include "app_mode.h"
#include "app_imu.h"
#include "app_race.h"
#include "app_servo_key.h"
#include "app_square.h"
#include "app_stepper_drive.h"
#include "app_stepper_openloop.h"
#include "bsp_d36_cmd.h"
#include "bsp_oled.h"
#include "bsp_servo.h"
#include "bsp_ulog.h"
#include "bsp_vbat.h"
#include "display_nav.h"
#include "mecanum_chassis.h"
#include "Motor.h"
#include "tim.h"
#include "vofa_debug.h"
#include <math.h>

Motor_t motor_left_front;
Motor_t motor_right_front;
Motor_t motor_left_back;
Motor_t motor_right_back;
MecanumChassis_t chassis;
#if APP_RUN_MODE != APP_MODE_RACE
static uint8_t s_square_seen_running;
#endif

#if APP_RUN_MODE != APP_MODE_RACE
static uint8_t d36_stepper_busy(uint8_t axis, void *user)
{ (void)user; return AppStepper_IsBusy((StepperAxis_t)axis); }
static uint32_t d36_stepper_remaining(uint8_t axis, void *user)
{ (void)user; return AppStepper_GetRemainingSteps((StepperAxis_t)axis); }
static HAL_StatusTypeDef d36_stepper_start(uint8_t axis, int8_t direction,
                                           uint32_t frequency_hz, void *user)
{ (void)user; return AppStepper_StartContinuous((StepperAxis_t)axis, direction, frequency_hz); }
static HAL_StatusTypeDef d36_stepper_move(uint8_t axis, int32_t steps,
                                          uint32_t frequency_hz, void *user)
{ (void)user; return AppStepper_MoveSteps((StepperAxis_t)axis, steps, frequency_hz); }
static void d36_stepper_stop(uint8_t axis, void *user)
{ (void)user; AppStepper_Stop((StepperAxis_t)axis); }
static void d36_stepper_stop_all(void *user)
{ (void)user; AppStepper_StopAll(); }
static void d36_stepper_enable(uint8_t axis, uint8_t enable, void *user)
{ (void)user; AppStepper_SetEnable((StepperAxis_t)axis, enable); }
static void d36_stepper_enable_all(uint8_t enable, void *user)
{ (void)user; AppStepper_SetEnableAll(enable); }

static const D36CmdStepperOps s_d36_stepper_ops = {
    d36_stepper_busy, d36_stepper_remaining, d36_stepper_start,
    d36_stepper_move, d36_stepper_stop, d36_stepper_stop_all,
    d36_stepper_enable, d36_stepper_enable_all
};
#endif

static void stop_chassis_outputs(void)
{
    Mecanum_Stop(&chassis);
    Motor_Stop(chassis.motor_lf);
    Motor_Stop(chassis.motor_rf);
    Motor_Stop(chassis.motor_lb);
    Motor_Stop(chassis.motor_rb);
}

static void update_chassis_imu_snapshot(void)
{
    NavImuSnapshot_t snapshot;

    snapshot.yaw_rad = app_imu_yaw_rad();
    snapshot.valid = app_imu_ok() ? 1u : 0u;
    MecanumChassis_SetImuSnapshot(&chassis, &snapshot);
}

static void update_ulog_snapshot(void)
{
    ULogAppSnapshot snapshot = {0};

    snapshot.imu_yaw_deg = app_imu_yaw_deg();
#if APP_RUN_MODE == APP_MODE_RACE
    {
        const RaceFsm *race = AppRace_Status();
        const RaceTaskPlan *plan = AppRace_TaskPlan();
        snapshot.status = (uint8_t)race->state;
        snapshot.action_index = race->action_index;
        snapshot.error = (uint8_t)race->error;
        snapshot.plan_index = plan->index;
        snapshot.plan_done = RaceTaskPlan_Done(plan);
        snapshot.finished = AppRace_LogFinished();
        if (snapshot.finished) snapshot.phase = 4U;
        else if (race->state != RACE_RUNNING ||
                 race->action_index >= race->action_count) snapshot.phase = 0U;
        else if (race->actions[race->action_index].kind == RACE_ACTION_TURN) snapshot.phase = 2U;
        else if (race->actions[race->action_index].kind == RACE_ACTION_SETTLE ||
                 race->actions[race->action_index].kind == RACE_ACTION_SCAN) snapshot.phase = 3U;
        else if (race->actions[race->action_index].kind == RACE_ACTION_MOVE) snapshot.phase = 1U;
    }
#else
    snapshot.status = Square_GetState();
    snapshot.action_index = Square_GetStep();
    if (snapshot.status == SQUARE_RUNNING) s_square_seen_running = 1U;
    snapshot.finished = (snapshot.status == SQUARE_DONE ||
                         snapshot.status == SQUARE_TIMEOUT ||
                         (s_square_seen_running && snapshot.status == SQUARE_IDLE));
    if (snapshot.status == SQUARE_DONE) snapshot.phase = 4U;
    else if (snapshot.status == SQUARE_RUNNING) {
        if (chassis.mode != 0U || chassis.target_reached != 0U ||
            fabsf(chassis.x_target) >= 1e-4f || fabsf(chassis.y_target) >= 1e-4f ||
            fabsf(chassis.vx_target) >= 1e-4f)
            snapshot.phase = ((snapshot.action_index & 1U) == 0U) ? 1U : 2U;
    }
#endif
    {
        uint8_t parked;
        uint32_t irq_state=__get_PRIMASK();
        __disable_irq();
        parked = (fabsf(motor_left_front.target_cmd) < 0.1f &&
                  fabsf(motor_right_front.target_cmd) < 0.1f &&
                  fabsf(motor_left_back.target_cmd) < 0.1f &&
                  fabsf(motor_right_back.target_cmd) < 0.1f &&
                  fabsf(motor_left_front.actual_speed) < 10.0f &&
                  fabsf(motor_right_front.actual_speed) < 10.0f &&
                  fabsf(motor_left_back.actual_speed) < 10.0f &&
                  fabsf(motor_right_back.actual_speed) < 10.0f);
        __set_PRIMASK(irq_state);
#if APP_RUN_MODE == APP_MODE_RACE
        if (AppRace_Status()->state == RACE_RUNNING) parked = 0u;
#else
        if (Square_GetState() == SQUARE_RUNNING) parked = 0u;
#endif
        ULog_PidBurstSetParked(parked);
    }
    ULog_SetAppSnapshot(&snapshot);
}
static void init_motors_and_chassis(void)
{
    Motor_Init(&motor_left_front,
               &htim10, TIM_CHANNEL_1, 4199,
               &htim11, TIM_CHANNEL_1, 4199,
               &htim2, 2.5f, 1.60f, 0.0f, 1, 1);
    Motor_Init(&motor_right_front,
               &htim9, TIM_CHANNEL_1, 4199,
               &htim9, TIM_CHANNEL_2, 4199,
               &htim3, 2.5f, 1.60f, 0.0f, 1, -1);
    Motor_Init(&motor_left_back,
               &htim1, TIM_CHANNEL_1, 8399,
               &htim1, TIM_CHANNEL_2, 8399,
               &htim4, 2.5f, 1.60f, 0.0f, -1, 1);
    Motor_Init(&motor_right_back,
               &htim1, TIM_CHANNEL_3, 8399,
               &htim1, TIM_CHANNEL_4, 8399,
               &htim5, 2.5f, 1.60f, 0.0f, -1, -1);

    MecanumChassis_Init(&chassis,
                        &motor_left_front, &motor_right_front,
                        &motor_left_back, &motor_right_back,
                        0.04f, 0.093f, 0.11f, 1.0f, 440.0f);

    chassis.pid_x.kp = 1.6f;
    chassis.pid_x.ki = 0.05f;
    chassis.pid_x.kd = 0.12f;
    chassis.pid_x.integral_max = 0.20f;
    chassis.pid_x.output_max = 1.0f;
    chassis.pid_y.kp = 1.6f;
    chassis.pid_y.ki = 0.05f;
    chassis.pid_y.kd = 0.12f;
    chassis.pid_y.integral_max = 0.20f;
    chassis.pid_y.output_max = 1.0f;
    chassis.pid_theta.kp = 3.2f;
    chassis.pid_theta.ki = 0.08f;
    chassis.pid_theta.kd = 0.18f;
    chassis.pid_theta.integral_max = 0.50f;
    chassis.pid_theta.output_max = PI;
}

void AppRuntime_Init(void)
{
#if APP_RUN_MODE == APP_MODE_SERVO
    __HAL_TIM_MOE_DISABLE(&htim1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    (void)HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
    Servo_Init();
    ServoKey_Init();
#else
#if APP_RUN_MODE != APP_MODE_RACE
    AppStepper_Init();
    Servo_Init();
    Servo_SetBothAngle(30.0f, 30.0f);
    OLED_Init();
    D36Cmd_SetStepperOps(&s_d36_stepper_ops, 0);
    D36Cmd_Init();
#else
    OLED_Init();
#endif
    __HAL_TIM_MOE_ENABLE(&htim1);
    init_motors_and_chassis();
#if APP_RUN_MODE != APP_MODE_RACE
    DisplayNav_Init();
    VOFA_Init();
#endif
    Vbat_Init();
    if (app_imu_init() != 0) {
#if APP_RUN_MODE != APP_MODE_RACE
        Error_Handler();
#endif
    }
    update_chassis_imu_snapshot();
#if APP_RUN_MODE == APP_MODE_RACE
    ULog_Init();
    AppRace_Init(&chassis);
#endif
    stop_chassis_outputs();
#if APP_RUN_MODE != APP_MODE_RACE
#if (MOTOR_TUNE != 0)
    Tune_Start(&motor_left_front);
#else
    Square_Start(&chassis);
#endif
#endif
#endif
}

void AppRuntime_Poll(void)
{
#if APP_RUN_MODE == APP_MODE_SERVO
    ServoKey_Poll();
#elif APP_RUN_MODE == APP_MODE_RACE
    app_imu_poll();
    update_chassis_imu_snapshot();
    AppRace_Poll();
    Vbat_Poll();
    update_ulog_snapshot();
    ULog_PidBurstPoll();
    ULog_Poll(&chassis);
#else
#if (MOTOR_TUNE != 0)
    Tune_Poll();
#else
    Square_Poll(&chassis);
#endif
    app_imu_poll();
    update_chassis_imu_snapshot();
    AppStepper_Poll();
    D36Cmd_Poll();
    Vbat_Poll();
    DisplayNav_Poll(&chassis);
    update_ulog_snapshot();
    ULog_PidBurstPoll();
    ULog_Poll(&chassis);
#endif
}

uint8_t AppRuntime_ControlTick(void)
{
#if APP_RUN_MODE == APP_MODE_RACE
    return AppRace_ControlTick();
#else
    return 1U;
#endif
}

void AppRuntime_MotorTick(void)
{
#if (ULOG_ENABLE != 0) && (ULOG_PID_BURST_ENABLE != 0)
    ULogPidSample sample;
#endif
    Motor_Update(&motor_left_front, 0.001f);
    Motor_Update(&motor_right_front, 0.001f);
    Motor_Update(&motor_left_back, 0.001f);
    Motor_Update(&motor_right_back, 0.001f);
#if (ULOG_ENABLE != 0) && (ULOG_PID_BURST_ENABLE != 0)
    if (ULog_PidBurstActive()) {
        sample.timestamp_ms = HAL_GetTick();
        sample.fl_target = motor_left_front.target_speed;
        sample.fl_actual = motor_left_front.actual_speed;
        sample.fl_output = motor_left_front.output;
        sample.fr_target = motor_right_front.target_speed;
        sample.fr_actual = motor_right_front.actual_speed;
        sample.fr_output = motor_right_front.output;
        sample.rl_target = motor_left_back.target_speed;
        sample.rl_actual = motor_left_back.actual_speed;
        sample.rl_output = motor_left_back.output;
        sample.rr_target = motor_right_back.target_speed;
        sample.rr_actual = motor_right_back.actual_speed;
        sample.rr_output = motor_right_back.output;
        ULog_PidBurstSampleIsr(&sample);
    }
#endif
}

void AppRuntime_ChassisTick(void)
{
    if (!g_tune_mode) MecanumChassis_Loop(&chassis, 0.01f);
}
