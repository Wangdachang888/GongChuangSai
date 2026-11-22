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
#if APP_PID_TEST_ENABLE
#include "app_pid_test.h"
#include "race_ui.h"
#include "race_config.h"
#if MOTOR_TUNE != 0
#error PID test cannot run MOTOR_TUNE
#endif
#endif

Motor_t motor_left_front;
Motor_t motor_right_front;
Motor_t motor_left_back;
Motor_t motor_right_back;
MecanumChassis_t chassis;
#if APP_RUN_MODE != APP_MODE_RACE && !APP_PID_TEST_ENABLE
static uint8_t s_square_seen_running;
#endif

#if APP_RUN_MODE != APP_MODE_RACE && !APP_PID_TEST_ENABLE
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

/* PID_ACCESS_BEGIN: main-loop-only parameter ownership; no UART/control duplication. */
static uint32_t s_pid_revision;
static uint32_t s_pid_idle_since;
static uint8_t s_pid_idle_seen;

static uint8_t pid_edit_stationary(void)
{
    Motor_t *motors[4] = {&motor_left_front, &motor_right_front,
                         &motor_left_back, &motor_right_back};
    unsigned i;
#if APP_PID_TEST_ENABLE
    if (AppPidTest_Status()->state != PID_TEST_IDLE &&
        AppPidTest_Status()->state != PID_TEST_DONE) return 0u;
#elif APP_RUN_MODE == APP_MODE_RACE
    if (AppRace_Status()->state != RACE_READY && AppRace_Status()->state != RACE_DONE)
        return 0u;
    if (AppRace_Status()->state == RACE_DONE && !AppRace_LogFinished()) return 0u;
#else
    if (Square_GetState() != SQUARE_IDLE && Square_GetState() != SQUARE_DONE)
        return 0u;
#endif
    /* Never reuse the terminal stale-speed upload fallback for parameter writes. */
    if (g_tune_mode || chassis.vx_target != 0.0f || chassis.vy_target != 0.0f ||
        chassis.omega_target != 0.0f || chassis.mode != 0u) return 0u;
    for (i=0u; i<4u; ++i) {
        Motor_t *m=motors[i];
        if (m->target_cmd != 0.0f || m->target_speed != 0.0f ||
            m->output != 0.0f || !isfinite(m->actual_speed) ||
            fabsf(m->actual_speed) >= 10.0f) return 0u;
    }
    return 1u;
}

static void pid_edit_observe(void)
{
    if (!pid_edit_stationary()) s_pid_idle_seen=0u;
    else if (!s_pid_idle_seen) {s_pid_idle_seen=1u;s_pid_idle_since=HAL_GetTick();}
}

const char *AppRuntime_PidAccess(const ULogPidRequest *request, ULogPidValues *values)
{
    Motor_t *motors[4] = {&motor_left_front, &motor_right_front,
                         &motor_left_back, &motor_right_back};
    PosPID_t *positions[3] = {&chassis.pid_x, &chassis.pid_y, &chassis.pid_theta};
    const MotorControlInfo *info=Motor_GetControlInfo();
    uint32_t irq_state;
    uint8_t axis=request->axis;
    const uint8_t config_write = request->write == 2u;
    if (axis >= 7u) return "AXIS";
    if (config_write && axis >= 4u) return "AXIS";
    if (request->write == 1u && (!isfinite(request->kp) || !isfinite(request->ki) ||
        !isfinite(request->kd) || request->kp < 0.0f || request->ki < 0.0f ||
        request->kd < 0.0f || request->kp > (axis<4u?10.0f:20.0f) ||
        request->ki > (axis<4u?8.0f:5.0f) || request->kd > 3.0f)) return "RANGE";
    if (config_write && (!isfinite(request->period_ms) || !isfinite(request->integral_max) ||
        !isfinite(request->output_max) || !isfinite(request->deadzone) ||
        !isfinite(request->ff_gain) || request->period_ms < 1.0f ||
        request->period_ms > 20.0f || request->period_ms != floorf(request->period_ms) ||
        request->integral_max < 0.0f || request->output_max < 1.0f ||
        request->output_max > 1000.0f || request->integral_max > request->output_max ||
        request->deadzone < 0.0f || request->deadzone > 500.0f ||
        request->ff_gain < 0.0f || request->ff_gain > 5.0f)) return "RANGE";
    irq_state=__get_PRIMASK(); __disable_irq();
    if (request->write) {
        if (request->revision != s_pid_revision) {
            __set_PRIMASK(irq_state); return "STALE_REVISION";
        }
        if (!pid_edit_stationary() || !s_pid_idle_seen ||
            (uint32_t)(HAL_GetTick()-s_pid_idle_since)<500u) {
            __set_PRIMASK(irq_state); return "NOT_PARKED_500MS";
        }
        if (axis<4u) {
            Motor_t *m=motors[axis];
            if (config_write) {
                m->speed_window_ms=(uint16_t)request->period_ms;
                m->integral_max=request->integral_max;
                m->output_max=request->output_max;
                m->deadzone=request->deadzone;
                m->ff_gain=request->ff_gain;
                if (m->integral > m->integral_max) m->integral=m->integral_max;
                if (m->output > m->output_max) m->output=m->output_max;
                if (m->output < -m->output_max) m->output=-m->output_max;
                m->acc_cnt=0;m->acc_ms=0u;m->speed_lpf=0.0f;
                m->integral=0.0f;m->err=0.0f;m->err_last=0.0f;
                m->output_last=0.0f;m->kick=0.0f;
            } else {
                m->kp=request->kp;m->ki=request->ki;m->kd=request->kd;
                m->integral=0.0f;m->err=0.0f;m->err_last=0.0f;
                /* Outputs are already zero; clear residual controller memory only. */
                m->output_last=0.0f;m->kick=0.0f;
            }
        } else {
            PosPID_t *p=positions[axis-4u];
            p->kp=request->kp;p->ki=request->ki;p->kd=request->kd;
            p->integral=0.0f;p->err=0.0f;p->err_last=0.0f;p->output=0.0f;
        }
        ++s_pid_revision;
    }
    if (axis<4u) {
        Motor_t *m=motors[axis];
        values->kp=m->kp;values->ki=m->ki;values->kd=m->kd;
        values->integral_max=m->integral_max;values->output_max=m->output_max;
        /* Host fixtures may not call Motor_Init; use the same defaults until
         * the runtime configuration has been initialized. */
        if (m->speed_window_ms == 0u) {
            values->period_ms=info->period_ms; values->ff_gain=info->ff_gain;
            values->deadzone=info->deadzone; values->ff_min_speed=info->ff_min_speed;
        } else {
            values->period_ms=(float)m->speed_window_ms; values->ff_gain=m->ff_gain;
            values->deadzone=m->deadzone; values->ff_min_speed=m->ff_min_speed;
        }
    } else {
        PosPID_t *p=positions[axis-4u];
        values->kp=p->kp;values->ki=p->ki;values->kd=p->kd;
        values->integral_max=p->integral_max;values->output_max=p->output_max;
        values->period_ms=10.0f;values->ff_gain=0.0f;values->deadzone=0.0f;
        values->ff_min_speed=0.0f;
    }
    values->revision=s_pid_revision;
    values->editable=pid_edit_stationary() && s_pid_idle_seen &&
        (uint32_t)(HAL_GetTick()-s_pid_idle_since)>=500u;
    __set_PRIMASK(irq_state);
    values->ramp=axis<4u?info->ramp:0.0f;
    values->slew=axis<4u?info->slew:0.0f;
    values->lpf=axis<4u?info->lpf:0.0f;
    values->kick_step=axis<4u?info->kick_step:0.0f;
    values->kick_max=axis<4u?info->kick_max:0.0f;
    values->stuck_ratio=axis<4u?info->stuck_ratio:0.0f;
    return "OK";
}
/* PID_ACCESS_END */

static void update_chassis_imu_snapshot(void)
{
    NavImuSnapshot_t snapshot;

    snapshot.yaw_rad = app_imu_yaw_rad();
    snapshot.valid = app_imu_ok() ? 1u : 0u;
    MecanumChassis_SetImuSnapshot(&chassis, &snapshot);
}

/* ULOG_SNAPSHOT_BEGIN */
static void update_ulog_snapshot(void)
{
    ULogAppSnapshot snapshot = {0};

    snapshot.imu_yaw_deg = app_imu_yaw_deg();
#if APP_PID_TEST_ENABLE
    snapshot.status = (uint8_t)AppPidTest_Status()->state;
    snapshot.action_index = AppPidTest_Status()->step;
    snapshot.error = (uint8_t)AppPidTest_Status()->reason;
    snapshot.phase = snapshot.status == PID_TEST_RUNNING ? 1u : 0u;
#elif APP_RUN_MODE == APP_MODE_RACE
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
#if APP_PID_TEST_ENABLE
        /* No terminal stale-feedback fallback. Continue sampling after stop;
         * the same real zero-output gate protects cache upload and PID access. */
        parked = parked &&
            chassis.mode == 0u && chassis.vx_target == 0.0f &&
            chassis.vy_target == 0.0f && chassis.omega_target == 0.0f &&
            motor_left_front.target_cmd == 0.0f && motor_right_front.target_cmd == 0.0f &&
            motor_left_back.target_cmd == 0.0f && motor_right_back.target_cmd == 0.0f &&
            motor_left_front.target_speed == 0.0f && motor_right_front.target_speed == 0.0f &&
            motor_left_back.target_speed == 0.0f && motor_right_back.target_speed == 0.0f &&
            motor_left_front.output == 0.0f && motor_right_front.output == 0.0f &&
            motor_left_back.output == 0.0f && motor_right_back.output == 0.0f;
#endif
#if APP_RUN_MODE == APP_MODE_RACE
        /* Control is stopped on terminal faults, so actual_speed may never
         * refresh. Authorize only final diagnostics, without changing feedback
         * or restarting control. The logger still waits 500ms continuously. */
        if (snapshot.status == RACE_FAULT || snapshot.status == RACE_ABORTED) {
            snapshot.terminal_outputs_stopped =
                motor_left_front.target_cmd == 0.0f &&
                motor_right_front.target_cmd == 0.0f &&
                motor_left_back.target_cmd == 0.0f &&
                motor_right_back.target_cmd == 0.0f &&
                motor_left_front.target_speed == 0.0f &&
                motor_right_front.target_speed == 0.0f &&
                motor_left_back.target_speed == 0.0f &&
                motor_right_back.target_speed == 0.0f &&
                motor_left_front.output == 0.0f &&
                motor_right_front.output == 0.0f &&
                motor_left_back.output == 0.0f &&
                motor_right_back.output == 0.0f;
        }
#endif
        __set_PRIMASK(irq_state);
#if APP_PID_TEST_ENABLE
        if (snapshot.status == PID_TEST_RUNNING) parked = 0u;
        snapshot.finished = snapshot.status >= PID_TEST_DONE && parked;
        if (snapshot.finished) snapshot.phase = 4u;
#elif APP_RUN_MODE == APP_MODE_RACE
        if (AppRace_Status()->state == RACE_RUNNING) parked = 0u;
#else
        if (Square_GetState() == SQUARE_RUNNING) parked = 0u;
#endif
        ULog_PidBurstSetParked(parked);
    }
    ULog_SetAppSnapshot(&snapshot);
}
/* ULOG_SNAPSHOT_END */

/* PID_TEST_RUNTIME_BEGIN */
#if APP_PID_TEST_ENABLE
static RaceInput s_pid_key;
static uint32_t s_pid_boot, s_pid_frames, s_pid_last_imu;
static volatile uint32_t s_pid_last_poll;
static volatile uint8_t s_pid_guard_active, s_pid_cutoff;
static uint8_t s_pid_capture_closed;

static const char *pid_test_state_name(AppPidTestState state)
{
    switch (state) {
    case PID_TEST_IDLE: return "READY";
    case PID_TEST_RUNNING: return "RUNNING";
    case PID_TEST_DONE: return "DONE";
    case PID_TEST_CANCELLED: return "CANCELLED";
    case PID_TEST_TIMEOUT: return "TIMEOUT";
    default: return "FAULT";
    }
}

static void pid_test_commands(uint32_t now, uint8_t ready, AppPidTestReason fault)
{
    ULogPidTestRequest request;
    const AppPidTestStatus *status;
    uint8_t ok = 0u;
    if (!ULog_TakePidTestRequest(&request)) return;
    status = AppPidTest_Status();
    if (request.command == ULOG_PID_TEST_STATUS) {
        ULog_ReplyPidTest(request.id, pid_test_state_name(status->state),
                          (uint8_t)status->state, (uint8_t)status->reason, status->action);
        return;
    }
    if (request.command == ULOG_PID_TEST_STOP) {
        if (status->state == PID_TEST_RUNNING) {
            AppPidTest_Stop(PID_TEST_REASON_MANUAL);
            ok = 1u;
        }
        status = AppPidTest_Status();
        ULog_ReplyPidTest(request.id, ok ? "STOPPED" : "NOT_RUNNING",
                          (uint8_t)status->state, (uint8_t)status->reason, status->action);
        return;
    }
    if (request.command == ULOG_PID_TEST_RESET) {
        ok = (fault == PID_TEST_REASON_NONE && app_imu_bias_ready() &&
              s_pid_idle_seen && (uint32_t)(now - s_pid_idle_since) >= 500u &&
              AppPidTest_Status()->state != PID_TEST_RUNNING) && AppPidTest_Rearm();
        status = AppPidTest_Status();
        ULog_ReplyPidTest(request.id, ok ? "READY" : "NOT_READY",
                          (uint8_t)status->state, (uint8_t)status->reason, status->action);
        return;
    }
    if (request.command == ULOG_PID_TEST_RUN) {
        if (!ready || !ULog_PidBurstActive() || !AppPidTest_SelectAction(request.action)) {
            ULog_ReplyPidTest(request.id, "REJECTED", (uint8_t)status->state,
                              (uint8_t)status->reason, status->action);
            return;
        }
        ok = AppPidTest_Start(now);
        if (ok) s_pid_capture_closed = 0u;
        status = AppPidTest_Status();
        ULog_ReplyPidTest(request.id, ok ? "STARTED" : "REJECTED",
                          (uint8_t)status->state, (uint8_t)status->reason, status->action);
    }
}

static void pid_test_poll(void)
{
    uint32_t now = HAL_GetTick(), frames = app_imu_frames();
    uint32_t irq;
    uint8_t event, ready;
    AppPidTestReason fault = PID_TEST_REASON_NONE;
    Motor_t *motors[4] = {&motor_left_front, &motor_right_front,
                         &motor_left_back, &motor_right_back};
    unsigned i;
    if (frames != s_pid_frames) { s_pid_frames = frames; s_pid_last_imu = now; }
    if (!app_imu_ok() || frames == 0u ||
        (uint32_t)(now - s_pid_last_imu) >= RACE_IMU_STALE_MS)
        fault = PID_TEST_REASON_IMU;
    for (i = 0u; i < 4u; ++i) {
        if (!isfinite(motors[i]->actual_speed) || !isfinite(motors[i]->output) ||
            !isfinite(motors[i]->target_speed) || !isfinite(motors[i]->target_cmd))
            fault = PID_TEST_REASON_FEEDBACK;
    }
    if (!chassis.enable || g_tune_mode) fault = PID_TEST_REASON_CONFIG;
    if (!ULog_IsRecording() || !ULog_PidBurstActive()) fault = PID_TEST_REASON_CAPTURE;
    if (s_pid_cutoff) fault = PID_TEST_REASON_LOOP;
    ready = fault == PID_TEST_REASON_NONE && app_imu_bias_ready() &&
        (uint32_t)(now - s_pid_boot) >= RACE_WARMUP_MS &&
        AppPidTest_Status()->state == PID_TEST_IDLE && pid_edit_stationary() &&
        s_pid_idle_seen && (uint32_t)(now - s_pid_idle_since) >= 500u;
    pid_test_commands(now, ready, fault);
    /* A command may have selected an action or rearmed the state. Re-evaluate
     * the physical start path below; no command ever runs from the IRQ. */
    ready = fault == PID_TEST_REASON_NONE && app_imu_bias_ready() &&
        (uint32_t)(now - s_pid_boot) >= RACE_WARMUP_MS &&
        AppPidTest_Status()->state == PID_TEST_IDLE && pid_edit_stationary() &&
        s_pid_idle_seen && (uint32_t)(now - s_pid_idle_since) >= 500u;
    event = RaceInput_Poll(&s_pid_key, now,
        AppPidTest_Status()->state == PID_TEST_RUNNING, ready);
    if ((event & RACE_INPUT_PRESS) && AppPidTest_Status()->state == PID_TEST_RUNNING)
        AppPidTest_Stop(PID_TEST_REASON_MANUAL);
    /* Recheck readiness at release: an earlier valid press is not a lease. */
    if ((event & RACE_INPUT_SHORT_RELEASE) && ready) (void)AppPidTest_Start(now);
    AppPidTest_Poll(now, fault);
    if (AppPidTest_Status()->state != PID_TEST_IDLE &&
        AppPidTest_Status()->state != PID_TEST_RUNNING &&
        !s_pid_capture_closed && ULog_PidBurstActive()) {
        ULog_PidBurstStop();
        s_pid_capture_closed = 1u;
    }
    irq = __get_PRIMASK(); __disable_irq();
    if (s_pid_cutoff) AppPidTest_Stop(PID_TEST_REASON_LOOP);
    s_pid_guard_active = AppPidTest_Status()->state == PID_TEST_RUNNING;
    s_pid_last_poll = now;
    s_pid_cutoff = 0u;
    __set_PRIMASK(irq);
}
#endif
/* PID_TEST_RUNTIME_END */
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
#if APP_RUN_MODE != APP_MODE_RACE && !APP_PID_TEST_ENABLE
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
    ULog_SetPidHandler(AppRuntime_PidAccess);
#if APP_RUN_MODE != APP_MODE_RACE && !APP_PID_TEST_ENABLE
    DisplayNav_Init();
    VOFA_Init();
#endif
    Vbat_Init();
    if (app_imu_init() != 0) {
#if APP_RUN_MODE != APP_MODE_RACE && !APP_PID_TEST_ENABLE
        Error_Handler();
#endif
    }
    update_chassis_imu_snapshot();
#if APP_PID_TEST_ENABLE
    ULog_Init();
    Mecanum_SetSpeedLimit(&chassis, RACE_MOVE_SPEED_MPS, RACE_TURN_SPEED_RADPS);
    AppPidTest_Init(&chassis);
    s_pid_boot = s_pid_last_poll = s_pid_last_imu = HAL_GetTick();
    s_pid_frames = app_imu_frames();
    RaceInput_Init(&s_pid_key, s_pid_boot);
#elif APP_RUN_MODE == APP_MODE_RACE
    ULog_Init();
    AppRace_Init(&chassis);
#endif
    stop_chassis_outputs();
#if APP_RUN_MODE != APP_MODE_RACE && !APP_PID_TEST_ENABLE
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
#elif APP_PID_TEST_ENABLE
    app_imu_poll();
    update_chassis_imu_snapshot();
    pid_test_poll();
    Vbat_Poll();
    update_ulog_snapshot();
    pid_edit_observe();
    ULog_PidBurstPoll();
    ULog_Poll(&chassis);
#elif APP_RUN_MODE == APP_MODE_RACE
    app_imu_poll();
    update_chassis_imu_snapshot();
    AppRace_Poll();
    Vbat_Poll();
    update_ulog_snapshot();
    pid_edit_observe();
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
    pid_edit_observe();
    ULog_PidBurstPoll();
    ULog_Poll(&chassis);
#endif
}

uint8_t AppRuntime_ControlTick(void)
{
#if APP_PID_TEST_ENABLE
    /* Emergency output cutoff only, as in the race watchdog. No test phase
     * progression, formatting, transmission or allocation in the ISR. */
    if (s_pid_guard_active &&
        (uint32_t)(HAL_GetTick() - s_pid_last_poll) >= RACE_IMU_STALE_MS) {
        s_pid_cutoff = 1u;
        stop_chassis_outputs();
        return 0u;
    }
    return 1u;
#elif APP_RUN_MODE == APP_MODE_RACE
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
