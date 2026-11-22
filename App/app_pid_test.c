#include "app_pid_test.h"
#if APP_PID_TEST_ENABLE
#include <math.h>

static MecanumChassis_t *s_chassis;
static AppPidTestStatus s_status;
static uint32_t s_entered;
static uint8_t s_action;

/* Normal: initial zero, (A,B,C)x3 with six levels each, final zero.
 * Creep retains its separate (+,0,-,0)x3 diagnostic. No PID resets. */
static uint8_t final_step(void)
{
    if (s_action == PID_TEST_ACTION_BODE && APP_PID_TEST_FREQUENCY) return 2u;
    return (s_action == PID_TEST_ACTION_CREEP || s_action == PID_TEST_ACTION_STALL) ? 13u : 55u;
}
static uint32_t duration(uint8_t step)
{
    static const uint16_t hold[6] = {500u,1000u,1500u,500u,1000u,1500u};
    if (step == 0u) return (s_action == PID_TEST_ACTION_BODE && APP_PID_TEST_FREQUENCY) ? APP_PID_TEST_FR_WARMUP_MS : 1000u;
    if (step == final_step()) return (s_action == PID_TEST_ACTION_BODE && APP_PID_TEST_FREQUENCY) ? APP_PID_TEST_FR_SETTLE_MS : 1000u;
    if (s_action == PID_TEST_ACTION_BODE && APP_PID_TEST_FREQUENCY) return APP_PID_TEST_FR_ACTIVE_MS;
    if (s_action == PID_TEST_ACTION_CREEP || s_action == PID_TEST_ACTION_STALL) return (step & 1u) ? 4000u : 1000u;
    return hold[(step - 1u) % 6u];
}

static void command(uint32_t elapsed_ms)
{
    float v = 0.0f;
    uint32_t irq;
    if (s_status.step > 0u && s_status.step < final_step()) {
        if (s_action == PID_TEST_ACTION_BODE && APP_PID_TEST_FREQUENCY) {
            const float t = (float)elapsed_ms * 0.001f;
            /* DFT-bin frequencies for a 1024-sample / 5 ms window. */
            v = APP_PID_TEST_FR_BASE_MPS + APP_PID_TEST_FR_AMPLITUDE_MPS *
                (sinf(2.0f * 3.14159265359f * 0.390625f * t) +
                 sinf(2.0f * 3.14159265359f * 0.78125f * t) +
                 sinf(2.0f * 3.14159265359f * 1.5625f * t) +
                 sinf(2.0f * 3.14159265359f * 2.34375f * t));
        } else if (s_action == PID_TEST_ACTION_CREEP || s_action == PID_TEST_ACTION_STALL) {
            const float speed = s_action == PID_TEST_ACTION_STALL ? 0.003f : s_status.speed_mps;
            if ((s_status.step % 4u) == 1u) v = speed;
            else if ((s_status.step % 4u) == 3u) v = -speed;
        } else {
            static const uint8_t order[18] = {0,1,0,2,1,2, 1,0,2,0,1,2, 2,1,0,1,2,0};
            static const float levels[3] = {APP_PID_TEST_LOW_MPS,
                APP_PID_TEST_MID_MPS, APP_PID_TEST_HIGH_MPS};
            static const float speed_levels[3] = {0.030f, 0.060f, 0.090f};
            const float *selected = s_action == PID_TEST_ACTION_SPEED ? speed_levels : levels;
            v = APP_PID_TEST_DIRECTION * selected[order[(s_status.step - 1u) % 18u]];
        }
    }
    irq = __get_PRIMASK();
    __disable_irq();
    Mecanum_SetBodySpeed(s_chassis, s_status.axis == 0u ? v : 0.0f,
                         s_status.axis == 1u ? v : 0.0f, 0.0f);
    __set_PRIMASK(irq);
}

void AppPidTest_Stop(AppPidTestReason reason)
{
    uint32_t irq;
    if (!s_chassis || s_status.state != PID_TEST_RUNNING) return;
    s_status.reason = reason;
    s_status.state = reason == PID_TEST_REASON_COMPLETE ? PID_TEST_DONE :
        reason == PID_TEST_REASON_MANUAL ? PID_TEST_CANCELLED :
        reason == PID_TEST_REASON_TIMEOUT ? PID_TEST_TIMEOUT : PID_TEST_FAULT;
    irq = __get_PRIMASK();
    __disable_irq();
    Mecanum_Stop(s_chassis);
    Motor_Stop(s_chassis->motor_lf);
    Motor_Stop(s_chassis->motor_rf);
    Motor_Stop(s_chassis->motor_lb);
    Motor_Stop(s_chassis->motor_rb);
    __set_PRIMASK(irq);
}

void AppPidTest_Init(MecanumChassis_t *chassis)
{
    s_chassis = chassis;
#if defined(APP_PID_TEST_HOST)
    /* Host fixtures retain the historical step profile so they continue to
     * exercise the original safety/state-machine contract. */
    s_action = APP_PID_TEST_CREEP ? PID_TEST_ACTION_CREEP : PID_TEST_ACTION_STEP;
#else
    s_action = APP_PID_TEST_FREQUENCY ? PID_TEST_ACTION_BODE :
        (APP_PID_TEST_CREEP ? PID_TEST_ACTION_CREEP : PID_TEST_ACTION_STEP);
#endif
    s_status = (AppPidTestStatus){PID_TEST_IDLE, PID_TEST_REASON_NONE,
        0u, APP_PID_TEST_AXIS, s_action, APP_PID_TEST_CREEP ? APP_PID_TEST_SPEED_MPS : APP_PID_TEST_HIGH_MPS, 0u};
    if (!chassis || !chassis->motor_lf || !chassis->motor_rf ||
        !chassis->motor_lb || !chassis->motor_rb ||
        !isfinite(s_status.speed_mps) || s_status.speed_mps <= 0.0f ||
        !isfinite(chassis->max_speed) || s_status.speed_mps > chassis->max_speed ||
        (APP_PID_TEST_CREEP ?
         (s_status.speed_mps != 0.006f && s_status.speed_mps != 0.012f) :
         (!isfinite(APP_PID_TEST_LOW_MPS) || !isfinite(APP_PID_TEST_MID_MPS) ||
          APP_PID_TEST_LOW_MPS <= 0.0f ||
          APP_PID_TEST_LOW_MPS >= APP_PID_TEST_MID_MPS ||
          APP_PID_TEST_MID_MPS >= APP_PID_TEST_HIGH_MPS))) {
        s_status.state = PID_TEST_FAULT;
        s_status.reason = PID_TEST_REASON_CONFIG;
    }
}

uint8_t AppPidTest_SelectAction(uint8_t action)
{
    if (action > PID_TEST_ACTION_STALL || s_status.state == PID_TEST_RUNNING) return 0u;
    s_action = action;
    s_status.action = action;
    s_status.step = 0u;
    s_status.reason = PID_TEST_REASON_NONE;
    s_status.speed_mps = (action == PID_TEST_ACTION_CREEP) ? 0.006f :
        (action == PID_TEST_ACTION_STALL ? 0.003f :
         (action == PID_TEST_ACTION_SPEED ? 0.09f : APP_PID_TEST_HIGH_MPS));
    return 1u;
}

uint8_t AppPidTest_Rearm(void)
{
    Motor_t *motors[4]; unsigned i;
    if (s_status.state != PID_TEST_DONE && s_status.state != PID_TEST_CANCELLED &&
        s_status.state != PID_TEST_TIMEOUT && s_status.state != PID_TEST_FAULT) return 0u;
    if (!s_chassis || !s_chassis->enable || s_chassis->mode != 0u ||
        s_chassis->vx_target != 0.0f || s_chassis->vy_target != 0.0f ||
        s_chassis->omega_target != 0.0f) return 0u;
    motors[0]=s_chassis->motor_lf; motors[1]=s_chassis->motor_rf;
    motors[2]=s_chassis->motor_lb; motors[3]=s_chassis->motor_rb;
    for (i=0u;i<4u;++i) if (!motors[i] || motors[i]->target_cmd != 0.0f ||
        motors[i]->target_speed != 0.0f || motors[i]->output != 0.0f ||
        !isfinite(motors[i]->actual_speed) || fabsf(motors[i]->actual_speed) >= 10.0f) return 0u;
    s_status.state=PID_TEST_IDLE; s_status.reason=PID_TEST_REASON_NONE;
    s_status.step=0u; s_status.started_ms=0u; s_entered=0u;
    s_status.action=s_action;
    return 1u;
}

uint8_t AppPidTest_Start(uint32_t now)
{
    if (s_status.state != PID_TEST_IDLE || !s_chassis || !s_chassis->enable)
        return 0u;
    s_status.started_ms = s_entered = now;
    s_status.state = PID_TEST_RUNNING;
    /* Publish the initial zero target as a normal main-loop command.  This
     * makes the first phase explicit and keeps the safety gate observable. */
    command(0u);
    return 1u;
}

void AppPidTest_Poll(uint32_t now, AppPidTestReason fault)
{
    if (s_status.state != PID_TEST_RUNNING) return;
    if (fault != PID_TEST_REASON_NONE) { AppPidTest_Stop(fault); return; }
    /* A delayed Poll never compresses/skips multiple motion phases. */
    if ((uint32_t)(now - s_status.started_ms) >=
        ((s_action == PID_TEST_ACTION_BODE && APP_PID_TEST_FREQUENCY) ?
         (APP_PID_TEST_FR_WARMUP_MS + APP_PID_TEST_FR_ACTIVE_MS + APP_PID_TEST_FR_SETTLE_MS + 5000u) :
         (APP_PID_TEST_CREEP ? 40000u : 70000u))) {
        AppPidTest_Stop(PID_TEST_REASON_TIMEOUT);
        return;
    }
    if ((uint32_t)(now - s_entered) >= duration(s_status.step)) {
        if (s_status.step == final_step()) { AppPidTest_Stop(PID_TEST_REASON_COMPLETE); return; }
        ++s_status.step;
        s_entered = now;
        command((uint32_t)(now - s_status.started_ms));
    }
    if (s_action == PID_TEST_ACTION_BODE && APP_PID_TEST_FREQUENCY && s_status.step == 1u)
        command((uint32_t)(now - s_status.started_ms - APP_PID_TEST_FR_WARMUP_MS));
}

const AppPidTestStatus *AppPidTest_Status(void) { return &s_status; }
#endif
