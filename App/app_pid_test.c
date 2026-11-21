#include "app_pid_test.h"
#if APP_PID_TEST_ENABLE
#include <math.h>

static MecanumChassis_t *s_chassis;
static AppPidTestStatus s_status;
static uint32_t s_entered;

/* Normal: initial zero, (A,B,C)x3 with six levels each, final zero.
 * Creep retains its separate (+,0,-,0)x3 diagnostic. No PID resets. */
static uint8_t final_step(void) { return APP_PID_TEST_CREEP ? 13u : 55u; }
static uint32_t duration(uint8_t step)
{
    static const uint16_t hold[6] = {500u,1000u,1500u,500u,1000u,1500u};
    if (step == 0u || step == final_step()) return 1000u;
    if (APP_PID_TEST_CREEP) return (step & 1u) ? 4000u : 1000u;
    return hold[(step - 1u) % 6u];
}

static void command(void)
{
    float v = 0.0f;
    uint32_t irq;
    if (s_status.step > 0u && s_status.step < final_step()) {
        if (APP_PID_TEST_CREEP) {
            if ((s_status.step % 4u) == 1u) v = s_status.speed_mps;
            else if ((s_status.step % 4u) == 3u) v = -s_status.speed_mps;
        } else {
            static const uint8_t order[18] = {0,1,0,2,1,2, 1,0,2,0,1,2, 2,1,0,1,2,0};
            static const float levels[3] = {APP_PID_TEST_LOW_MPS,
                APP_PID_TEST_MID_MPS, APP_PID_TEST_HIGH_MPS};
            v = APP_PID_TEST_DIRECTION * levels[order[(s_status.step - 1u) % 18u]];
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
    s_status = (AppPidTestStatus){PID_TEST_IDLE, PID_TEST_REASON_NONE,
        0u, APP_PID_TEST_AXIS, APP_PID_TEST_CREEP ? APP_PID_TEST_SPEED_MPS : APP_PID_TEST_HIGH_MPS, 0u};
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

uint8_t AppPidTest_Start(uint32_t now)
{
    if (s_status.state != PID_TEST_IDLE || !s_chassis || !s_chassis->enable)
        return 0u;
    s_status.started_ms = s_entered = now;
    s_status.state = PID_TEST_RUNNING;
    command();
    return 1u;
}

void AppPidTest_Poll(uint32_t now, AppPidTestReason fault)
{
    if (s_status.state != PID_TEST_RUNNING) return;
    if (fault != PID_TEST_REASON_NONE) { AppPidTest_Stop(fault); return; }
    /* A delayed Poll never compresses/skips multiple motion phases. */
    if ((uint32_t)(now - s_status.started_ms) >=
        (APP_PID_TEST_CREEP ? 40000u : 70000u)) {
        AppPidTest_Stop(PID_TEST_REASON_TIMEOUT);
        return;
    }
    if ((uint32_t)(now - s_entered) >= duration(s_status.step)) {
        if (s_status.step == final_step()) { AppPidTest_Stop(PID_TEST_REASON_COMPLETE); return; }
        ++s_status.step;
        s_entered = now;
        command();
    }
}

const AppPidTestStatus *AppPidTest_Status(void) { return &s_status; }
#endif
