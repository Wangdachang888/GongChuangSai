#ifndef APP_PID_TEST_H
#define APP_PID_TEST_H

#include "app_mode.h"
#if APP_PID_TEST_ENABLE
#include "mecanum_chassis.h"

typedef enum {
    PID_TEST_IDLE, PID_TEST_RUNNING, PID_TEST_DONE,
    PID_TEST_CANCELLED, PID_TEST_TIMEOUT, PID_TEST_FAULT
} AppPidTestState;
typedef enum {
    PID_TEST_REASON_NONE, PID_TEST_REASON_COMPLETE, PID_TEST_REASON_MANUAL,
    PID_TEST_REASON_TIMEOUT, PID_TEST_REASON_IMU, PID_TEST_REASON_LOOP,
    PID_TEST_REASON_FEEDBACK, PID_TEST_REASON_CONFIG, PID_TEST_REASON_CAPTURE
} AppPidTestReason;
typedef struct {
    AppPidTestState state;
    AppPidTestReason reason;
    uint8_t step, axis;
    float speed_mps;
    uint32_t started_ms;
} AppPidTestStatus;

/* One run per init/MCU reset. Caller owns physical input and acquisition gates.
 * All calls main-loop-only. Fault is explicit, not inferred from low wheel speed. */
void AppPidTest_Init(MecanumChassis_t *chassis);
uint8_t AppPidTest_Start(uint32_t now);
void AppPidTest_Poll(uint32_t now, AppPidTestReason fault);
void AppPidTest_Stop(AppPidTestReason reason);
const AppPidTestStatus *AppPidTest_Status(void);
#endif
#endif
