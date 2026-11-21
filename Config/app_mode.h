#ifndef APP_MODE_H
#define APP_MODE_H

#define APP_MODE_LEGACY       0
#define APP_MODE_SERVO        1
#define APP_MODE_STEPPER      2
#define APP_MODE_RACE         3
#define APP_MODE_PID_TEST     4

#ifndef APP_PID_TEST_ENABLE
#define APP_PID_TEST_ENABLE 0
#endif

#ifndef APP_RUN_MODE
#define APP_RUN_MODE APP_MODE_RACE
#endif

#if APP_RUN_MODE < APP_MODE_LEGACY || APP_RUN_MODE > APP_MODE_PID_TEST
#error Invalid APP_RUN_MODE
#endif

#if (APP_PID_TEST_ENABLE != 0 && APP_PID_TEST_ENABLE != 1) || \
    ((APP_PID_TEST_ENABLE != 0) != (APP_RUN_MODE == APP_MODE_PID_TEST))
#error PID test requires APP_PID_TEST_ENABLE=1 and APP_RUN_MODE=APP_MODE_PID_TEST exclusively
#endif
#if APP_PID_TEST_ENABLE
#if defined(MOTOR_TUNE) && MOTOR_TUNE != 0
#error PID test and MOTOR_TUNE are mutually exclusive
#endif
/* Axis: 0=forward/backward, 1=left/right; m/s, body frame. */
#ifndef APP_PID_TEST_AXIS
#define APP_PID_TEST_AXIS 0
#endif
#ifndef APP_PID_TEST_SPEED_MPS
#define APP_PID_TEST_SPEED_MPS 0.03f
#endif
/* Separate, unvalidated creep profile: explicitly select 0.006/0.012 m/s. */
#ifndef APP_PID_TEST_CREEP
#define APP_PID_TEST_CREEP 0
#endif
#if APP_PID_TEST_AXIS != 0 && APP_PID_TEST_AXIS != 1
#error Invalid PID test axis
#endif
#if APP_PID_TEST_CREEP != 0 && APP_PID_TEST_CREEP != 1
#error Invalid PID test profile
#endif
#endif

#endif
