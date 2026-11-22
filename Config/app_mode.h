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
/* Creep-only speed; normal profile uses LOW/MID/HIGH below. */
#ifndef APP_PID_TEST_SPEED_MPS
#define APP_PID_TEST_SPEED_MPS 0.03f
#endif
/* Normal identification: one direction per run, three strictly ordered levels. */
#ifndef APP_PID_TEST_DIRECTION
#define APP_PID_TEST_DIRECTION 1
#endif
#ifndef APP_PID_TEST_LOW_MPS
#define APP_PID_TEST_LOW_MPS 0.015f
#endif
#ifndef APP_PID_TEST_MID_MPS
#define APP_PID_TEST_MID_MPS 0.030f
#endif
#ifndef APP_PID_TEST_HIGH_MPS
#define APP_PID_TEST_HIGH_MPS 0.045f
#endif
#if APP_PID_TEST_DIRECTION != 1 && APP_PID_TEST_DIRECTION != -1
#error Invalid PID test direction
#endif
/* Separate, unvalidated creep profile: explicitly select 0.006/0.012 m/s. */
#ifndef APP_PID_TEST_CREEP
#define APP_PID_TEST_CREEP 0
#endif
#ifndef APP_PID_TEST_FREQUENCY
#define APP_PID_TEST_FREQUENCY 1
#endif
#if APP_PID_TEST_CREEP != 0 && APP_PID_TEST_FREQUENCY != 0
#error PID test profiles are mutually exclusive
#endif
#ifndef APP_PID_TEST_FR_BASE_MPS
#define APP_PID_TEST_FR_BASE_MPS 0.030f
#endif
#ifndef APP_PID_TEST_FR_AMPLITUDE_MPS
#define APP_PID_TEST_FR_AMPLITUDE_MPS 0.0020f
#endif
#ifndef APP_PID_TEST_FR_WARMUP_MS
#define APP_PID_TEST_FR_WARMUP_MS 4000u
#endif
#ifndef APP_PID_TEST_FR_ACTIVE_MS
#define APP_PID_TEST_FR_ACTIVE_MS 40960u
#endif
#ifndef APP_PID_TEST_FR_SETTLE_MS
#define APP_PID_TEST_FR_SETTLE_MS 2000u
#endif
#if APP_PID_TEST_AXIS != 0 && APP_PID_TEST_AXIS != 1
#error Invalid PID test axis
#endif
#if APP_PID_TEST_CREEP != 0 && APP_PID_TEST_CREEP != 1
#error Invalid PID test profile
#endif
#if APP_PID_TEST_FREQUENCY != 0 && APP_PID_TEST_FREQUENCY != 1
#error Invalid PID frequency profile
#endif
#endif

#endif
