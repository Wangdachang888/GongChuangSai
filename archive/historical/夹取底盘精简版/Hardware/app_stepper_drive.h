/**
 * @file    app_stepper_drive.h
 * @brief   D36A stepper motion planner.
 *
 * Normal moves use a jerk-limited S-curve. A move marked with
 * SetBrakeMode(..., 1) uses the same S-curve for acceleration, then
 * switches directly to trapezoidal braking.
 */
#ifndef __APP_STEPPER_DRIVE_H
#define __APP_STEPPER_DRIVE_H

#include <stdint.h>

#ifndef DRIVE_PPR
#define DRIVE_PPR              800U
#endif
#ifndef DRIVE_TICK_MS
#define DRIVE_TICK_MS          5U
#endif
#ifndef DRIVE_RPM_MIN
#define DRIVE_RPM_MIN          1.0f
#endif
#ifndef DRIVE_RPM_MAX
#define DRIVE_RPM_MAX          750.0f
#endif
#ifndef DRIVE_DEFAULT_ACCEL
#define DRIVE_DEFAULT_ACCEL    80.0f
#endif
#ifndef DRIVE_TERMINAL_FREQ_HZ
#define DRIVE_TERMINAL_FREQ_HZ 300U
#endif
#ifndef DRIVE_TURNTABLE_TERMINAL_FREQ_HZ
#define DRIVE_TURNTABLE_TERMINAL_FREQ_HZ 60U
#endif
#ifndef DRIVE_DIRECTION_SETTLE_MS
#define DRIVE_DIRECTION_SETTLE_MS 3U
#endif
#ifndef DRIVE_DEMO_RPM
#define DRIVE_DEMO_RPM         30.0f
#endif
#ifndef DRIVE_DEMO_ACCEL
#define DRIVE_DEMO_ACCEL       80.0f
#endif
#ifndef DRIVE_DEMO_REVS
#define DRIVE_DEMO_REVS        1.0f
#endif
#ifndef DRIVE_DEMO_PAUSE_MS
#define DRIVE_DEMO_PAUSE_MS    500U
#endif
#ifndef DRIVE_DEMO_SPIN
#define DRIVE_DEMO_SPIN        0
#endif

void     StepperDrive_Init(void);
void     StepperDrive_Poll(void);
void     StepperDrive_Spin(uint8_t axis, int8_t dir, float rpm, float accel);
void     StepperDrive_MoveRevs(uint8_t axis, int8_t dir, float revs, float rpm, float accel);
void     StepperDrive_MoveSteps(uint8_t axis, int8_t dir, int32_t steps, float rpm, float accel);
void     StepperDrive_SetAccel(uint8_t axis, float accel);
void     StepperDrive_SetBrakeMode(uint8_t axis, uint8_t trap_brake);
void     StepperDrive_Stop(uint8_t axis);
void     StepperDrive_StopAll(void);

uint8_t  StepperDrive_IsBusy(uint8_t axis);
float    StepperDrive_GetRPM(uint8_t axis);
uint32_t StepperDrive_GetStepsLeft(uint8_t axis);
void     StepperDrive_DemoStart(void);

#endif /* __APP_STEPPER_DRIVE_H */
