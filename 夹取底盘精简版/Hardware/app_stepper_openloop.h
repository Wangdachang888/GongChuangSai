/**
 ******************************************************************************
 * @file    app_stepper_openloop.h
 * @brief   D36 open-loop lift stepper (TIM8 STEP + GPIO DIR)
 ******************************************************************************
 * PC7 is reserved for SERVO_3; only axis 1 is exposed by this application.
 * APP_STEPPER_USE_TRAP=1: MoveSteps uses a pulse-domain S-curve planner;
 * final reset moves can select direct trapezoidal braking at runtime.
 * Requires AppStepper_Poll() in the main loop.
 ******************************************************************************
 */
#ifndef __APP_STEPPER_OPENLOOP_H
#define __APP_STEPPER_OPENLOOP_H

#include "bsp_stepper.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef APP_STEPPER_ENABLE
#define APP_STEPPER_ENABLE              1
#endif

#ifndef APP_STEPPER_BOOT_TEST
#define APP_STEPPER_BOOT_TEST           0
#endif

#ifndef APP_STEPPER_FEED_THRESHOLD
#define APP_STEPPER_FEED_THRESHOLD      400U
#endif

#ifndef D36_MICROSTEP
#define D36_MICROSTEP                   4U
#endif

#define APP_STEPPER_PULSES_PER_REV      (200U * D36_MICROSTEP)

#ifndef APP_STEPPER_LIFT_MICROSTEP
#define APP_STEPPER_LIFT_MICROSTEP      4U
#endif

#define APP_STEPPER_LIFT_PULSES_PER_REV (200U * APP_STEPPER_LIFT_MICROSTEP)

#ifndef APP_STEPPER_DEFAULT_FREQ_HZ
#define APP_STEPPER_DEFAULT_FREQ_HZ     800U
#endif

#ifndef APP_STEPPER_BOOT_STEPS
#define APP_STEPPER_BOOT_STEPS          (APP_STEPPER_PULSES_PER_REV / 4U)
#endif

#ifndef APP_STEPPER_USE_TRAP
#define APP_STEPPER_USE_TRAP            1
#endif

#ifndef APP_STEPPER_TRAP_ACCEL_RPMS
#define APP_STEPPER_TRAP_ACCEL_RPMS     80.0f
#endif

void AppStepper_Init(void);
void AppStepper_Poll(void);

HAL_StatusTypeDef AppStepper_MoveSteps(StepperAxis_t axis,
                                       int32_t steps,
                                       uint32_t frequency_hz);
HAL_StatusTypeDef AppStepper_MoveStepsProfile(StepperAxis_t axis,
                                               int32_t steps,
                                               uint32_t frequency_hz,
                                               float accel_rpms);
HAL_StatusTypeDef AppStepper_StartContinuous(StepperAxis_t axis,
                                             int8_t direction,
                                             uint32_t frequency_hz);

void AppStepper_Stop(StepperAxis_t axis);
void AppStepper_StopAll(void);
void AppStepper_SetEnable(StepperAxis_t axis, uint8_t enable);
void AppStepper_SetEnableAll(uint8_t enable);
void AppStepper_SetAccel(StepperAxis_t axis, float accel_rpms);
void AppStepper_SetBrakeMode(StepperAxis_t axis, uint8_t trap_brake);
uint8_t AppStepper_IsBusy(StepperAxis_t axis);
uint32_t AppStepper_GetRemainingSteps(StepperAxis_t axis);

#ifdef __cplusplus
}
#endif

#endif /* __APP_STEPPER_OPENLOOP_H */
