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
#include "stepper_config.h"

#ifdef __cplusplus
extern "C" {
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
