/**
 ******************************************************************************
 * @file    app_stepper_openloop.c
 * @brief   D36 dual-axis open-loop application layer (+ optional trap profile)
 ******************************************************************************
 */
#include "app_stepper_openloop.h"

#if (APP_STEPPER_USE_TRAP != 0)
#include "app_stepper_drive.h"
#endif

#if (APP_STEPPER_ENABLE != 0)

typedef struct {
    uint8_t  continuous;
    int8_t   direction;
    uint32_t frequency_hz;
} AppStepperAxisCtrl_t;

static AppStepperAxisCtrl_t s_ctrl[2];

static GPIO_PinState AppStepper_DirLevelFromSign(int8_t direction)
{
    if (direction < 0) {
        return GPIO_PIN_SET;
    }
    return GPIO_PIN_RESET;
}

static HAL_StatusTypeDef AppStepper_ApplyDirection(StepperAxis_t axis,
                                                   int8_t direction)
{
    if (direction == 0) {
        return HAL_ERROR;
    }
    if (Stepper_IsBusy(axis) != 0U) {
        Stepper_Stop(axis);
    }
    return Stepper_SetDirection(axis, AppStepper_DirLevelFromSign(direction));
}

static uint32_t pulses_per_rev(StepperAxis_t axis)
{
    return (axis == STEPPER_AXIS_1) ? APP_STEPPER_LIFT_PULSES_PER_REV :
                                      APP_STEPPER_PULSES_PER_REV;
}

static float freq_to_rpm(StepperAxis_t axis, uint32_t frequency_hz)
{
    float rpm;
    if (frequency_hz < 1U) {
        frequency_hz = 1U;
    }
    rpm = ((float)frequency_hz * 60.0f) / (float)pulses_per_rev(axis);
    if (rpm < 1.0f) {
        rpm = 1.0f;
    }
    return rpm;
}

void AppStepper_Init(void)
{
    s_ctrl[STEPPER_AXIS_1].continuous = 0U;
    s_ctrl[STEPPER_AXIS_1].direction = 1;
    s_ctrl[STEPPER_AXIS_1].frequency_hz = APP_STEPPER_DEFAULT_FREQ_HZ;
    s_ctrl[STEPPER_AXIS_2].continuous = 0U;
    s_ctrl[STEPPER_AXIS_2].direction = 1;
    s_ctrl[STEPPER_AXIS_2].frequency_hz = APP_STEPPER_DEFAULT_FREQ_HZ;

#if (APP_STEPPER_USE_TRAP != 0)
    StepperDrive_Init();
#else
    Stepper_Init();
#endif

#if (APP_STEPPER_BOOT_TEST != 0)
    (void)AppStepper_MoveSteps(STEPPER_AXIS_1,
                               (int32_t)APP_STEPPER_BOOT_STEPS,
                              APP_STEPPER_DEFAULT_FREQ_HZ);
#endif
}

void AppStepper_Poll(void)
{
    int i;
    uint32_t remain;
    StepperAxis_t axis;

#if (APP_STEPPER_USE_TRAP != 0)
    StepperDrive_Poll();
#endif

    for (i = 0; i < 2; i++) {
        axis = (StepperAxis_t)i;
        if (s_ctrl[axis].continuous == 0U) {
            continue;
        }
        remain = Stepper_GetRemainingSteps(axis);
        if (remain < APP_STEPPER_FEED_THRESHOLD) {
            (void)Stepper_Feed(axis,
                               pulses_per_rev(axis),
                               s_ctrl[axis].frequency_hz);
        }
    }
}

HAL_StatusTypeDef AppStepper_MoveSteps(StepperAxis_t axis,
                                       int32_t steps,
                                       uint32_t frequency_hz)
{
    return AppStepper_MoveStepsProfile(axis, steps, frequency_hz,
                                       APP_STEPPER_TRAP_ACCEL_RPMS);
}

HAL_StatusTypeDef AppStepper_MoveStepsProfile(StepperAxis_t axis,
                                              int32_t steps,
                                              uint32_t frequency_hz,
                                              float accel_rpms)
{
    uint32_t abs_steps;
    int8_t direction;

    if (axis != STEPPER_AXIS_1) {
        return HAL_ERROR;
    }

    s_ctrl[axis].continuous = 0U;

    if (steps == 0) {
        AppStepper_Stop(axis);
        return HAL_OK;
    }

    if (steps > 0) {
        direction = 1;
        abs_steps = (uint32_t)steps;
    } else {
        direction = -1;
        abs_steps = (uint32_t)(-steps);
    }

    s_ctrl[axis].direction = direction;
    s_ctrl[axis].frequency_hz = frequency_hz;

#if (APP_STEPPER_USE_TRAP != 0)
    StepperDrive_MoveSteps((uint8_t)axis, direction, (int32_t)abs_steps,
                           freq_to_rpm(axis, frequency_hz), accel_rpms);
    return HAL_OK;
#else
    {
        HAL_StatusTypeDef status = AppStepper_ApplyDirection(axis, direction);
        if (status != HAL_OK) {
            return status;
        }
        return Stepper_Start(axis, abs_steps, frequency_hz);
    }
#endif
}

HAL_StatusTypeDef AppStepper_StartContinuous(StepperAxis_t axis,
                                             int8_t direction,
                                             uint32_t frequency_hz)
{
    HAL_StatusTypeDef status;

    if (axis != STEPPER_AXIS_1) {
        return HAL_ERROR;
    }

#if (APP_STEPPER_USE_TRAP != 0)
    StepperDrive_Stop((uint8_t)axis);
#endif

    status = AppStepper_ApplyDirection(axis, direction);
    if (status != HAL_OK) {
        return status;
    }

    s_ctrl[axis].continuous = 1U;
    s_ctrl[axis].direction = direction;
    s_ctrl[axis].frequency_hz = frequency_hz;
    return Stepper_Start(axis, pulses_per_rev(axis), frequency_hz);
}

void AppStepper_Stop(StepperAxis_t axis)
{
    s_ctrl[axis].continuous = 0U;
#if (APP_STEPPER_USE_TRAP != 0)
    StepperDrive_Stop((uint8_t)axis);
#else
    Stepper_Stop(axis);
#endif
}

void AppStepper_StopAll(void)
{
    s_ctrl[STEPPER_AXIS_1].continuous = 0U;
    s_ctrl[STEPPER_AXIS_2].continuous = 0U;
#if (APP_STEPPER_USE_TRAP != 0)
    StepperDrive_StopAll();
#else
    Stepper_StopAll();
#endif
}

void AppStepper_SetEnable(StepperAxis_t axis, uint8_t enable)
{
    if (axis != STEPPER_AXIS_1) return;
    Stepper_SetEnable(axis, enable);
}

void AppStepper_SetEnableAll(uint8_t enable)
{
    /* Axis 2 is reserved for PC7/SERVO_3 and must remain disabled. */
    AppStepper_SetEnable(STEPPER_AXIS_1, enable);
    AppStepper_SetEnable(STEPPER_AXIS_2, 0U);
}

void AppStepper_SetAccel(StepperAxis_t axis, float accel_rpms)
{
    if (axis != STEPPER_AXIS_1) return;
#if (APP_STEPPER_USE_TRAP != 0)
    StepperDrive_SetAccel((uint8_t)axis, accel_rpms);
#else
    (void)axis;
    (void)accel_rpms;
#endif
}

void AppStepper_SetBrakeMode(StepperAxis_t axis, uint8_t trap_brake)
{
    if (axis != STEPPER_AXIS_1) return;
#if (APP_STEPPER_USE_TRAP != 0)
    StepperDrive_SetBrakeMode((uint8_t)axis, trap_brake);
#else
    (void)axis;
    (void)trap_brake;
#endif
}

uint8_t AppStepper_IsBusy(StepperAxis_t axis)
{
    if (axis != STEPPER_AXIS_1) return 0U;
#if (APP_STEPPER_USE_TRAP != 0)
    if (s_ctrl[axis].continuous != 0U) {
        return Stepper_IsBusy(axis);
    }
    return StepperDrive_IsBusy((uint8_t)axis);
#else
    return Stepper_IsBusy(axis);
#endif
}

uint32_t AppStepper_GetRemainingSteps(StepperAxis_t axis)
{
    if (axis != STEPPER_AXIS_1) return 0U;
#if (APP_STEPPER_USE_TRAP != 0)
    if (s_ctrl[axis].continuous != 0U) {
        return Stepper_GetRemainingSteps(axis);
    }
    return StepperDrive_GetStepsLeft((uint8_t)axis);
#else
    return Stepper_GetRemainingSteps(axis);
#endif
}

#else /* APP_STEPPER_ENABLE == 0 */

void AppStepper_Init(void) {}
void AppStepper_Poll(void) {}
HAL_StatusTypeDef AppStepper_MoveSteps(StepperAxis_t axis, int32_t steps, uint32_t frequency_hz)
{ (void)axis;(void)steps;(void)frequency_hz; return HAL_ERROR; }
HAL_StatusTypeDef AppStepper_MoveStepsProfile(StepperAxis_t axis, int32_t steps,
                                              uint32_t frequency_hz, float accel_rpms)
{ (void)axis;(void)steps;(void)frequency_hz;(void)accel_rpms; return HAL_ERROR; }
HAL_StatusTypeDef AppStepper_StartContinuous(StepperAxis_t axis, int8_t direction, uint32_t frequency_hz)
{ (void)axis;(void)direction;(void)frequency_hz; return HAL_ERROR; }
void AppStepper_Stop(StepperAxis_t axis) { (void)axis; }
void AppStepper_StopAll(void) {}
void AppStepper_SetEnable(StepperAxis_t axis, uint8_t enable) { (void)axis;(void)enable; }
void AppStepper_SetEnableAll(uint8_t enable) { (void)enable; }
void AppStepper_SetAccel(StepperAxis_t axis, float accel_rpms) { (void)axis; (void)accel_rpms; }
void AppStepper_SetBrakeMode(StepperAxis_t axis, uint8_t trap_brake) { (void)axis; (void)trap_brake; }
uint8_t AppStepper_IsBusy(StepperAxis_t axis) { (void)axis; return 0U; }
uint32_t AppStepper_GetRemainingSteps(StepperAxis_t axis) { (void)axis; return 0U; }

#endif /* APP_STEPPER_ENABLE */
