#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <stdint.h>
#include "bsp_ulog.h"

const char *AppRuntime_PidAccess(const ULogPidRequest *request, ULogPidValues *values);
void AppRuntime_Init(void);
void AppRuntime_Poll(void);
uint8_t AppRuntime_ControlTick(void);
void AppRuntime_MotorTick(void);
void AppRuntime_ChassisTick(void);

#endif
