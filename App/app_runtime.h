#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <stdint.h>

void AppRuntime_Init(void);
void AppRuntime_Poll(void);
uint8_t AppRuntime_ControlTick(void);
void AppRuntime_MotorTick(void);
void AppRuntime_ChassisTick(void);

#endif
