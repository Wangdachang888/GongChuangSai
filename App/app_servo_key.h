/**
 * @file    app_servo_key.h
 * @brief   PE0 key jog for dual servos
 *          short press  = +SERVO_KEY_STEP_DEG (default 30)
 *          long  press  = -SERVO_KEY_STEP_DEG
 */
#ifndef APP_SERVO_KEY_H
#define APP_SERVO_KEY_H

#include <stdint.h>

#ifndef APP_SERVO_KEY_MODE
#define APP_SERVO_KEY_MODE           1
#endif

#ifndef SERVO_KEY_STEP_DEG
#define SERVO_KEY_STEP_DEG           30.0f
#endif

#ifndef SERVO_KEY_LONG_PRESS_MS
#define SERVO_KEY_LONG_PRESS_MS      700U
#endif

#ifndef SERVO_KEY_DEBOUNCE_MS
#define SERVO_KEY_DEBOUNCE_MS        20U
#endif

#ifndef SERVO_KEY_USE_OLED
#define SERVO_KEY_USE_OLED           1
#endif

#ifndef SERVO_KEY_OLED_MS
#define SERVO_KEY_OLED_MS            200U
#endif

void ServoKey_Init(void);
void ServoKey_Poll(void);

#endif /* APP_SERVO_KEY_H */
