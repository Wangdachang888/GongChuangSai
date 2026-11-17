/**
 * @file    bsp_servo.h
 * @brief   Dual 359° servo on TIM12 (PB14/PB15) with smooth accel profile
 */
#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_COUNT              2U
#define SERVO_1                  0U   /* PB14 TIM12_CH1 */
#define SERVO_2                  1U   /* PB15 TIM12_CH2 */

#ifndef SERVO_ANGLE_MIN_DEG
#define SERVO_ANGLE_MIN_DEG      0.0f
#endif
#ifndef SERVO_ANGLE_MAX_DEG
#define SERVO_ANGLE_MAX_DEG      359.0f
#endif

#ifndef SERVO_PULSE_MIN_US
#define SERVO_PULSE_MIN_US       500U
#endif
#ifndef SERVO_PULSE_MAX_US
#define SERVO_PULSE_MAX_US       2500U
#endif
#ifndef SERVO_PULSE_CENTER_US
#define SERVO_PULSE_CENTER_US    1500U
#endif

#ifndef SERVO_BOOT_ANGLE_DEG
#define SERVO_BOOT_ANGLE_DEG     0.0f
#endif

/* Motion profile: smaller = gentler. 30° move ~1.0–1.5 s */
#ifndef SERVO_MAX_SPEED_DPS
#define SERVO_MAX_SPEED_DPS      35.0f    /* deg/s cruise */
#endif
#ifndef SERVO_ACCEL_DPS2
#define SERVO_ACCEL_DPS2         70.0f    /* deg/s^2 accel & brake */
#endif
#ifndef SERVO_STOP_EPS_DEG
#define SERVO_STOP_EPS_DEG       0.2f
#endif
#ifndef SERVO_VEL_EPS_DPS
#define SERVO_VEL_EPS_DPS        0.5f
#endif

void Servo_Init(void);

/* Soft target (uses accel profile). Call Servo_Poll() in main loop. */
void Servo_SetAngle(uint8_t id, float angle_deg);
void Servo_SetBothAngle(float angle1_deg, float angle2_deg);
void Servo_Nudge(uint8_t id, float delta_deg);
void Servo_NudgeAll(float delta_deg);

/* Instant jump (init / emergency). */
void Servo_SetAngleImmediate(uint8_t id, float angle_deg);
void Servo_SetPulseUs(uint8_t id, uint16_t pulse_us);

void Servo_Center(uint8_t id);
void Servo_CenterAll(void);
void Servo_Disable(uint8_t id);
void Servo_DisableAll(void);
void Servo_EnableAll(void);

/* Must be polled often (e.g. every main-loop). */
void Servo_Poll(void);

float Servo_GetAngle(uint8_t id);       /* current commanded angle */
float Servo_GetTarget(uint8_t id);
uint16_t Servo_GetPulseUs(uint8_t id);
uint8_t Servo_IsBusy(uint8_t id);
uint8_t Servo_IsBusyAny(void);

/* Runtime tune */
void Servo_SetSpeedLimit(float max_speed_dps);
void Servo_SetAccel(float accel_dps2);
void Servo_SetProfile(uint8_t id, float max_speed_dps, float accel_dps2);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SERVO_H */
