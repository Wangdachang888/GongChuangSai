#ifndef RACE_TEST_BOARD_H
#define RACE_TEST_BOARD_H
#include "usart.h"
#include <stdbool.h>
#define __MECANUM_CHASSIS_H
#define APP_IMU_H
#define __BSP_OLED_H
#define GPIOE ((void *)5)
#define GPIO_PIN_0 1U
#define GPIO_PIN_RESET 0U
#define OLED_CMD 0U
#define OLED_DATA 1U
typedef struct { float output, target_speed, actual_speed; } Motor_t;
typedef struct {
    Motor_t *motor_lf, *motor_rf, *motor_lb, *motor_rb;
    float x_pos, y_pos, theta_pos, vx_actual, vy_actual, omega_actual;
    float x_target, y_target, theta_target, max_speed, max_omega, position_tol, theta_tol;
    float vx_target, vy_target, omega_target;
    uint8_t body_frame, mode, target_reached;
} MecanumChassis_t;
extern uint8_t OLED_GRAM[128][8];
uint32_t HAL_GetTick(void);
int HAL_GPIO_ReadPin(void *port, uint16_t pin);
void Mecanum_Stop(MecanumChassis_t *chassis);
void Motor_Stop(Motor_t *motor);
void Mecanum_SetSpeedLimit(MecanumChassis_t *chassis, float linear, float angular);
void Mecanum_SetTolerance(MecanumChassis_t *chassis, float position, float yaw);
void Mecanum_SetPositionMode(MecanumChassis_t *chassis, float x, float y, float yaw);
float app_imu_yaw_deg(void);
bool app_imu_ok(void);
bool app_imu_bias_ready(void);
uint32_t app_imu_frames(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
void OLED_WR_Byte(uint8_t value, uint8_t command);
#endif
