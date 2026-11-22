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
typedef struct { float output, target_speed, actual_speed, target_cmd;
    float kp,ki,kd,integral_max,output_max,integral,err,err_last,output_last,kick;
    int acc_cnt; unsigned short acc_ms; float speed_lpf;
    unsigned short speed_window_ms; float ff_gain,deadzone,ff_min_speed;
} Motor_t;
typedef struct {float kp,ki,kd,integral_max,output_max,integral,err,err_last,output;} PosPID_t;
typedef struct {float period_ms,ff_gain,deadzone,ff_min_speed,ramp,slew,lpf,kick_step,kick_max,stuck_ratio;} MotorControlInfo;
const MotorControlInfo *Motor_GetControlInfo(void);
extern volatile uint8_t g_tune_mode;
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
typedef struct {
    Motor_t *motor_lf, *motor_rf, *motor_lb, *motor_rb;
    float x_pos, y_pos, theta_pos, vx_actual, vy_actual, omega_actual;
    float x_target, y_target, theta_target, max_speed, max_omega, position_tol, theta_tol;
    float vx_target, vy_target, omega_target;
    uint8_t body_frame, mode, target_reached, enable;
    PosPID_t pid_x,pid_y,pid_theta;
} MecanumChassis_t;
extern uint8_t OLED_GRAM[128][8];
uint32_t HAL_GetTick(void);
int HAL_GPIO_ReadPin(void *port, uint16_t pin);
void Mecanum_Stop(MecanumChassis_t *chassis);
void Mecanum_SetBodySpeed(MecanumChassis_t *chassis, float vx, float vy, float omega);
void Motor_Stop(Motor_t *motor);
void Mecanum_SetSpeedLimit(MecanumChassis_t *chassis, float linear, float angular);
void Mecanum_SetTolerance(MecanumChassis_t *chassis, float position, float yaw);
void Mecanum_SetPositionMode(MecanumChassis_t *chassis, float x, float y, float yaw);
void Mecanum_GetPosition(MecanumChassis_t *chassis, float *x, float *y, float *theta);
void Mecanum_GetVelocity(MecanumChassis_t *chassis, float *vx, float *vy, float *omega);
uint8_t Mecanum_IsTargetReached(MecanumChassis_t *chassis);
float app_imu_yaw_deg(void);
bool app_imu_ok(void);
bool app_imu_bias_ready(void);
uint32_t app_imu_frames(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
void OLED_WR_Byte(uint8_t value, uint8_t command);
#endif
