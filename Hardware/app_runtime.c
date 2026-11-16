#include "app_runtime.h"
#include "app_mode.h"
#include "app_imu.h"
#include "app_race.h"
#include "app_servo_key.h"
#include "app_square.h"
#include "app_stepper_drive.h"
#include "app_stepper_openloop.h"
#include "bsp_d36_cmd.h"
#include "bsp_oled.h"
#include "bsp_servo.h"
#include "bsp_ulog.h"
#include "bsp_vbat.h"
#include "display_nav.h"
#include "mecanum_chassis.h"
#include "Motor.h"
#include "tim.h"
#include "vofa_debug.h"

Motor_t motor_left_front;
Motor_t motor_right_front;
Motor_t motor_left_back;
Motor_t motor_right_back;
MecanumChassis_t chassis;

static void stop_chassis_outputs(void)
{
    Mecanum_Stop(&chassis);
    Motor_Stop(chassis.motor_lf);
    Motor_Stop(chassis.motor_rf);
    Motor_Stop(chassis.motor_lb);
    Motor_Stop(chassis.motor_rb);
}

static void init_motors_and_chassis(void)
{
    Motor_Init(&motor_left_front,
               &htim10, TIM_CHANNEL_1, 4199,
               &htim11, TIM_CHANNEL_1, 4199,
               &htim2, 2.5f, 1.60f, 0.0f, 1, 1);
    Motor_Init(&motor_right_front,
               &htim9, TIM_CHANNEL_1, 4199,
               &htim9, TIM_CHANNEL_2, 4199,
               &htim3, 2.5f, 1.60f, 0.0f, 1, -1);
    Motor_Init(&motor_left_back,
               &htim1, TIM_CHANNEL_1, 8399,
               &htim1, TIM_CHANNEL_2, 8399,
               &htim4, 2.5f, 1.60f, 0.0f, -1, 1);
    Motor_Init(&motor_right_back,
               &htim1, TIM_CHANNEL_3, 8399,
               &htim1, TIM_CHANNEL_4, 8399,
               &htim5, 2.5f, 1.60f, 0.0f, -1, -1);

    MecanumChassis_Init(&chassis,
                        &motor_left_front, &motor_right_front,
                        &motor_left_back, &motor_right_back,
                        0.04f, 0.093f, 0.11f, 1.0f, 440.0f);

    chassis.pid_x.kp = 1.6f;
    chassis.pid_x.ki = 0.05f;
    chassis.pid_x.kd = 0.12f;
    chassis.pid_x.integral_max = 0.20f;
    chassis.pid_x.output_max = 1.0f;
    chassis.pid_y.kp = 1.6f;
    chassis.pid_y.ki = 0.05f;
    chassis.pid_y.kd = 0.12f;
    chassis.pid_y.integral_max = 0.20f;
    chassis.pid_y.output_max = 1.0f;
    chassis.pid_theta.kp = 3.2f;
    chassis.pid_theta.ki = 0.08f;
    chassis.pid_theta.kd = 0.18f;
    chassis.pid_theta.integral_max = 0.50f;
    chassis.pid_theta.output_max = PI;
}

void AppRuntime_Init(void)
{
#if APP_RUN_MODE == APP_MODE_SERVO
    __HAL_TIM_MOE_DISABLE(&htim1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    (void)HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
    Servo_Init();
    ServoKey_Init();
#else
#if APP_RUN_MODE != APP_MODE_RACE
    AppStepper_Init();
    Servo_Init();
    Servo_SetBothAngle(30.0f, 30.0f);
    OLED_Init();
    D36Cmd_Init();
#else
    OLED_Init();
#endif
    __HAL_TIM_MOE_ENABLE(&htim1);
    init_motors_and_chassis();
#if APP_RUN_MODE != APP_MODE_RACE
    DisplayNav_Init();
    VOFA_Init();
#endif
    Vbat_Init();
    if (app_imu_init() != 0) {
#if APP_RUN_MODE != APP_MODE_RACE
        Error_Handler();
#endif
    }
#if APP_RUN_MODE == APP_MODE_RACE
    ULog_Init();
    AppRace_Init(&chassis);
#endif
    stop_chassis_outputs();
#if APP_RUN_MODE != APP_MODE_RACE
#if (MOTOR_TUNE != 0)
    Tune_Start(&motor_left_front);
#else
    Square_Start(&chassis);
#endif
#endif
#endif
}

void AppRuntime_Poll(void)
{
#if APP_RUN_MODE == APP_MODE_SERVO
    ServoKey_Poll();
#elif APP_RUN_MODE == APP_MODE_RACE
    app_imu_poll();
    AppRace_Poll();
    Vbat_Poll();
    ULog_Poll(&chassis);
#else
#if (MOTOR_TUNE != 0)
    Tune_Poll();
#else
    Square_Poll(&chassis);
#endif
    app_imu_poll();
    AppStepper_Poll();
    D36Cmd_Poll();
    Vbat_Poll();
    DisplayNav_Poll(&chassis);
    ULog_Poll(&chassis);
#endif
}

uint8_t AppRuntime_ControlTick(void)
{
#if APP_RUN_MODE == APP_MODE_RACE
    return AppRace_ControlTick();
#else
    return 1U;
#endif
}

void AppRuntime_MotorTick(void)
{
    Motor_Update(&motor_left_front, 0.001f);
    Motor_Update(&motor_right_front, 0.001f);
    Motor_Update(&motor_left_back, 0.001f);
    Motor_Update(&motor_right_back, 0.001f);
}

void AppRuntime_ChassisTick(void)
{
    if (!g_tune_mode) MecanumChassis_Loop(&chassis, 0.01f);
}
