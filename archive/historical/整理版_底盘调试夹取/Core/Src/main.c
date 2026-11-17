#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "motor.h"
#include "mecanum_chassis.h"
#include "app_imu.h"
#include "app_stepper_openloop.h"
#include "bsp_d36_cmd.h"
#include "bsp_servo.h"
#include "app_pick_control.h"

Motor_t motor_left_front;
Motor_t motor_right_front;
Motor_t motor_left_back;
Motor_t motor_right_back;
MecanumChassis_t chassis;

void SystemClock_Config(void);

static void Chassis_Init(void)
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
    chassis.pid_y = chassis.pid_x;
    chassis.pid_theta.kp = 3.2f;
    chassis.pid_theta.ki = 0.08f;
    chassis.pid_theta.kd = 0.18f;
    chassis.pid_theta.integral_max = 0.50f;
    chassis.pid_theta.output_max = 3.14159265f;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM6_Init();
    MX_TIM8_Init();
    MX_TIM9_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();
    MX_TIM12_Init();
    MX_USART3_UART_Init();

    __HAL_TIM_MOE_ENABLE(&htim1);
    Chassis_Init();
    Servo_Init();
    AppStepper_Init();
    D36Cmd_Init();
    PickControl_Init();

    if (app_imu_init() != 0) {
        Error_Handler();
    }
    HAL_TIM_Base_Start_IT(&htim6);
    Mecanum_Stop(&chassis);

    while (1) {
        app_imu_poll();
        AppStepper_Poll();
        D36Cmd_Poll();
        PickControl_Poll();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t chassis_cnt;

    if (htim->Instance != TIM6) return;

    Motor_Update(&motor_left_front, 0.001f);
    Motor_Update(&motor_right_front, 0.001f);
    Motor_Update(&motor_left_back, 0.001f);
    Motor_Update(&motor_right_back, 0.001f);

    if (++chassis_cnt >= 10U) {
        chassis_cnt = 0U;
        MecanumChassis_Loop(&chassis, 0.01f);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 4;
    osc.PLL.PLLN = 168;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
