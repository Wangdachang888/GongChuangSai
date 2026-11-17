/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "mecanum_chassis.h"
#include "app_imu.h"
#include "display_nav.h"
#include "bsp_oled.h"
#include "app_stepper_openloop.h"
#include "bsp_d36_cmd.h"
#include "bsp_vbat.h"
#include "app_stepper_drive.h"
#include "app_square.h"
#include "vofa_debug.h"
#include "bsp_ulog.h"
#include "app_mode.h"
#include "app_race.h"
#include "bsp_servo.h"
#include "app_servo_key.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Motor_t motor_left_front;   // ���A = ��ǰ��
Motor_t motor_right_front;  // ���B = ��ǰ��
Motor_t motor_left_back;    // ���C = �����
Motor_t motor_right_back;   // ���D = �Һ���

MecanumChassis_t chassis;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ��ʾ��ʵ�ʻ����� MecanumChassis_Init ����ɣ�
 *   440 ����/��Ȧ �� (2�С�0.04 m) �� 1750 ����/��
 *   ���������ֻ����ע�͵��Ĳ��Դ����ã���ע�͵� 6500 �Ǵ��� */
#define PULSE_PER_METER  1750
#define SPEED            1000    /* ǰ���ٶȣ�����/�� */

/* ==========================================================================
 * �������뵼����ICM20948 + ��ʽ��̼���ϵ�����
 * --------------------------------------------------------------------------
 * ��ʼ��˳�򣨼� USER CODE 2����
 *   1. OLED_Init()                  OLED ��ʾ
 *   2. DisplayNav_Init()            PE0 �������������룩������
 *   3. app_imu_init()               PB10/PB11 ���� I2C��Լ 1s���ڼ䳵���뾲ֹ
 *   4. HAL_TIM_Base_Start_IT(&htim6) ���ſ� 1ms �жϣ���ֹ��ʼ��δ���
 *
 * ��ѭ����
 *   app_imu_poll()       200Hz �� IMU + ��̬���㣨�ڲ��Զ���Ƶ����ֹ���жϣ�
 *   DisplayNav_Poll()    100ms ˢ��һҳ���̰� PE0 ��ҳ������ 0.8s λ�˹���
 *
 * λ����Դ��nav.c������ = IMU Ϊ�� + ���ٸ�����ƽ�� = ����ͶӰ����
 *          ���д�� chassis.x_pos / y_pos / theta_pos��λ�û�ֱ��ʹ�á�
 * ========================================================================== */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
 
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
#if APP_RUN_MODE == APP_MODE_SERVO
    /* Servo bring-up: freeze chassis PWM, PE0 short=+30 long=-30 */
    __HAL_TIM_MOE_DISABLE(&htim1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    (void)HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);

    Servo_Init();                 /* boot angle 0 deg */
    ServoKey_Init();              /* OLED + PE0 */
    while (1) {
        ServoKey_Poll();
    }
#else
#if APP_RUN_MODE != APP_MODE_RACE
    AppStepper_Init();  /* D36 lift stepper */
    Servo_Init();
    Servo_SetBothAngle(30.0f, 30.0f); /* each servo -> 30 deg */
    OLED_Init();
    D36Cmd_Init();      /* USART3 text commands -> D36 */
#else
    OLED_Init();
#endif
    /* 3. �߼���ʱ��TIM1��������������룡������PWM�� */
	__HAL_TIM_MOE_ENABLE(&htim1);
    
    /* 4. �ĸ������ʼ������ȫ��Ӧ���ű��� */
    // ���A����ǰ����PWM=TIM10_CH1+TIM11_CH1��������=TIM2
    Motor_Init(&motor_left_front,
               &htim10, TIM_CHANNEL_1,  4199,
               &htim11, TIM_CHANNEL_1,  4199,
               &htim2,
               2.5f, 1.60f, 0.0f,1,1
               );

    // ���B����ǰ����PWM=TIM9_CH1+TIM9_CH2��������=TIM3
    Motor_Init(&motor_right_front,
               &htim9, TIM_CHANNEL_1,  4199,
               &htim9, TIM_CHANNEL_2,  4199,
               &htim3,
               2.5f, 1.60f, 0.0f,1,-1
               );

    // ���C����󣩣�PWM=TIM1_CH1+TIM1_CH2��������=TIM4
    Motor_Init(&motor_left_back,
               &htim1, TIM_CHANNEL_1,  8399,
               &htim1, TIM_CHANNEL_2,  8399,
               &htim4,
               2.5f, 1.60f, 0.0f,-1,1
               );

    // ���D���Һ󣩣�PWM=TIM1_CH3+TIM1_CH4��������=TIM5
    Motor_Init(&motor_right_back,
               &htim1, TIM_CHANNEL_3,  8399,
               &htim1, TIM_CHANNEL_4,  8399,
               &htim5,
               2.5f, 1.60f, 0.0f,-1,-1
               );

//kp	��������	Խ����ӦԽ�죬���������
//ki	��������	������̬������ᳬ��/���ֱ���
//kd	΢������	���Ƴ�����������������

    /* 5. ���̳�ʼ�� */
    MecanumChassis_Init(&chassis,
                        &motor_left_front, &motor_right_front,
                        &motor_left_back, &motor_right_back,
                        0.04f, 0.093f, 0.11f,
                        1.0f, 440.0f);
//wheel_radius		    ���Ӱ뾶						m			���߲������� 50mm ���� = 0.05f
//wheel_base_half	    ����ࣨǰ�������ľ��һ�룩	m			����ǰ�������ľ��� �� 2
//track_width_half	    ���־ࣨ���������ľ��һ�룩	m			�������������ľ��� �� 2
//gear_ratio			���ٱ�							��			����ֱ���� 1
//enc_ppr_x4			������ 4 ��Ƶÿת������			��			���������� �� 4���� 500 �� = 2000.0f

    /* 6. ����λ�û�PID������ʾ��ֵ�����ֳ������� */
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

    /* Yaw must hold during translate; weak kp caused ~6deg crab-walk on side2 */
    chassis.pid_theta.kp = 3.2f;
    chassis.pid_theta.ki = 0.08f;
    chassis.pid_theta.kd = 0.18f;
    chassis.pid_theta.integral_max = 0.50f;
    chassis.pid_theta.output_max = PI;
	
    /* 7. ���� 1ms ��ʱ�жϣ���󿪣���ֹ��ʼ��δ��ɾʹ����� */
    /* 7.5 IMU ��ʼ����Լ 1s��100ms ��λ + 0.8s ��ƫ�궨�����˿̳����뾲ֹ��
     *     ���� TIM6 ����֮ǰ������궨�ڼ���̻���ʼ������
     *     �ڲ��᣺����λ -> ������/�˲�/ODR -> �������� -> ������ƫ�궨�� */
#if APP_RUN_MODE != APP_MODE_RACE
    DisplayNav_Init();
    VOFA_Init();                 /* USART2 PD5/PD6 via ULog + USART1 �� VOFA+ ���� */
#endif
    Vbat_Init();                 /* PC0/PB0 ADC: driver and board VIN */
    if (app_imu_init() != 0)
    {
#if APP_RUN_MODE != APP_MODE_RACE
        Error_Handler();
#endif
    }

#if APP_RUN_MODE == APP_MODE_RACE
    ULog_Init();                 /* USART2 TF logger, independent of VOFA */
    AppRace_Init(&chassis);
#endif
    HAL_TIM_Base_Start_IT(&htim6);
    Mecanum_Stop(&chassis);          /* ���ʾ��Ŀ�꣬�����ϵ����� */

    /* ˳ʱ�������Σ��߳� 0.5 m���ϵ�� 2 ���ʼ
     * ������ app_square.h��SQUARE_SIDE_M / SQUARE_VMAX / SQUARE_KP_POS ...
     * ���Ȳⲽ��������������л��� StepperDrive_Init(); StepperDrive_DemoStart(); */
#if APP_RUN_MODE != APP_MODE_RACE
#if (MOTOR_TUNE != 0)
    Tune_Start(&motor_left_front);
#else
    Square_Start(&chassis);
#endif
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if APP_RUN_MODE == APP_MODE_RACE
    app_imu_poll();
    AppRace_Poll();
    Vbat_Poll();
    ULog_Poll(&chassis);
#else
//	  int32_t now = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

//	  if (now - enc_start <= PULSE_PER_METER) 
//	{
	#if (MOTOR_TUNE != 0)
        Tune_Poll();                 /* �����Ծ���� + VOFA+ ������ */
#else
        Square_Poll(&chassis);       /* �����ι켣��20ms ���� */
#endif
        app_imu_poll();          /* 200Hz ��̬���£��� IMU + ���㣬����ж� */
    AppStepper_Poll();      /* continuous spin feed */
    D36Cmd_Poll();         /* parse USART3 stepper commands */
    Vbat_Poll();                 /* 100ms ���µ�Դ��ѹ */
    DisplayNav_Poll(&chassis);   /* 100ms ˢ��һҳ���̰� PE0 ��ҳ���������� */
    ULog_Poll(&chassis);          /* USART2 PD5/PD6 -> microSD CSV log */

    /* Square/Tune own stop; do not Mecanum_Stop on target_reached
     * or PH_GUIWEI position hold (kp boost) will be aborted. */
//	}
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  
#endif
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t chassis_cnt = 0;

    if (htim->Instance == TIM6)
    {
#if APP_RUN_MODE == APP_MODE_RACE
        if (!AppRace_ControlTick()) return;
#endif
		static uint8_t speed_cnt = 0;

        speed_cnt++;
        if (speed_cnt >= 1)     /* ÿ 1ms ��һ�� */
        /* 1ms��4 ·����ٶȻ��ջ� */
        {
		speed_cnt = 0;
		Motor_Update(&motor_left_front,  0.001f);
        Motor_Update(&motor_right_front, 0.001f);
        Motor_Update(&motor_left_back,   0.001f);
        Motor_Update(&motor_right_back,  0.001f);
		}
        /* 10ms�������˶�ѧ + λ�û� */
        chassis_cnt++;
        if (chassis_cnt >= 10)
        {
            chassis_cnt = 0;
            if (!g_tune_mode)
            {
                MecanumChassis_Loop(&chassis, 0.01f);
            }
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */





