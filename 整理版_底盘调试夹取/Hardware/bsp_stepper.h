/**
  ******************************************************************************
  * @file    bsp_stepper.h
  * @brief   D36A �����������������������TIM8 ˫ͨ����֧��ָ�������Զ�ֹͣ��
  ******************************************************************************
  * ����ԭ�������ֱض�����
  *
  * 1. D36A ��"����+����"����������
  *    - STEP ����ÿ��һ�������أ������תһ��ϸ�ֲ���
  *    - DIR  ���ŵĵ�ƽ����ת������
  *
  * 2. �������� TIM8 ��"����Ƚ�-��תģʽ"�������壺
  *    - TIM8 �� 100kHz Ƶ�ʼ�����ÿ 10us �� 1����
  *    - ÿ������ֵ׷�ϱȽϼĴ��� CCR ʱ�����ŵ�ƽ�Զ���תһ�Σ��������жϣ�
  *    - ���ж���� CCR ������һ��"������"����������͵õ�������
  *    - ��ת 2 �� = 1 ���������壬���Է� N ����Ҫ��ת 2N �Σ�
  *    - ��ת�������˾͹ص���ͨ�� ���� ���ʵ����"���̶��������Զ�ͣ"��
  *
  * 3. ���"PWMģʽ+������"�����ַ�ʽ���ŵ��������������ͬһ����ʱ��
  *    ���������ɵ�Ƶ�ʺͲ�����CCR1 ����1��CCR2 ����2����
  ******************************************************************************
  */
#ifndef __BSP_STEPPER_H
#define __BSP_STEPPER_H

#include <stdint.h>
#include "main.h"

/* TIM8 ����Ƶ�� 100kHz��1 �� = 2 �η�ת����������Ƶ�ʷ�Χ�� 1 ~ 5000 Hz */
/* D36A �� EN ���ԣ�0 = �ߵ�ƽʹ�ܣ��ֲ�Ĭ�ϣ���1 = �͵�ƽʹ�� */
#ifndef STEPPER_EN_ACTIVE_LOW
#define STEPPER_EN_ACTIVE_LOW   0
#endif

#define STEPPER_EN_LEVEL_ON     (STEPPER_EN_ACTIVE_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET)
#define STEPPER_EN_LEVEL_OFF    (STEPPER_EN_ACTIVE_LOW ? GPIO_PIN_SET   : GPIO_PIN_RESET)

#define STEPPER_TIMER_TICK_HZ    100000U
#define STEPPER_MIN_FREQ_HZ           1U
#define STEPPER_MAX_FREQ_HZ        5000U

/* ���ţ���1 = ST1:PC6 / DIR1:PC8 / EN1:PA2����2 = ST2:PC7 / DIR2:PC9 / EN2:PA3 */
typedef enum
{
    STEPPER_AXIS_1 = 0,
    STEPPER_AXIS_2 = 1
} StepperAxis_t;

/* ��ʼ������ MX_TIM8_Init ֮�����һ�Σ�����ʼ���������ᶯ */
void Stepper_Init(void);

/* ���÷����ƽ����������в������ķ��򣬻᷵�� HAL_BUSY */
HAL_StatusTypeDef Stepper_SetDirection(StepperAxis_t axis,
                                       GPIO_PinState direction);
GPIO_PinState Stepper_GetDirection(StepperAxis_t axis);

/* EN: high=enable, low=sleep (D36A manual). Axis 2 starts disabled. */
void Stepper_SetEnable(StepperAxis_t axis, uint8_t enable);
void Stepper_SetEnableAll(uint8_t enable);
uint8_t Stepper_GetEnable(StepperAxis_t axis);

/**
  * ����һ�����壺�� steps �����壬Ƶ�� frequency_hz�������Զ�ͣ��
  * �������ز��ȴ������ỹû����ʱ�ٴε��û᷵�� HAL_BUSY��
  */
HAL_StatusTypeDef Stepper_Start(StepperAxis_t axis,
                                uint32_t steps,
                                uint32_t frequency_hz);

/**
  * ���������е����崮"����"��׷�� steps �����岢��Ƶ�ʸ���Ϊ
  * frequency_hz���¾������޷��νӣ�����������ת����
  * �������ʱ��Ч�� Stepper_Start()��
  * ע�⣺�����в�����������Ҫ�������� Stop �� SetDirection��
  */
HAL_StatusTypeDef Stepper_Feed(StepperAxis_t axis,
                               uint32_t steps,
                               uint32_t frequency_hz);

/* ����ֹͣ */
void Stepper_Stop(StepperAxis_t axis);
void Stepper_StopAll(void);

/* �����Ƿ��ڷ����壨1=æ��*/
uint8_t Stepper_IsBusy(StepperAxis_t axis);

/* ��ѯʣ��δ����Ĳ��� */
uint32_t Stepper_GetRemainingSteps(StepperAxis_t axis);

/* �� TIM8_CC_IRQHandler ���õ��жϴ���������Ӧ�ô��벻Ҫ���� */
void Stepper_TIM8_CC_IRQHandler(void);

#endif /* __BSP_STEPPER_H */
