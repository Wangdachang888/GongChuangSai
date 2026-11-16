/**
 * @file    vofa_debug.h
 * @brief   VOFA+ �����ã������Ծ���� + FireWater �ı�������
 *
 * �÷���
 *   1. �� MOTOR_TUNE �� 1��������¼
 *    *   2. USB-TTL -> USART1 PA9(TX)/PA10(RX), 115200; USART2 reserved for TF log
 *   3. VOFA+ ѡ FireWater ���棬115200-8-N-1
 *   4. �ϵ��ÿ 3 ���һ�� 300 ����/����ٶȽ�Ծ���ٻ� 0��ѭ��
 *   5. ���ߣ�ch0=target  ch1=actual  ch2=output
 */
#ifndef __VOFA_DEBUG_H
#define __VOFA_DEBUG_H

#include "Motor.h"

/* 1 = �����Ծ����ģʽ��0 = ������������ */
#ifndef MOTOR_TUNE
#define MOTOR_TUNE            0
#endif

/* �����ڼ��� 1����ѭ��ֻ�ܽ�Ծ���ԣ�TIM6 ���������̻� */
extern volatile uint8_t g_tune_mode;

/* USART2 PD5/PD6 is initialized by ULog_Init() inside VOFA_Init(). */
void VOFA_Init(void);
void VOFA_PlotMotor(const Motor_t *m);
void Tune_Start(Motor_t *m);
void Tune_Poll(void);

#endif /* __VOFA_DEBUG_H */
