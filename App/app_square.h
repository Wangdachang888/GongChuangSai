/**
 * @file    app_square.h
 * @brief   0.5 m ���������ߣ����õ���λ�û� + �ο������ĩ�˹�λ��
 *
 * ���ԭ��
 *   1. ƽ�ƺ�ת��ȫ���������еĵ���λ�û���pid_x / pid_y / pid_theta����
 *      ���ļ�ֻ����"��һ��ȥ�ġ�ʲôʱ���㵽"��
 *   2. IMU �Ľ��뷽ʽ�ο� GongXun2025 ��д����
 *        - �н�����λ�û��� pid_theta ���ֺ���
 *        - ÿ������ʱ��һ�Ρ�ĩ�˹�λ��(GuiWei)��ͣ�� + ��ʱ�Ӵ������档
 *   3. ���������ɵ���㣨Motor.c�������������һ�㴦����
 *
 * 8 �����У�˳ʱ�룩��
 *   0 ����1(0.5, 0)      ���� 0
 *   1 ԭ��ת�� -90��
 *   2 ����2(0.5,-0.5)    ���� -90��
 *   3 ԭ��ת�� -180��
 *   4 ����3(0,-0.5)      ���� -180��
 *   5 ԭ��ת�� +90��
 *   6 ����4(0,0)         ���� +90��
 *   7 ԭ��ת�� 0��
 */
#ifndef __APP_SQUARE_H
#define __APP_SQUARE_H

#include <stdint.h>
#include "mecanum_chassis.h"

/* ---------------- �켣���� ---------------- */
/* 开场右移 / 收场左移（世界系：-Y 为右，+Y 为左） */
#ifndef SQUARE_PRE_RIGHT_M
#define SQUARE_PRE_RIGHT_M      0.20f    /* first: translate right (m), world -Y */
#endif
#ifndef SQUARE_POST_LEFT_M
#define SQUARE_POST_LEFT_M      0.20f    /* unused if HOME to (0,0); keep for docs */
#endif

#ifndef SQUARE_SIDE_M
#define SQUARE_SIDE_M           1.80f    /* square side (m) */
#endif
#ifndef SQUARE_START_DELAY_MS
#define SQUARE_START_DELAY_MS   18000u   /* �� IMU Ԥ��+�궨��� */
#endif

#ifndef SQUARE_VMAX
#define SQUARE_VMAX             0.55f   /* cruise m/s; lower = less coast overshoot */
#endif
#ifndef SQUARE_WMAX
#define SQUARE_WMAX             1.20f   /* yaw cruise rad/s */
#endif

#ifndef SQUARE_TOL_M
#define SQUARE_TOL_M            0.015f   /* λ�õ�λ�ݲ� m */
#endif
#ifndef SQUARE_TOL_RAD
#define SQUARE_TOL_RAD          0.018f   /* �Ƕȵ�λ�ݲ� rad��Լ 2�㣩 */
#endif

/* ---------------- ĩ�˹�λ����Ӧ�ο����� Chassis_GuiWei�� ---------------- */
#ifndef SQUARE_GUIWEI_MS
#define SQUARE_GUIWEI_MS        700u     /* ��λ����ʱ�� ms��2m �߳��� 300�� */
#endif
#ifndef SQUARE_GUIWEI_KP_SCALE
#define SQUARE_GUIWEI_KP_SCALE  4.0f     /* ��λʱ theta �� Kp �Ŵ��� */
#endif

/* ---------------- ��ʱ ---------------- */
#ifndef SQUARE_STEP_TIMEOUT_MS
#define SQUARE_STEP_TIMEOUT_MS  20000u   /* ������ʱ */
#endif

/* ---------------- ״̬ ---------------- */
#define SQUARE_IDLE             0u
#define SQUARE_RUNNING          1u
#define SQUARE_DONE             2u
#define SQUARE_TIMEOUT          3u

/* ---------------- �ӿ� ---------------- */
void    Square_Start(MecanumChassis_t *chassis);
void    Square_Poll(MecanumChassis_t *chassis);   /* ��ѭ�����ã�10~20ms һ�μ��ɣ� */
void    Square_Abort(MecanumChassis_t *chassis);

uint8_t Square_GetState(void);
uint8_t Square_GetStep(void);                     /* 0~7 */
uint8_t Square_IsRunning(void);
uint8_t Square_IsDone(void);

#endif /* __APP_SQUARE_H */
