/**
 * @file    display_nav.h
 * @brief   导航/调参可视化页面（128x64 OLED，12px 字体）
 *
 * 4 个页面，按 PE0 用户按键切换（短按翻页，长按 0.8s 位姿归零）：
 *   P0 NAV     ：x / y / yaw / vx / vy / omega
 *   P1 CHASSIS ：位置环目标值与实际值对比
 *   P2 PARAM   ：导航可调参数（只读显示，改参数在 nav.c）
 *   P3 STATUS  ：IMU 帧数、错误、静止/打滑标志
 *
 * 用法（main 的 while(1) 里调用，内部自带 100ms 节流与按键扫描）：
 * @code
 *   OLED_Init();
 *   DisplayNav_Init();
 *   ...
 *   DisplayNav_Poll(&chassis);
 * @endcode
 */
#ifndef __DISPLAY_NAV_H
#define __DISPLAY_NAV_H

#include "mecanum_chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 刷新周期（ms），OLED 全屏刷新约 1~2ms，别设太小 */
#ifndef DISPLAY_NAV_PERIOD_MS
#define DISPLAY_NAV_PERIOD_MS   100u
#endif

/** 是否使用 PE0 按键翻页（需要把 PE0 从推挽输出改成上拉输入） */
#ifndef DISPLAY_NAV_USE_KEY
#define DISPLAY_NAV_USE_KEY     1
#endif

#ifndef DISPLAY_NAV_KEY_PORT
#define DISPLAY_NAV_KEY_PORT    GPIOE
#define DISPLAY_NAV_KEY_PIN     GPIO_PIN_0
#endif

/** 长按判定时间（ms），长按 = 位姿归零 */
#ifndef DISPLAY_NAV_LONG_MS
#define DISPLAY_NAV_LONG_MS     800u
#endif

/** 初始化（配置按键引脚，清屏） */
void DisplayNav_Init(void);

/** 周期调用：按键扫描 + 按需刷新页面 */
void DisplayNav_Poll(MecanumChassis_t *chassis);

/** 手动切到下一页（也可以外部按键调用） */
void DisplayNav_NextPage(void);

/** 当前页面号 0~3 */
uint8_t DisplayNav_GetPage(void);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_NAV_H */
