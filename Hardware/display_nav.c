/**
 * @file    display_nav.c
 * @brief   导航/调参可视化页面实现
 *
 * 中文界面：16x16 汉字（每行最多 8 个），数字用 12px ASCII。
 * 汉字只能使用 oledfont.h 里 cfont16 已有的字，否则不显示。
 * 页面切换：PE0 短按；长按 0.8s 把位姿和 IMU 航向一起归零。
 */
#include "display_nav.h"

#include "bsp_oled.h"
#include "app_imu.h"
#include "nav.h"
#include "bsp_vbat.h"
#include "app_square.h"

#include <stdint.h>

#define CW              6u          /* 12px 字体字符宽度 */
#define ROW0            0u
#define ROW1            12u
#define ROW2            24u
#define ROW3            36u
#define ROW4            48u

/* 中文页面的 4 行（16px 行高），ASCII 数字画在行内 +3 处 */
#define CN_ROW0         0u
#define CN_ROW1         16u
#define CN_ROW2         32u
#define CN_ROW3         48u

#define PAGE_COUNT      4u

#ifndef DISPLAY_NAV_USE_KEY
#define DISPLAY_NAV_USE_KEY 1
#endif

/* ------------------------------------------------------------------------- */
/* 内部状态                                                                  */
/* ------------------------------------------------------------------------- */
static uint8_t  s_page;
static uint32_t s_last_draw_ms;

#if DISPLAY_NAV_USE_KEY
static uint8_t  s_key_last;
static uint8_t  s_key_long_done;
static uint32_t s_key_press_ms;
static uint32_t s_key_edge_ms;
#endif

/* ------------------------------------------------------------------------- */
/* 基础绘图                                                                  */
/* ------------------------------------------------------------------------- */

static void put_str(uint8_t x, uint8_t y, const char *s)
{
    while (*s != '\0') {
        OLED_ShowChar(x, y, (uint8_t)(*s), 12, 1);
        x = (uint8_t)(x + CW);
        s++;
    }
}

static void put_char(uint8_t x, uint8_t y, char c)
{
    OLED_ShowChar(x, y, (uint8_t)c, 12, 1);
}

/** 无符号整数，固定位数，6px 步进 */
static void put_uint(uint8_t x, uint8_t y, uint32_t v, uint8_t digits)
{
    uint32_t div = 1u;
    uint8_t  i;

    for (i = 1u; i < digits; i++) {
        div *= 10u;
    }
    for (i = 0; i < digits; i++) {
        uint32_t d = (div > 0u) ? ((v / div) % 10u) : 0u;
        put_char(x, y, (char)('0' + d));
        x = (uint8_t)(x + CW);
        if (div > 0u) {
            div /= 10u;
        }
    }
}

/**
 * @brief 定点小数：+012.34 / -0.001
 * @param ilen 整数位数（含前导空格），dlen 小数位数
 * @note  总宽度 = 1(符号) + ilen + 1(小数点) + dlen 个字符
 */
static void put_fixed(uint8_t x, uint8_t y, float v, uint8_t ilen, uint8_t dlen)
{
    uint32_t scale = 1u, div, ip, fp;
    uint8_t  i, pad = 1u;

    if (!(v == v)) {                 /* NaN */
        v = 0.0f;
    }
    if (v < 0.0f) {
        put_char(x, y, '-');
        v = -v;
    } else {
        put_char(x, y, '+');
    }
    x = (uint8_t)(x + CW);

    for (i = 0; i < dlen; i++) {
        scale *= 10u;
    }
    if (v > 9999.0f) {
        v = 9999.0f;
    }
    ip = (uint32_t)v;
    fp = (uint32_t)((v - (float)ip) * (float)scale + 0.5f);
    if (fp >= scale) {
        fp = 0u;
        ip += 1u;
    }

    div = 1u;
    for (i = 1u; i < ilen; i++) {
        div *= 10u;
    }
    for (i = 0; i < ilen; i++) {
        uint32_t d = (div > 0u) ? ((ip / div) % 10u) : 0u;
        if (d == 0u && pad && div > 1u) {
            put_char(x, y, ' ');
        } else {
            pad = 0u;
            put_char(x, y, (char)('0' + d));
        }
        x = (uint8_t)(x + CW);
        if (div > 0u) {
            div /= 10u;
        }
    }

    put_char(x, y, '.');
    x = (uint8_t)(x + CW);

    div = scale / 10u;
    for (i = 0; i < dlen; i++) {
        uint32_t d = (div > 0u) ? ((fp / div) % 10u) : 0u;
        put_char(x, y, (char)('0' + d));
        x = (uint8_t)(x + CW);
        if (div > 0u) {
            div /= 10u;
        }
    }
}

static void clear_gram(void)
{
    uint8_t n, i;

    for (n = 0; n < 8u; n++) {
        for (i = 0; i < 128u; i++) {
            OLED_GRAM[i][n] = 0x00;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* 四个页面                                                                  */
/* ------------------------------------------------------------------------- */

/* 中文小工具：画一串 16x16 汉字（每字 16px 宽） */
static void put_cn(uint8_t x, uint8_t y, const char *s)
{
    OLED_ShowCHinese(x, y, (uint8_t *)s);
}

/* 右上角页码：1/4 */
static void put_page_no(void)
{
    put_uint(96, 3, s_page + 1u, 1);
    put_char(102, 3, '/');
    put_uint(108, 3, PAGE_COUNT, 1);
}

/* P1：姿态 —— 航向、角速度、采样读数 */
static void draw_nav_page(const MecanumChassis_t *c, const Nav_t *n)
{
    put_cn(0, CN_ROW0, "姿态");
    put_page_no();

    put_cn(0, CN_ROW1, "航向");
    put_fixed(36, (uint8_t)(CN_ROW1 + 3u), app_imu_yaw_deg(), 3, 1);

    put_cn(0, CN_ROW2, "角速度");
    put_fixed(52, (uint8_t)(CN_ROW2 + 3u), n->omega_wheel, 2, 2);

    put_cn(0, CN_ROW3, "读数");
    put_uint(36, (uint8_t)(CN_ROW3 + 3u), app_imu_frames(), 6);

    (void)c;
}

/* P2：位置 —— 坐标 / 航向 / 当前段 / 速度指令 */
static void draw_chassis_page(const MecanumChassis_t *c)
{
    put_cn(0, CN_ROW0, "位置");
    put_page_no();

    put_char(0, (uint8_t)(CN_ROW1 + 3u), 'X');
    put_fixed(12, (uint8_t)(CN_ROW1 + 3u), c->x_pos, 2, 3);
    put_char(54, (uint8_t)(CN_ROW1 + 3u), 'Y');
    put_fixed(66, (uint8_t)(CN_ROW1 + 3u), c->y_pos, 2, 3);

    put_cn(0, CN_ROW2, "向");
    put_fixed(20, (uint8_t)(CN_ROW2 + 3u), app_imu_yaw_deg(), 3, 1);
    {
        uint8_t sq_state = Square_GetState();
        char    sq_ch;

        if (sq_state == SQUARE_DONE) {
            sq_ch = 'E';                          /* 走完 */
        } else if (sq_state == SQUARE_TIMEOUT) {
            sq_ch = 'T';                          /* 超时停车 */
        } else if (sq_state == SQUARE_RUNNING) {
            sq_ch = (char)('0' + (Square_GetStep() + 1u));
        } else {
            sq_ch = '-';                          /* 没启动 */
        }
        put_str(64, (uint8_t)(CN_ROW2 + 3u), "S");
        put_char(70, (uint8_t)(CN_ROW2 + 3u), sq_ch);
        put_str(76, (uint8_t)(CN_ROW2 + 3u), "/8");
    }

    put_str(0, (uint8_t)(CN_ROW3 + 3u), "V");
    put_fixed(6, (uint8_t)(CN_ROW3 + 3u), c->vx_target, 1, 2);
    put_str(42, (uint8_t)(CN_ROW3 + 3u), "W");
    put_fixed(48, (uint8_t)(CN_ROW3 + 3u), c->omega_target, 1, 2);

    /* 右下角 8 格进度条：走完一段亮一格 */
    {
        uint8_t seg;
        uint8_t done = Square_GetStep();
        uint8_t y0   = (uint8_t)(CN_ROW3 + 3u);

        if (Square_GetState() == SQUARE_DONE) {
            done = 8u;
        }
        for (seg = 0u; seg < 8u; seg++) {
            uint8_t x0 = (uint8_t)(84u + seg * 5u);
            uint8_t yy;
            uint8_t on = (seg < done) ? 1u : 0u;

            for (yy = 0u; yy < 8u; yy++) {
                OLED_DrawPoint(x0, (uint8_t)(y0 + yy), on);
                OLED_DrawPoint((uint8_t)(x0 + 1u), (uint8_t)(y0 + yy), on);
                OLED_DrawPoint((uint8_t)(x0 + 2u), (uint8_t)(y0 + yy), on);
            }
        }
    }
}

/* P3：速度 —— 目标前进、目标转向、空转次数 */
static void draw_param_page(const Nav_t *n, const MecanumChassis_t *c)
{
    put_cn(0, CN_ROW0, "速度");
    put_page_no();

    put_cn(0, CN_ROW1, "前进");
    put_fixed(36, (uint8_t)(CN_ROW1 + 3u), c->vx_target, 1, 2);

    put_cn(0, CN_ROW2, "转向");
    put_fixed(36, (uint8_t)(CN_ROW2 + 3u), c->omega_target, 1, 2);

    put_cn(0, CN_ROW3, "空转");
    put_uint(36, (uint8_t)(CN_ROW3 + 3u), n->slip_count, 5);
}

/* P4：状态 —— 电压、错误计数、按键提示 */
static void draw_status_page(const Nav_t *n)
{
    put_cn(0, CN_ROW0, "状态");
    put_page_no();

    put_cn(0, CN_ROW1, "电压");
    if (Vbat_IsDriverValid()) {
        put_fixed(36, (uint8_t)(CN_ROW1 + 3u), Vbat_GetDriverVoltage(), 2, 1);
    } else {
        put_str(36, (uint8_t)(CN_ROW1 + 3u), "--.-");
    }
    put_char(62, (uint8_t)(CN_ROW1 + 3u), '/');
    if (Vbat_IsBoardValid()) {
        put_fixed(68, (uint8_t)(CN_ROW1 + 3u), Vbat_GetBoardVoltage(), 2, 1);
    } else {
        put_str(68, (uint8_t)(CN_ROW1 + 3u), "--.-");
    }

    put_cn(0, CN_ROW2, "错误");
    put_uint(36, (uint8_t)(CN_ROW2 + 3u), app_imu_errors(), 4);

    put_cn(0, CN_ROW3, "请按键");
    (void)n;
}

/* ------------------------------------------------------------------------- */
/* 按键                                                                      */
/* ------------------------------------------------------------------------- */

#if DISPLAY_NAV_USE_KEY
static uint8_t key_pressed(void)
{
    return (HAL_GPIO_ReadPin(DISPLAY_NAV_KEY_PORT, DISPLAY_NAV_KEY_PIN) == GPIO_PIN_RESET)
               ? 1u : 0u;
}

static void key_scan(MecanumChassis_t *chassis, uint32_t now)
{
    uint8_t k = key_pressed();

    /* 20ms 消抖 */
    if (k != s_key_last && (uint32_t)(now - s_key_edge_ms) >= 20u) {
        s_key_edge_ms = now;
        s_key_last    = k;

        if (k) {
            s_key_press_ms  = now;
            s_key_long_done = 0u;
        } else if (!s_key_long_done &&
                   (uint32_t)(now - s_key_press_ms) < DISPLAY_NAV_LONG_MS) {
            DisplayNav_NextPage();
        }
    } else if (k && !s_key_long_done &&
               (uint32_t)(now - s_key_press_ms) >= DISPLAY_NAV_LONG_MS) {
        s_key_long_done = 1u;
        /* 位姿 + IMU 航向一起归零；中断里在跑底盘环，先关一下 */
        __disable_irq();
        Mecanum_ResetPosition(chassis);
        __enable_irq();
    }
}
#endif

/* ------------------------------------------------------------------------- */
/* 对外接口                                                                  */
/* ------------------------------------------------------------------------- */

void DisplayNav_Init(void)
{
#if DISPLAY_NAV_USE_KEY
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    g.Pin  = DISPLAY_NAV_KEY_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DISPLAY_NAV_KEY_PORT, &g);
#endif

    s_page        = 1u;      /* 上电直接显示“位置”页，方便看正方形进度 */
    s_last_draw_ms = 0u;

    clear_gram();
    OLED_Refresh_Gram();
}

void DisplayNav_NextPage(void)
{
    s_page = (uint8_t)((s_page + 1u) % PAGE_COUNT);
}

uint8_t DisplayNav_GetPage(void)
{
    return s_page;
}

void DisplayNav_Poll(MecanumChassis_t *chassis)
{
    uint32_t now = HAL_GetTick();
    const Nav_t *nav;

    if (chassis == NULL) {
        return;
    }

#if DISPLAY_NAV_USE_KEY
    key_scan(chassis, now);
#endif

    if ((uint32_t)(now - s_last_draw_ms) < DISPLAY_NAV_PERIOD_MS) {
        return;
    }
    s_last_draw_ms = now;

    nav = MecanumChassis_GetNav();
    if (nav == NULL) {
        return;
    }

    clear_gram();
    switch (s_page) {
    case 0u: draw_nav_page(chassis, nav);      break;
    case 1u: draw_chassis_page(chassis);       break;
    case 2u: draw_param_page(nav, chassis);             break;
    default: draw_status_page(nav);            break;
    }
    OLED_Refresh_Gram();
}
