/**
 * @file    app_servo_key.c
 * @brief   PE0 USER key: short=+30°, long=-30° on both servos
 */
#include "app_servo_key.h"
#include "bsp_servo.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#if SERVO_KEY_USE_OLED
#include "bsp_oled.h"
#endif

static uint8_t s_raw_pressed;
static uint8_t s_stable_pressed;
static uint8_t s_click_armed;
static uint8_t s_consume_release;
static uint8_t s_wait_boot_release;
static uint32_t s_key_changed_ms;
static uint32_t s_press_start_ms;
static uint32_t s_oled_ms;
static int8_t s_last_dir; /* +1 short, -1 long */

static uint8_t KeyPressed(void)
{
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void Beep(uint32_t ms)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(ms);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

#if SERVO_KEY_USE_OLED
static void Text(uint8_t x, uint8_t y, const char *s)
{
    while (*s != '\0') {
        OLED_ShowChar(x, y, (uint8_t)*s++, 12U, 1U);
        x = (uint8_t)(x + 6U);
    }
}

static void ShowUi(uint8_t force)
{
    uint8_t x, page;
    uint32_t now = HAL_GetTick();
    uint32_t a1, a2;

    if (force == 0U && (now - s_oled_ms) < SERVO_KEY_OLED_MS) {
        return;
    }
    s_oled_ms = now;

    for (x = 0U; x < 128U; ++x) {
        for (page = 0U; page < 8U; ++page) {
            OLED_GRAM[x][page] = 0U;
        }
    }

    Text(0U, 0U, "SERVO PE0");
    if (s_last_dir > 0) {
        Text(78U, 0U, "+30");
    } else if (s_last_dir < 0) {
        Text(78U, 0U, "-30");
    } else {
        Text(78U, 0U, "---");
    }

    Text(0U, 16U, "S=+30 L=-30");
    if (Servo_IsBusyAny() != 0U) {
        Text(90U, 16U, "MOV");
    } else {
        Text(90U, 16U, "OK ");
    }

    a1 = (uint32_t)(Servo_GetAngle(SERVO_1) + 0.5f);
    a2 = (uint32_t)(Servo_GetAngle(SERVO_2) + 0.5f);
    Text(0U, 32U, "A1:");
    OLED_ShowNumber(18U, 32U, a1, 3U, 12U);
    Text(48U, 32U, "A2:");
    OLED_ShowNumber(66U, 32U, a2, 3U, 12U);

    Text(0U, 48U, "PB14 PB15");
    OLED_Refresh_Gram();
}
#endif

static void DoNudge(float delta)
{
    s_last_dir = (delta >= 0.0f) ? 1 : -1;
    Servo_NudgeAll(delta);
    Beep((delta >= 0.0f) ? 30U : 80U);
#if SERVO_KEY_USE_OLED
    ShowUi(1U);
#endif
}

void ServoKey_Init(void)
{
    s_raw_pressed = 0U;
    s_stable_pressed = 0U;
    s_click_armed = 0U;
    s_consume_release = 0U;
    s_key_changed_ms = HAL_GetTick();
    s_press_start_ms = 0U;
    s_oled_ms = 0U;
    s_last_dir = 0;
    s_wait_boot_release = KeyPressed();

#if SERVO_KEY_USE_OLED
    OLED_Init();
    ShowUi(1U);
#endif
}

void ServoKey_Poll(void)
{
    Servo_Poll();
    uint32_t now = HAL_GetTick();
    uint8_t raw = KeyPressed();

    if (raw != s_raw_pressed) {
        s_raw_pressed = raw;
        s_key_changed_ms = now;
    }

    if ((now - s_key_changed_ms) >= SERVO_KEY_DEBOUNCE_MS) {
        if (s_stable_pressed != s_raw_pressed) {
            s_stable_pressed = s_raw_pressed;

            if (s_stable_pressed != 0U) {
                s_press_start_ms = now;
                if (s_wait_boot_release != 0U) {
                    s_click_armed = 0U;
                } else {
                    s_click_armed = 1U;
                }
            } else {
                if (s_wait_boot_release != 0U) {
                    s_wait_boot_release = 0U;
                    s_click_armed = 0U;
                    s_consume_release = 0U;
                } else if (s_consume_release != 0U) {
                    s_consume_release = 0U;
                    s_click_armed = 0U;
                } else if (s_click_armed != 0U) {
                    uint32_t held = now - s_press_start_ms;
                    s_click_armed = 0U;
                    if (held >= SERVO_KEY_LONG_PRESS_MS) {
                        DoNudge(-SERVO_KEY_STEP_DEG);
                    } else {
                        DoNudge(+SERVO_KEY_STEP_DEG);
                    }
                }
            }
        }
    }

#if SERVO_KEY_USE_OLED
    ShowUi(0U);
#endif
}

