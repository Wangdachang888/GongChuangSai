#include "race_ui.h"
#include "bsp_oled.h"
#include "race_vision.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

static uint32_t s_screen_time;
static uint32_t s_page_time;
static uint8_t s_page;

static void text_line(uint8_t row, const char *text)
{
    uint8_t column = 0U;
    while (*text && column < 21U) {
        OLED_ShowChar((uint8_t)(column * 6U), row, (uint8_t)*text++, 12U, 1U);
        column++;
    }
}

void RaceInput_Init(RaceInput *input, uint32_t now)
{
    uint8_t pressed;
    if (input == 0) return;
    pressed = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET;
    input->raw = pressed;
    input->stable = pressed;
    input->armed = 0U;
    input->long_sent = 0U;
    input->start_on_release = 0U;
    input->changed_ms = now;
    input->pressed_ms = now;
}

uint8_t RaceInput_Poll(RaceInput *input, uint32_t now,
                       uint8_t race_running, uint8_t can_start)
{
    uint8_t event = 0U;
    uint8_t pressed;

    if (input == 0) return 0U;
    pressed = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET;
    if (pressed != input->raw) {
        input->raw = pressed;
        input->changed_ms = now;
    }
    if ((uint32_t)(now - input->changed_ms) < 20U) return 0U;
    if (!input->armed) {
        input->stable = pressed;
        if (!pressed) input->armed = 1U;
        return 0U;
    }
    if (pressed != input->stable) {
        input->stable = pressed;
        if (pressed) {
            input->pressed_ms = now;
            input->long_sent = 0U;
            input->start_on_release = can_start;
            event |= RACE_INPUT_PRESS;
        } else if (!input->long_sent && input->start_on_release) {
            event |= RACE_INPUT_SHORT_RELEASE;
        }
    }
    if (pressed && !input->long_sent &&
        (uint32_t)(now - input->pressed_ms) >= 700U) {
        input->long_sent = 1U;
        if (!race_running) event |= RACE_INPUT_LONG;
    }
    return event;
}

void RaceUi_Init(void)
{
    s_screen_time = 0U;
    s_page_time = 0U;
    s_page = 0U;
}

void RaceUi_Poll(const RaceFsm *race, uint32_t now, uint8_t warmed_up)
{
    static const char *const states[] = {
        "READY", "RUNNING", "MATERIAL READY", "FAULT", "STOPPED"
    };
    static const char *const actions[] = {"MOVE", "TURN", "SETTLE", "SCAN"};
    char line[32];
    uint8_t column;

    if (race == 0) return;
    if (s_page == 0U && (uint32_t)(now - s_screen_time) >= 200U) {
        s_screen_time = now;
        memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
        (void)snprintf(line, sizeof(line), "RACE Z%u %s", race->zone,
                       states[race->state]);
        text_line(0U, line);
        if (race->state == RACE_READY) {
            text_line(12U, warmed_up ? "SHORT:START" : "IMU WARMUP: KEEP STILL");
            text_line(24U, "LONG:ZONE 1/2");
        } else if (race->state == RACE_RUNNING) {
            (void)snprintf(line, sizeof(line), "%u/%u %s",
                           race->action_index + 1U, race->action_count,
                           actions[race->actions[race->action_index].kind]);
            text_line(12U, line);
            text_line(24U, "PRESS:STOP");
        } else {
            (void)snprintf(line, sizeof(line), "ERROR:%u RESET TO ARM",
                           (unsigned)race->error);
            text_line(12U, line);
        }
        text_line(36U, race->task_code);
        text_line(48U, RaceVision_Healthy() ? "USART3:RX READY" : "USART3:FAULT");
    }
    if ((uint32_t)(now - s_page_time) < 2U) return;
    s_page_time = now;
    OLED_WR_Byte((uint8_t)(0xb0U + s_page), OLED_CMD);
    OLED_WR_Byte(0x00U, OLED_CMD);
    OLED_WR_Byte(0x10U, OLED_CMD);
    for (column = 0U; column < 128U; column++) {
        OLED_WR_Byte(OLED_GRAM[column][s_page], OLED_DATA);
    }
    s_page = (uint8_t)((s_page + 1U) & 7U);
}
