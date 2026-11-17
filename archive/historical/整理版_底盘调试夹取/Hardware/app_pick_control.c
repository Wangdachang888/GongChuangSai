#include "app_pick_control.h"
#include "app_pick_fsm.h"
#include "app_stepper_openloop.h"
#include "bsp_servo.h"
#include "main.h"

#define PICK_KEY_PORT GPIOE
#define PICK_KEY_PIN  GPIO_PIN_0
#define PICK_DEBOUNCE_MS 20U
#define PICK_LONG_MS 700U

static PickFsm s_fsm;
static uint8_t s_raw;
static uint8_t s_stable;
static uint8_t s_armed;
static uint8_t s_ignore_until_release;
static uint8_t s_long;
static uint32_t s_edge_ms;
static uint32_t s_press_ms;

static const PickConfig s_config = {
    0.0f, 175.0f, 0.0f, 140.0f,
    -5333, 3733, -10400, 12000,
    -12000, 2400, -2400, 10400, 1600,
    5000U, 750.0f, 127.764f, 191.646f,
    720.0f, 1000.0f, 1000U, 10000U, 60000U,
    500U, 30U, 4667U, 5333U, 1333U, 3U,
    -1, 300, 200, 100, 800U, 250.0f
};

static uint8_t key_pressed(void)
{
    return HAL_GPIO_ReadPin(PICK_KEY_PORT, PICK_KEY_PIN) == GPIO_PIN_RESET;
}

void PickControl_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Pin = PICK_KEY_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PICK_KEY_PORT, &gpio);

    PickFsm_Init(&s_fsm, &s_config);
    s_raw = key_pressed();
    s_stable = s_raw;
    s_ignore_until_release = s_raw;
    s_armed = 0U;
    s_long = 0U;
    s_edge_ms = HAL_GetTick();
    s_press_ms = 0U;
    PickFsm_Reset(&s_fsm, s_edge_ms);
}

void PickControl_Poll(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t raw = key_pressed();

    PickFsm_Poll(&s_fsm, now);
    if (PickFsm_IsDone(&s_fsm) || s_fsm.state == PICK_FAULT ||
        s_fsm.state == PICK_ABORTED) {
        if (!Servo_IsBusyAny() && !AppStepper_IsBusy(STEPPER_AXIS_1) &&
            !AppStepper_IsBusy(STEPPER_AXIS_2)) {
            PickFsm_Reset(&s_fsm, now);
        }
    }

    if (raw != s_raw) {
        s_raw = raw;
        s_edge_ms = now;
    }
    if ((uint32_t)(now - s_edge_ms) < PICK_DEBOUNCE_MS) return;
    if (s_stable == raw) {
        if (raw && !s_long && s_armed &&
            (uint32_t)(now - s_press_ms) >= PICK_LONG_MS) {
            s_long = 1U;
        }
        return;
    }

    s_stable = raw;
    if (raw) {
        s_press_ms = now;
        s_long = 0U;
        s_armed = (s_ignore_until_release == 0U) ? 1U : 0U;
    } else if (s_armed && PickFsm_IsReady(&s_fsm)) {
        if (s_long) (void)PickFsm_StartReverse(&s_fsm, now);
        else (void)PickFsm_Start(&s_fsm, now);
        s_armed = 0U;
    } else {
        s_armed = 0U;
        s_ignore_until_release = 0U;
    }
}

uint8_t PickControl_IsBusy(void)
{
    return PickFsm_IsActive(&s_fsm);
}
