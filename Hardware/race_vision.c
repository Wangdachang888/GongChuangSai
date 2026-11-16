#include "app_mode.h"
#include "race_vision.h"
#include "race_vision_protocol.h"
#include "usart.h"

#if APP_RUN_MODE == APP_MODE_RACE
#define RX_SIZE 256U
#define RX_MASK (RX_SIZE - 1U)
static volatile uint8_t s_buffer[RX_SIZE];
static volatile uint16_t s_head, s_tail;
static volatile uint8_t s_rx_fault;
static uint8_t s_byte, s_active, s_ready;
static uint8_t s_reset_command = 'R';
static RaceVisionDecoder s_decoder;

uint8_t RaceVision_Init(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_rx_fault = 0U;
    s_active = 0U;
    RaceVisionDecoder_Reset(&s_decoder);
    s_ready = HAL_UART_Receive_IT(&huart3, &s_byte, 1U) == HAL_OK;
    return s_ready;
}

uint8_t RaceVision_Healthy(void)
{
    return s_ready && !s_rx_fault;
}

uint8_t RaceVision_BeginScan(void)
{
    uint32_t saved_irq = __get_PRIMASK();
    __disable_irq();
    s_tail = s_head;
    __set_PRIMASK(saved_irq);
    RaceVisionDecoder_Reset(&s_decoder);
    s_active = 0U;
    if (!RaceVision_Healthy()) return 0U;
    if (HAL_UART_Transmit_IT(&huart3, &s_reset_command, 1U) != HAL_OK) return 0U;
    s_active = 1U;
    return 1U;
}

void RaceVision_EndScan(void)
{
    s_active = 0U;
    RaceVisionDecoder_Reset(&s_decoder);
}

uint8_t RaceVision_Poll(char code[RACE_TASK_CODE_SIZE])
{
    uint16_t budget = RX_SIZE;
    uint8_t found = 0U;
    while (budget-- && s_tail != s_head) {
        uint8_t byte = s_buffer[s_tail];
        s_tail = (uint16_t)((s_tail + 1U) & RX_MASK);
        if (s_active && !found && RaceVision_Healthy()) {
            found = RaceVisionDecoder_Push(&s_decoder, byte, code);
        }
    }
    return found && RaceVision_Healthy();
}

void RaceVision_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint16_t next;
    if (uart->Instance != USART3) return;
    next = (uint16_t)((s_head + 1U) & RX_MASK);
    if (next == s_tail) {
        s_rx_fault = 1U;
    } else {
        s_buffer[s_head] = s_byte;
        s_head = next;
    }
    if (HAL_UART_Receive_IT(&huart3, &s_byte, 1U) != HAL_OK) s_rx_fault = 1U;
}

void RaceVision_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance != USART3) return;
    s_rx_fault = 1U;
}
#endif
