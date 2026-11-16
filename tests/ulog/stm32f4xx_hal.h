#ifndef ULOG_TEST_HAL_H
#define ULOG_TEST_HAL_H
#include "main.h"
#include <stddef.h>
int ulog_test_snprintf(char *buf, size_t size, const char *format, ...);
#include "usart.h"
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART2_CLK_ENABLE() ((void)0)
#define GPIOD ((void *)4)
#define USART2 ((void *)2)
#define GPIO_PIN_5 32U
#define GPIO_PIN_6 64U
#define GPIO_MODE_AF_PP 0
#define GPIO_PULLUP 0
#define GPIO_SPEED_FREQ_VERY_HIGH 0
#define GPIO_AF7_USART2 7
#define UART_WORDLENGTH_8B 0
#define UART_STOPBITS_1 0
#define UART_PARITY_NONE 0
#define UART_MODE_TX_RX 0
#define UART_HWCONTROL_NONE 0
#define UART_OVERSAMPLING_16 0
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config);
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len, uint32_t timeout);
#endif
