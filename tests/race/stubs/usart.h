#ifndef RACE_TEST_USART_H
#define RACE_TEST_USART_H
#include <stdint.h>
#define USART3 ((void *)3)
#define HAL_OK 0
#define HAL_ERROR 1
typedef struct {
    void *Instance;
#ifdef ULOG_TEST
    struct { uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling; } Init;
#endif
} UART_HandleTypeDef;
extern UART_HandleTypeDef huart3;
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length);
int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length);
#endif
