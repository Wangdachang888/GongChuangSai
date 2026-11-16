/**
  ******************************************************************************
  * @file    bsp_d36_cmd.h
  * @brief   USART3 text command protocol for open-loop D36 control
  ******************************************************************************
  * IMPORTANT:
  *   D36A itself has NO UART. Host talks to STM32 over USART3; STM32
  *   generates STEP/DIR pulses to D36A.
  *
  * Port: USART3 @ 115200 8N1  (C30D TYPE-C USB serial3: PD8=TX, PD9=RX)
  * Line ending: CR / LF / CRLF
  ******************************************************************************
  */
#ifndef __BSP_D36_CMD_H
#define __BSP_D36_CMD_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef APP_D36_UART_CMD_ENABLE
#define APP_D36_UART_CMD_ENABLE     1
#endif

void D36Cmd_Init(void);
void D36Cmd_Poll(void);
void D36Cmd_Printf(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_D36_CMD_H */
