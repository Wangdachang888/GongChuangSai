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

#include "usart.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef APP_D36_UART_CMD_ENABLE
#define APP_D36_UART_CMD_ENABLE     1
#endif

typedef struct {
    uint8_t (*is_busy)(uint8_t axis, void *user);
    uint32_t (*get_remaining_steps)(uint8_t axis, void *user);
    HAL_StatusTypeDef (*start_continuous)(uint8_t axis, int8_t direction,
                                          uint32_t frequency_hz, void *user);
    HAL_StatusTypeDef (*move_steps)(uint8_t axis, int32_t steps,
                                    uint32_t frequency_hz, void *user);
    void (*stop)(uint8_t axis, void *user);
    void (*stop_all)(void *user);
    void (*set_enable)(uint8_t axis, uint8_t enable, void *user);
    void (*set_enable_all)(uint8_t enable, void *user);
} D36CmdStepperOps;

void D36Cmd_SetStepperOps(const D36CmdStepperOps *ops, void *user);
void D36Cmd_Init(void);
void D36Cmd_Poll(void);
void D36Cmd_Printf(const char *s);
void D36Cmd_RxCpltCallback(UART_HandleTypeDef *huart);
void D36Cmd_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_D36_CMD_H */
