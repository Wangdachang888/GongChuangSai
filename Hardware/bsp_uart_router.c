#include "app_mode.h"
#include "bsp_d36_cmd.h"
#include "vision.h"

/* Keep the router independent of the STM32 HAL-heavy logger header. */
#if defined(USART2)
void ULog_TxCpltCallback(void);
UART_HandleTypeDef *ULog_GetHandle(void);
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if APP_RUN_MODE == APP_MODE_RACE
    Vision_RxCpltCallback(huart);
#elif (APP_D36_UART_CMD_ENABLE != 0)
    D36Cmd_RxCpltCallback(huart);
#else
    (void)huart;
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
#if APP_RUN_MODE == APP_MODE_RACE
    Vision_ErrorCallback(huart);
#elif (APP_D36_UART_CMD_ENABLE != 0)
    D36Cmd_ErrorCallback(huart);
#else
    (void)huart;
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#if defined(USART2)
    if ((huart != 0) && (huart == ULog_GetHandle())) {
        ULog_TxCpltCallback();
        return;
    }
#endif
#if APP_RUN_MODE == APP_MODE_RACE
    Vision_TxCpltCallback(huart);
#else
    (void)huart;
#endif
}
