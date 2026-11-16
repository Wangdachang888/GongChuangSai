#include "board.h"
#include "bsp_ulog.h"
#include "app_race.h"
#include "app_square.h"
#include "app_mode.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
static uint32_t now, writes;
static uint8_t finished, square = SQUARE_RUNNING;
static int tx_error, init_error, format_error;
int ulog_test_snprintf(char *buf, size_t size, const char *format, ...)
{
    int n; va_list args;
    if (format_error) return (int)size;
    va_start(args, format); n=vsnprintf(buf,size,format,args); va_end(args); return n;
}
static RaceFsm race;
static RaceTaskPlan plan;
static char last[800];
uint32_t HAL_GetTick(void) { return now; }
float app_imu_yaw_deg(void) { return 0; }
uint8_t AppRace_LogFinished(void) { return finished; }
const RaceFsm *AppRace_Status(void) { return &race; }
const RaceTaskPlan *AppRace_TaskPlan(void) { return &plan; }
uint8_t Square_GetState(void) { return square; }
uint8_t Square_GetStep(void) { return 0; }
float Vbat_GetDriverVoltage(void) { return 12.0f; }
float Vbat_GetBoardVoltage(void) { return 11.9f; }
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config) { (void)port; (void)config; }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart) { (void)uart; return init_error; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len, uint32_t timeout)
{
    assert(uart->Init.BaudRate == 115200 && timeout >= 30 && len < sizeof(last));
    memcpy(last, data, len); last[len] = 0; writes++;
    return tx_error;
}
int main(int argc, char **argv)
{
    Motor_t motors[4] = {{0}};
    MecanumChassis_t c = {0};
    uint32_t count;
    int commas = 0;
    const char *p;
    const char *scenario = argc > 1 ? argv[1] : "normal";
    c.motor_lf=&motors[0]; c.motor_rf=&motors[1]; c.motor_lb=&motors[2]; c.motor_rb=&motors[3];
    race.state=RACE_RUNNING; race.action_count=1;
    if (!strcmp(scenario,"init_error")) init_error=HAL_ERROR;
    ULog_Init();
#if ULOG_ENABLE == 0
    assert(!ULog_IsRecording()); ULog_Poll(&c); ULog_Printf("x"); assert(!writes); return 0;
#endif
    if (init_error) { ULog_Poll(&c); ULog_Printf("x"); assert(!writes && !ULog_IsRecording()); return 0; }
    assert(ULog_IsRecording());
    ULog_Poll(&c); assert(!writes);
    if (!strcmp(scenario,"early")) { finished=1; square=SQUARE_DONE; }
    if (finished) {
        ULog_Poll(&c); assert(!writes && !ULog_IsRecording()); return 0;
    }
    now=ULOG_STARTUP_WAIT_MS;
    ULog_Poll(&c); assert(writes == 1);
    for (p=last; *p; p++) if (*p==',') commas++;
    assert(commas == 42 && strstr(last,"\r\n"));
    now++;
    ULog_Poll(&c); assert(writes == 1);
    finished=1; square=SQUARE_DONE;
    if (!strcmp(scenario,"abort")) square=SQUARE_IDLE;
    if (!strcmp(scenario,"timeout")) square=SQUARE_TIMEOUT;
    if (!strcmp(scenario,"tx_error")) tx_error=HAL_ERROR;
    if (!strcmp(scenario,"format_error")) format_error=1;
    if (!strcmp(scenario,"nonfinite")) c.theta_target=INFINITY;
    if (!strcmp(scenario,"manual")) ULog_Stop();
    if (!strcmp(scenario,"null")) ULog_Poll(0); else ULog_Poll(&c);
    assert(!ULog_IsRecording());
    if (tx_error) assert(g_ulog_tx_error_count == 1);
    if (!strcmp(scenario,"format_error")) assert(g_ulog_format_error_count == 1);
    if (!strcmp(scenario,"nonfinite")) assert(g_ulog_format_error_count == 1);
    if (!strcmp(scenario,"normal")) assert(writes == 2);
    count=writes;
    for (int i=0;i<1000;i++) { now+=50; ULog_Poll(&c); }
    ULog_WriteRaw((const uint8_t *)"raw",3); ULog_Printf("text");
    ULog_Init(); ULog_Poll(&c);
    assert(writes == count && !ULog_IsRecording());
    printf("PASS logger: %s\n",scenario);
    return 0;
}
