/**
 * @file    bsp_ulog.h
 * @brief   USART2 (PD5=TX / PD6=RX) logger for BT10/serial recorder
 *
 * Output format: Kart-log-view v4 compatible 43-channel numeric CSV (no text header).
 * Open with kart_log_view HTML (expects >=43 numeric fields per line).
 *
 * Wiring:
 *   MCU PD5 (USART2_TX) -> BT10/UART module RX
 *   MCU PD6 (USART2_RX) -> module TX (optional)
 *   GND common, 115200 8N1
 */
#ifndef __BSP_ULOG_H
#define __BSP_ULOG_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "mecanum_chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ULOG_ENABLE
#define ULOG_ENABLE           1
#endif

/* 50ms = 20Hz. Viewer DT is 20ms; 50ms is still fine for trajectory. */
#ifndef ULOG_PERIOD_MS
#define ULOG_PERIOD_MS        50u
#endif

/* High-rate inner-loop capture is an explicit diagnostic feature.  It is
 * disabled by default so a competition build cannot consume RAM or UART
 * bandwidth unexpectedly. */
#ifndef ULOG_PID_BURST_ENABLE
#define ULOG_PID_BURST_ENABLE  0
#endif

#ifndef ULOG_PID_BURST_CAPACITY
#define ULOG_PID_BURST_CAPACITY 256u
#endif

/* Give the external UART -> TF logger time to initialize.
 * This is only a startup delay; it is NOT a card-ready handshake. */
#ifndef ULOG_STARTUP_WAIT_MS
#define ULOG_STARTUP_WAIT_MS    2000u
#endif

/* Kart viewer: v_mps = raw * ULOG_PV */
#ifndef ULOG_PV
#define ULOG_PV               0.073632f
#endif

/* motor pulse/s -> m/s (matches PULSE_PER_METER=1750 in main.c) */
#ifndef ULOG_PULSE_PER_METER
#define ULOG_PULSE_PER_METER  1750.0f
#endif

typedef struct {
    uint8_t phase;
    uint8_t status;
    uint8_t action_index;
    uint8_t error;
    uint8_t plan_index;
    uint8_t plan_done;
    uint8_t finished;
    float imu_yaw_deg;
} ULogAppSnapshot;

typedef struct {
    uint32_t timestamp_ms;
    float fl_target;
    float fl_actual;
    float fl_output;
    float fr_target;
    float fr_actual;
    float fr_output;
    float rl_target;
    float rl_actual;
    float rl_output;
    float rr_target;
    float rr_actual;
    float rr_output;
} ULogPidSample;

/* Init once per boot. Stop stays latched until MCU reset. */
void ULog_Init(void);
uint8_t ULog_IsRecording(void); /* Stream enabled, NOT a card-close ACK */
void ULog_Poll(const MecanumChassis_t *chassis);
void ULog_SetAppSnapshot(const ULogAppSnapshot *snapshot);
void ULog_Stop(void);            /* Stop UART stream only, NOT a TF file-close command */

/* Explicit diagnostic capture of the 1 kHz motor inner loop.  Start/stop are
 * expected to be used only in a non-competition test build. */
void ULog_PidBurstStart(void);
void ULog_PidBurstStop(void);
uint8_t ULog_PidBurstActive(void);
void ULog_PidBurstSampleIsr(const ULogPidSample *sample);
void ULog_PidBurstPoll(void);
extern volatile uint32_t g_ulog_pid_burst_overrun;

/* Inspect these counters in Keil Watch during diagnosis. */
extern volatile uint32_t g_ulog_tx_error_count;
extern volatile uint32_t g_ulog_format_error_count;
extern volatile uint16_t g_ulog_last_line_len;

void ULog_WriteRaw(const uint8_t *data, uint16_t len);
void ULog_Printf(const char *s);
UART_HandleTypeDef *ULog_GetHandle(void);
/* Called from the single HAL UART TX-complete callback. */
void ULog_TxCpltCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ULOG_H */
