/**
 * @file    bsp_ulog.c
 * @brief   USART2 PD5/PD6 -> BT10/serial link, Kart S3 viewer compatible 43-ch CSV
 *
 * Channel map used by kart_log_view:
 *   [0]  mode        chassis mode (0 speed / 1 position)
 *   [1]  phase       0 idle, 1 move, 2 turn, 4 done
 *   [2]  status      race state / square state
 *   [3]  action      route action index / square step
 *   [6]  error       race error code
 *   [7]  plan        task-plan index
 *   [9]  plan_done   task-plan completion flag
 *   [10]/[11] vx/vy  measured body velocity (m/s)
 *   [12]/[13] vx/vy  target body velocity (m/s)
 *   [16] yaw         chassis yaw (deg)
 *   [17] yawu        imu yaw (deg)
 *   [18] yrate       omega_target (deg/s)
 *   [19] v_tgt       target speed / PV   (viewer shows *PV as m/s)
 *   [20] v_act       actual speed / PV
 *   [32] ox          x (m)
 *   [33] oy          y (m)
 *   [34] odo         odometry path length (m)
 *   [35..37] target  FR/RL/RR target pulse/s (FL is [27])
 *   [38] maxout     maximum absolute wheel control output
 *   [39] elat        body-frame lateral error (m)
 *   [40]/[41] VIN    driver and board voltage (V)
 *   [42] herr        yaw error (deg)
 * Other channels filled with useful extras or 0.
 */
#include "bsp_ulog.h"

#if (ULOG_ENABLE != 0)

#include "app_mode.h"
#include "bsp_vbat.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static UART_HandleTypeDef s_huart2;
static uint8_t  s_inited;
static uint8_t  s_stream_enabled;
static uint8_t  s_end_sent;
static uint8_t  s_data_sent;
static uint8_t  s_tx_failed;
/* TX queue keeps UART transmission out of the application polling path. */
#define ULOG_TX_QUEUE_SIZE 4096u
static uint8_t s_tx_queue[ULOG_TX_QUEUE_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static volatile uint16_t s_tx_inflight;
static volatile uint8_t s_tx_busy;
#if defined(ULOG_TEST)
#define ULOG_CRITICAL_ENTER() ((void)0)
#define ULOG_CRITICAL_EXIT()  ((void)0)
#else
#define ULOG_CRITICAL_ENTER() __disable_irq()
#define ULOG_CRITICAL_EXIT()  __enable_irq()
#endif
static uint32_t s_init_ms;
static uint32_t s_last_ms;
/* Error counters visible in Keil Watch. Non-zero values require investigation. */
volatile uint32_t g_ulog_tx_error_count;
volatile uint32_t g_ulog_format_error_count;
volatile uint16_t g_ulog_last_line_len;
static float    s_odo;
static float    s_last_x;
static float    s_last_y;
static uint8_t  s_have_last;
static ULogAppSnapshot s_app_snapshot;

#if (ULOG_PID_BURST_ENABLE != 0)
static volatile ULogPidSample s_pid_ring[ULOG_PID_BURST_CAPACITY];
static volatile uint16_t s_pid_head;
static volatile uint16_t s_pid_tail;
static volatile uint8_t s_pid_active;
static volatile uint8_t s_pid_end_pending;
volatile uint32_t g_ulog_pid_burst_overrun;
#endif

UART_HandleTypeDef *ULog_GetHandle(void)
{
    return &s_huart2;
}

void ULog_SetAppSnapshot(const ULogAppSnapshot *snapshot)
{
    if (snapshot != 0) s_app_snapshot = *snapshot;
}

void ULog_WriteRaw(const uint8_t *data, uint16_t len)
{
    uint16_t head, tail, used, free_space, first;
    if ((s_inited == 0u) || (s_stream_enabled == 0u) || (data == 0) || (len == 0u)) {
        return;
    }
#if defined(ULOG_TEST)
    {
        HAL_StatusTypeDef rc = HAL_UART_Transmit(&s_huart2, (uint8_t *)data, len, 30u);
        if (rc != HAL_OK) {
            g_ulog_tx_error_count++;
            s_tx_failed = 1u;
            ULog_Stop();
        }
        return;
    }
#endif
    if (len >= ULOG_TX_QUEUE_SIZE) {
        g_ulog_tx_error_count++;
        return;
    }
    /* A short critical section protects head/tail against USART2 IRQ. */
    ULOG_CRITICAL_ENTER();
    head = s_tx_head;
    tail = s_tx_tail;
    used = (uint16_t)((head >= tail) ? (uint16_t)(head - tail) :
                      (uint16_t)(ULOG_TX_QUEUE_SIZE - tail + head));
    free_space = (uint16_t)(ULOG_TX_QUEUE_SIZE - used - 1u);
    if (len > free_space) {
        ULOG_CRITICAL_EXIT();
        g_ulog_tx_error_count++;
        return;
    }
    first = (uint16_t)(ULOG_TX_QUEUE_SIZE - head);
    if (first > len) first = len;
    memcpy(&s_tx_queue[head], data, first);
    if (len > first) memcpy(&s_tx_queue[0], data + first, (size_t)(len - first));
    s_tx_head = (uint16_t)((head + len) % ULOG_TX_QUEUE_SIZE);
    ULOG_CRITICAL_EXIT();

    /* Start the first queued segment. HAL_UART_Transmit_IT is non-blocking. */
    if (s_tx_busy == 0u) {
#if defined(ULOG_TEST)
        /* Host tests provide only the blocking HAL stub. */
        (void)HAL_UART_Transmit(&s_huart2, (uint8_t *)data, len, 0u);
#else
        ULog_TxCpltCallback();
#endif
    }
}

void ULog_TxCpltCallback(void)
{
    uint16_t tail, head, count;
    HAL_StatusTypeDef rc;
    if (s_inited == 0u) return;
    ULOG_CRITICAL_ENTER();
    tail = s_tx_tail;
    head = s_tx_head;
    if (s_tx_busy != 0u) {
        s_tx_tail = (uint16_t)((tail + s_tx_inflight) % ULOG_TX_QUEUE_SIZE);
        tail = s_tx_tail;
        s_tx_inflight = 0u;
    }
    s_tx_busy = 0u;
    head = s_tx_head;
    if (tail == head) {
        ULOG_CRITICAL_EXIT();
        return;
    }
    count = (head > tail) ? (uint16_t)(head - tail) :
            (uint16_t)(ULOG_TX_QUEUE_SIZE - tail);
    s_tx_inflight = count;
    s_tx_busy = 1u;
    ULOG_CRITICAL_EXIT();
#if defined(ULOG_TEST)
    (void)rc;
#else
    rc = HAL_UART_Transmit_IT(&s_huart2, &s_tx_queue[tail], count);
    if (rc != HAL_OK) {
        ULOG_CRITICAL_ENTER();
        s_tx_busy = 0u;
        s_tx_tail = (uint16_t)((tail + count) % ULOG_TX_QUEUE_SIZE);
        ULOG_CRITICAL_EXIT();
        g_ulog_tx_error_count++;
    }
#endif
}

void ULog_Stop(void)
{
    /* This stops transmission by the STM32 only. The recorder may still
     * have cached writes or an open FAT32 file; do not immediately power off. */
    if ((s_stream_enabled != 0u) && (ULog_PidBurstActive() != 0u)) {
        static const uint8_t pid_end_frame[] = "#BTLOG,PID_END\r\n";
        ULog_PidBurstStop();
        ULog_WriteRaw(pid_end_frame, (uint16_t)(sizeof(pid_end_frame) - 1u));
    }
    if ((s_stream_enabled != 0u) && (s_data_sent != 0u) &&
        (s_tx_failed == 0u) && (s_end_sent == 0u)) {
        static const uint8_t end_frame[] = "#BTLOG,END\r\n";
        s_end_sent = 1u;
        ULog_WriteRaw(end_frame, (uint16_t)(sizeof(end_frame) - 1u));
    }
    s_stream_enabled = 0u;
}

void ULog_PidBurstStart(void)
{
#if (ULOG_PID_BURST_ENABLE != 0)
    if (!ULog_IsRecording()) return;
    ULOG_CRITICAL_ENTER();
    s_pid_head = 0u;
    s_pid_tail = 0u;
    s_pid_active = 1u;
    s_pid_end_pending = 0u;
    g_ulog_pid_burst_overrun = 0u;
    ULOG_CRITICAL_EXIT();
    ULog_Printf("#BTLOG,PID_BEGIN,timestamp_ms,fl_target,fl_actual,fl_output,fr_target,fr_actual,fr_output,rl_target,rl_actual,rl_output,rr_target,rr_actual,rr_output\r\n");
#endif
}

void ULog_PidBurstStop(void)
{
#if (ULOG_PID_BURST_ENABLE != 0)
    ULOG_CRITICAL_ENTER();
    s_pid_active = 0u;
    s_pid_end_pending = 1u;
    ULOG_CRITICAL_EXIT();
#endif
}

uint8_t ULog_PidBurstActive(void)
{
#if (ULOG_PID_BURST_ENABLE != 0)
    return s_pid_active;
#else
    return 0u;
#endif
}

void ULog_PidBurstSampleIsr(const ULogPidSample *sample)
{
#if (ULOG_PID_BURST_ENABLE != 0)
    uint16_t next;
    if ((sample == 0) || (s_pid_active == 0u)) return;
    next = (uint16_t)((s_pid_head + 1u) % ULOG_PID_BURST_CAPACITY);
    if (next == s_pid_tail) {
        g_ulog_pid_burst_overrun++;
        return;
    }
    s_pid_ring[s_pid_head] = *sample;
    s_pid_head = next;
#else
    (void)sample;
#endif
}

void ULog_PidBurstPoll(void)
{
#if (ULOG_PID_BURST_ENABLE != 0)
    static char line[256];
    ULogPidSample sample;
    uint16_t tail;
    int n;

    if (!ULog_IsRecording()) return;
    ULOG_CRITICAL_ENTER();
    if (s_pid_tail == s_pid_head) {
        if ((s_pid_active == 0u) && (s_pid_end_pending != 0u)) {
            static const uint8_t end_frame[] = "#BTLOG,PID_END\r\n";
            s_pid_end_pending = 0u;
            ULOG_CRITICAL_EXIT();
            ULog_WriteRaw(end_frame, (uint16_t)(sizeof(end_frame) - 1u));
            return;
        }
        ULOG_CRITICAL_EXIT();
        return;
    }
    tail = s_pid_tail;
    sample = s_pid_ring[tail];
    s_pid_tail = (uint16_t)((tail + 1u) % ULOG_PID_BURST_CAPACITY);
    ULOG_CRITICAL_EXIT();

    n = snprintf(line, sizeof(line),
        "#BTLOG,PID,%lu,%.4f,%.4f,%.3f,%.4f,%.4f,%.3f,%.4f,%.4f,%.3f,%.4f,%.4f,%.3f\r\n",
        (unsigned long)sample.timestamp_ms,
        sample.fl_target, sample.fl_actual, sample.fl_output,
        sample.fr_target, sample.fr_actual, sample.fr_output,
        sample.rl_target, sample.rl_actual, sample.rl_output,
        sample.rr_target, sample.rr_actual, sample.rr_output);
    if ((n > 0) && ((size_t)n < sizeof(line))) {
        ULog_WriteRaw((const uint8_t *)line, (uint16_t)n);
    } else {
        g_ulog_format_error_count++;
    }
#else
    /* Compile-time no-op keeps the application call site unchanged. */
#endif
}

uint8_t ULog_IsRecording(void)
{
    return s_inited && s_stream_enabled;
}

void ULog_Printf(const char *s)
{
    if (s == 0) {
        return;
    }
    ULog_WriteRaw((const uint8_t *)s, (uint16_t)strlen(s));
}

void ULog_Init(void)
{
    GPIO_InitTypeDef g = {0};
    if (s_inited != 0u) return; /* Never rearm a stopped log. */

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    g.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &g);

    s_huart2.Instance          = USART2;
    s_huart2.Init.BaudRate     = 115200;
    s_huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart2.Init.StopBits     = UART_STOPBITS_1;
    s_huart2.Init.Parity       = UART_PARITY_NONE;
    s_huart2.Init.Mode         = UART_MODE_TX_RX;
    s_huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&s_huart2) != HAL_OK) {
        s_inited = 0u;
        return;
    }

#if !defined(ULOG_TEST)
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif

    s_inited = 1u;
    s_stream_enabled = 1u;
    s_end_sent = 0u;
    s_data_sent = 0u;
    s_tx_failed = 0u;
    s_init_ms = HAL_GetTick();
    s_last_ms = s_init_ms;
    g_ulog_tx_error_count = 0u;
    g_ulog_format_error_count = 0u;
    g_ulog_last_line_len = 0u;
    s_odo = 0.0f;
    s_have_last = 0u;
    s_tx_head = 0u;
    s_tx_tail = 0u;
    s_tx_inflight = 0u;
    s_tx_busy = 0u;
#if (ULOG_PID_BURST_ENABLE != 0)
    s_pid_head = 0u;
    s_pid_tail = 0u;
    s_pid_active = 0u;
    s_pid_end_pending = 0u;
    g_ulog_pid_burst_overrun = 0u;
#endif

    /* Do not send the old #U43 marker immediately at boot: the external
     * recorder may not be initialized yet. Log data starts after delay. */
}

static float ulog_wrap180(float deg)
{
    /* Bounded even for corrupt/non-finite feedback during a fault. */
    deg = fmodf(deg, 360.0f);
    if (deg > 180.0f) deg -= 360.0f;
    if (deg < -180.0f) deg += 360.0f;
    return deg;
}

static uint8_t ulog_finished(void)
{
    return s_app_snapshot.finished;
}

void ULog_Poll(const MecanumChassis_t *chassis)
{
    /* MUST be static: main stack is only 1KB; local buf[640] caused HardFault. */
    static char buf[768];
    static float ch[43];
    int n;
    uint32_t now;
    float yaw_deg, imu_deg, eyaw;
    float ex, ey, elat;
    float vx, vy, v_mps, v_act_mps;
    float yrate_dps;
    uint8_t i;
    uint8_t phase;
    uint8_t finished;

    if (!ULog_IsRecording()) return;
    finished = ulog_finished();
    if ((chassis == 0) ||
        (chassis->motor_lf == 0) || (chassis->motor_rf == 0) ||
        (chassis->motor_lb == 0) || (chassis->motor_rb == 0)) {
        if (finished) ULog_Stop();
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_init_ms) < ULOG_STARTUP_WAIT_MS) {
        if (finished) ULog_Stop(); /* Recorder may not be ready yet. */
        return;
    }
    if (!finished && (uint32_t)(now - s_last_ms) < ULOG_PERIOD_MS) {
        return;
    }
    s_last_ms = now;

    if (s_have_last != 0u) {
        float dx = chassis->x_pos - s_last_x;
        float dy = chassis->y_pos - s_last_y;
        s_odo += sqrtf(dx * dx + dy * dy);
    }
    s_last_x = chassis->x_pos;
    s_last_y = chassis->y_pos;
    s_have_last = 1u;

    yaw_deg = chassis->theta_pos * (180.0f / (float)M_PI);
    imu_deg = s_app_snapshot.imu_yaw_deg;
    ex = chassis->x_target - chassis->x_pos;
    ey = chassis->y_target - chassis->y_pos;
    eyaw = ulog_wrap180(chassis->theta_target * (180.0f / (float)M_PI) - yaw_deg);

    /* body-frame lateral error */
    {
        float c = cosf(chassis->theta_pos);
        float s = sinf(chassis->theta_pos);
        elat = -ex * s + ey * c;
    }

    vx = chassis->vx_target;
    vy = chassis->vy_target;
    v_mps = sqrtf(vx * vx + vy * vy);
    yrate_dps = chassis->omega_target * (180.0f / (float)M_PI);

    v_act_mps = 0.25f * (
        fabsf(chassis->motor_lf->actual_speed) +
        fabsf(chassis->motor_rf->actual_speed) +
        fabsf(chassis->motor_lb->actual_speed) +
        fabsf(chassis->motor_rb->actual_speed)) / ULOG_PULSE_PER_METER;

    phase = s_app_snapshot.phase;

    for (i = 0; i < 43u; i++) {
        ch[i] = 0.0f;
    }

    ch[0]  = (float)chassis->mode;
    ch[1]  = (float)phase;
    ch[2]  = (float)s_app_snapshot.status;
    ch[3]  = (float)s_app_snapshot.action_index;
    ch[6]  = (float)s_app_snapshot.error;
    ch[7]  = (float)s_app_snapshot.plan_index;
    ch[9]  = (float)s_app_snapshot.plan_done;
    ch[4]  = (float)chassis->target_reached;
    ch[5]  = (float)now;                 /* ms */
    ch[8]  = (float)phase;
    ch[10] = chassis->vx_actual;        /* measured body vx, m/s */
    ch[11] = chassis->vy_actual;        /* measured body vy, m/s */
    ch[12] = chassis->vx_target;        /* target body vx, m/s */
    ch[13] = chassis->vy_target;        /* target body vy, m/s */
    ch[14] = chassis->omega_target;      /* st_tgt-like */
    ch[15] = chassis->omega_actual;
    ch[16] = yaw_deg;
    ch[17] = imu_deg;
    ch[18] = yrate_dps;
    ch[19] = v_mps / ULOG_PV;            /* viewer * PV => m/s */
    ch[20] = v_act_mps / ULOG_PV;
    ch[21] = chassis->x_target;
    ch[22] = chassis->y_target;
    ch[23] = chassis->theta_target * (180.0f / (float)M_PI);
    ch[24] = ex;
    ch[25] = ey;
    ch[26] = eyaw;
    ch[27] = chassis->motor_lf->target_speed;
    ch[28] = chassis->motor_lf->actual_speed;
    ch[29] = chassis->motor_rf->actual_speed;
    ch[30] = chassis->motor_lb->actual_speed;
    ch[31] = chassis->motor_rb->actual_speed;
    ch[32] = chassis->x_pos;             /* ox */
    ch[33] = chassis->y_pos;             /* oy */
    ch[34] = s_odo;                      /* odo */
    ch[35] = chassis->motor_rf->target_speed;
    ch[36] = chassis->motor_lb->target_speed;
    ch[37] = chassis->motor_rb->target_speed;
    ch[38] = fmaxf(fmaxf(fabsf(chassis->motor_lf->output),
                         fabsf(chassis->motor_rf->output)),
                   fmaxf(fabsf(chassis->motor_lb->output),
                         fabsf(chassis->motor_rb->output)));
    ch[39] = elat;
    ch[40] = Vbat_GetDriverVoltage();   /* D36A/driver VIN, V */
    ch[41] = Vbat_GetBoardVoltage();    /* board VIN, V */
    ch[42] = eyaw;                       /* herr */

    for (i = 0; i < 43u; i++) {
        if (!isfinite(ch[i])) {
            g_ulog_format_error_count++;
            ULog_Stop();
            return;
        }
    }

    n = snprintf(
        buf, sizeof(buf),
        "%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,"
        "%.0f,%.0f,%.0f,%.0f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,"
        "%.3f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.1f,%.1f,%.1f,"
        "%.1f,%.1f,%.4f,%.4f,%.4f,%.1f,%.1f,%.0f,%.0f,%.4f,"
        "%.0f,%.3f,%.2f\r\n",
        ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7], ch[8], ch[9],
        ch[10], ch[11], ch[12], ch[13], ch[14], ch[15], ch[16], ch[17], ch[18], ch[19],
        ch[20], ch[21], ch[22], ch[23], ch[24], ch[25], ch[26], ch[27], ch[28], ch[29],
        ch[30], ch[31], ch[32], ch[33], ch[34], ch[35], ch[36], ch[37], ch[38], ch[39],
        ch[40], ch[41], ch[42]);

    if (n <= 0 || (size_t)n >= sizeof(buf)) {
        /* Never transmit a truncated CSV row or read past buf. */
        g_ulog_format_error_count++;
        if (finished) ULog_Stop();
        return;
    }
    g_ulog_last_line_len = (uint16_t)n;
    s_data_sent = 1u;
    ULog_WriteRaw((const uint8_t *)buf, (uint16_t)n);

    /* Write one final status row, then stop sending indefinitely.
     * Actual file sync/close is the external recorder's responsibility. */
    if (finished) {
        ULog_Stop();
    }
}

#else /* ULOG_ENABLE == 0 */
volatile uint32_t g_ulog_tx_error_count;
volatile uint32_t g_ulog_format_error_count;
volatile uint16_t g_ulog_last_line_len;

void ULog_Init(void) {}
void ULog_Poll(const MecanumChassis_t *chassis) { (void)chassis; }
void ULog_SetAppSnapshot(const ULogAppSnapshot *snapshot) { (void)snapshot; }
void ULog_Stop(void) {}
uint8_t ULog_IsRecording(void) { return 0u; }
void ULog_WriteRaw(const uint8_t *data, uint16_t len) { (void)data; (void)len; }
void ULog_Printf(const char *s) { (void)s; }
UART_HandleTypeDef *ULog_GetHandle(void) { return 0; }
void ULog_TxCpltCallback(void) {}
void ULog_PidBurstStart(void) {}
void ULog_PidBurstStop(void) {}
uint8_t ULog_PidBurstActive(void) { return 0u; }
void ULog_PidBurstSampleIsr(const ULogPidSample *sample) { (void)sample; }
void ULog_PidBurstPoll(void) {}
volatile uint32_t g_ulog_pid_burst_overrun;

#endif
