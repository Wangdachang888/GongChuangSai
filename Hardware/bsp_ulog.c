/**
 * @file    bsp_ulog.c
 * @brief   USART2 PD5/PD6 -> microSD, Kart S3 viewer compatible 43-ch CSV
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

#include "app_square.h"
#include "app_imu.h"
#include "app_mode.h"
#include "bsp_vbat.h"
#if APP_RUN_MODE == APP_MODE_RACE
#include "app_race.h"
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static UART_HandleTypeDef s_huart2;
static uint8_t  s_inited;
static uint8_t  s_stream_enabled;
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
#if APP_RUN_MODE != APP_MODE_RACE
static uint8_t s_square_seen_running;
#endif

UART_HandleTypeDef *ULog_GetHandle(void)
{
    return &s_huart2;
}

void ULog_WriteRaw(const uint8_t *data, uint16_t len)
{
    if ((s_inited == 0u) || (s_stream_enabled == 0u) || (data == 0) || (len == 0u)) {
        return;
    }
    /* 115200 8N1: 10 serial bits per byte. 20 ms can be too short
     * for ~230+ bytes, causing a partial CSV row. Add a margin. */
    uint32_t baud = s_huart2.Init.BaudRate;
    uint32_t timeout_ms =
        ((uint32_t)len * 10000u + baud - 1u) / baud + 15u;
    HAL_StatusTypeDef rc;
    if (timeout_ms < 30u) {
        timeout_ms = 30u;
    }
    rc = HAL_UART_Transmit(&s_huart2, (uint8_t *)data, len, timeout_ms);
    if (rc != HAL_OK) {
        g_ulog_tx_error_count++;
        ULog_Stop(); /* A failed transmission may have emitted a partial row. */
    }
}

void ULog_Stop(void)
{
    /* This stops transmission by the STM32 only. The recorder may still
     * have cached writes or an open FAT32 file; do not immediately power off. */
    s_stream_enabled = 0u;
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

    s_inited = 1u;
    s_stream_enabled = 1u;
    s_init_ms = HAL_GetTick();
    s_last_ms = s_init_ms;
    g_ulog_tx_error_count = 0u;
    g_ulog_format_error_count = 0u;
    g_ulog_last_line_len = 0u;
    s_odo = 0.0f;
    s_have_last = 0u;

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

static uint8_t ulog_phase(const MecanumChassis_t *chassis)
{
#if APP_RUN_MODE == APP_MODE_RACE
    const RaceFsm *race = AppRace_Status();
    (void)chassis;
    if (AppRace_LogFinished()) return 4u;
    if (race->state != RACE_RUNNING || race->action_index >= race->action_count) return 0u;
    if (race->actions[race->action_index].kind == RACE_ACTION_TURN) return 2u;
    if (race->actions[race->action_index].kind == RACE_ACTION_SETTLE ||
        race->actions[race->action_index].kind == RACE_ACTION_SCAN) return 3u;
    return race->actions[race->action_index].kind == RACE_ACTION_MOVE ? 1u : 0u;
#else
    uint8_t st = Square_GetState();
    uint8_t step = Square_GetStep();

    if (st == 2u) {          /* SQUARE_DONE */
        return 4u;
    }
    if (st != 1u) {          /* not running */
        return 0u;
    }
    if (chassis->mode == 0u && chassis->target_reached == 0u &&
        (fabsf(chassis->x_target) < 1e-4f) && (fabsf(chassis->y_target) < 1e-4f) &&
        (fabsf(chassis->vx_target) < 1e-4f)) {
        return 0u;           /* start delay / idle */
    }
    /* square: even step = move, odd step = turn */
    return ((step & 1u) == 0u) ? 1u : 2u;
#endif
}

static uint8_t ulog_finished(void)
{
#if APP_RUN_MODE == APP_MODE_RACE
    return AppRace_LogFinished();
#else
    uint8_t state = Square_GetState();
    if (state == SQUARE_RUNNING) s_square_seen_running = 1u;
    return state == SQUARE_DONE || state == SQUARE_TIMEOUT ||
           (s_square_seen_running && state == SQUARE_IDLE);
#endif
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
    imu_deg = app_imu_yaw_deg();
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

    phase = ulog_phase(chassis);

    for (i = 0; i < 43u; i++) {
        ch[i] = 0.0f;
    }

    ch[0]  = (float)chassis->mode;
    ch[1]  = (float)phase;
#if APP_RUN_MODE == APP_MODE_RACE
    ch[2]  = (float)AppRace_Status()->state;
    ch[3]  = (float)AppRace_Status()->action_index;
    ch[6]  = (float)AppRace_Status()->error;
    ch[7]  = (float)AppRace_TaskPlan()->index;
    ch[9]  = (float)RaceTaskPlan_Done(AppRace_TaskPlan());
#else
    ch[2]  = (float)Square_GetState();
    ch[3]  = (float)Square_GetStep();
#endif
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
void ULog_Stop(void) {}
uint8_t ULog_IsRecording(void) { return 0u; }
void ULog_WriteRaw(const uint8_t *data, uint16_t len) { (void)data; (void)len; }
void ULog_Printf(const char *s) { (void)s; }
UART_HandleTypeDef *ULog_GetHandle(void) { return 0; }

#endif
