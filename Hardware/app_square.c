/**
 * @file    app_square.c
 * @brief   Trajectory:
 *          1) translate RIGHT 0.20 m
 *          2) clockwise square side 1.80 m
 *          3) return to START (0,0), yaw 0
 *
 * World frame: +X forward, -Y right, +Y left (same as corner table).
 */
#include "app_square.h"
#include "vofa_debug.h"
#include "bsp_ulog.h"
#include "app_imu.h"
#include "usart.h"
#include "main.h"

#include <math.h>
#include <stdio.h>

#define SQ_HALF_PI      1.57079633f
#define SQ_TICK_MS      20u

#define PH_MOVE         0u
#define PH_TURN         1u
#define PH_GUIWEI       2u

/* Waypoint kinds */
#define WP_MOVE         0u
#define WP_TURN         1u

/*
 * Absolute waypoints (world frame).
 * Start pose after reset: (0,0,0)
 * PRE  : (0, -0.20, 0)                 right 20cm
 * SQ   : 1.8m clockwise about that origin
 * HOME : (0, 0, 0)                     back to start
 */
typedef struct {
    uint8_t kind;   /* WP_MOVE / WP_TURN */
    float x;
    float y;
    float yaw;      /* target yaw rad */
} SqWp_t;

static const SqWp_t s_wps[] = {
    /* 0 PRE right */
    { WP_MOVE, 0.0f,                 -SQUARE_PRE_RIGHT_M,                 0.0f },
    /* 1-8 square (origin at pre point) */
    { WP_MOVE, SQUARE_SIDE_M,        -SQUARE_PRE_RIGHT_M,                 0.0f },
    { WP_TURN, SQUARE_SIDE_M,        -SQUARE_PRE_RIGHT_M,                -SQ_HALF_PI },
    { WP_MOVE, SQUARE_SIDE_M,        -SQUARE_PRE_RIGHT_M - SQUARE_SIDE_M,-SQ_HALF_PI },
    { WP_TURN, SQUARE_SIDE_M,        -SQUARE_PRE_RIGHT_M - SQUARE_SIDE_M,-2.0f * SQ_HALF_PI },
    { WP_MOVE, 0.0f,                 -SQUARE_PRE_RIGHT_M - SQUARE_SIDE_M,-2.0f * SQ_HALF_PI },
    { WP_TURN, 0.0f,                 -SQUARE_PRE_RIGHT_M - SQUARE_SIDE_M, SQ_HALF_PI },
    { WP_MOVE, 0.0f,                 -SQUARE_PRE_RIGHT_M,                 SQ_HALF_PI },
    { WP_TURN, 0.0f,                 -SQUARE_PRE_RIGHT_M,                 0.0f },
    /* 9 HOME */
    { WP_MOVE, 0.0f,                  0.0f,                               0.0f },
};

#define SQ_WP_COUNT     (sizeof(s_wps) / sizeof(s_wps[0]))

static uint8_t  s_state = SQUARE_IDLE;
static uint8_t  s_wp;            /* 0..SQ_WP_COUNT-1 */
static uint8_t  s_phase;
static uint32_t s_t0;
static uint32_t s_last_ms;
static uint8_t  s_started;
static uint32_t s_tstart;

static float    s_kp_save;
static float    s_ki_save;
static float    s_kd_save;

static float sq_wrap_pi(float a)
{
    while (a >  3.14159265f) { a -= 6.28318531f; }
    while (a < -3.14159265f) { a += 6.28318531f; }
    return a;
}

static void sq_enter_wp(MecanumChassis_t *ch)
{
    const SqWp_t *wp;

    if (s_wp >= SQ_WP_COUNT) {
        return;
    }
    wp = &s_wps[s_wp];
    s_t0 = HAL_GetTick();

    if (wp->kind == WP_TURN) {
        /* Hold current xy (near planned), command yaw */
        Mecanum_SetPositionMode(ch, ch->x_pos, ch->y_pos, sq_wrap_pi(wp->yaw));
        s_phase = PH_TURN;
    } else {
        Mecanum_SetPositionMode(ch, wp->x, wp->y, sq_wrap_pi(wp->yaw));
        s_phase = PH_MOVE;
    }
}

static void sq_hold_wp(MecanumChassis_t *ch)
{
    const SqWp_t *wp = &s_wps[s_wp];
    if (wp->kind == WP_TURN) {
        Mecanum_SetPositionMode(ch, ch->x_pos, ch->y_pos, sq_wrap_pi(wp->yaw));
    } else {
        Mecanum_SetPositionMode(ch, wp->x, wp->y, sq_wrap_pi(wp->yaw));
    }
}

void Square_Start(MecanumChassis_t *chassis)
{
    if (chassis == NULL) {
        return;
    }

    Mecanum_Enable(chassis, 1u);
    Mecanum_Stop(chassis);
    Mecanum_ResetPosition(chassis);

    Mecanum_SetTolerance(chassis, SQUARE_TOL_M, SQUARE_TOL_RAD);
    Mecanum_SetSpeedLimit(chassis, SQUARE_VMAX, SQUARE_WMAX);

    s_state   = SQUARE_RUNNING;
    s_wp      = 0u;
    s_phase   = PH_MOVE;
    s_last_ms = HAL_GetTick();
    s_tstart  = s_last_ms;
    s_started = 0u;
}

void Square_Abort(MecanumChassis_t *chassis)
{
    if (chassis != NULL) {
        chassis->pid_theta.kp = s_kp_save;
        chassis->pid_theta.ki = s_ki_save;
        chassis->pid_theta.kd = s_kd_save;
        Mecanum_Stop(chassis);
    }
    s_state = SQUARE_IDLE;
}

void Square_Poll(MecanumChassis_t *chassis)
{
    uint32_t now;

    if ((chassis == NULL) || (s_state != SQUARE_RUNNING)) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_ms) < SQ_TICK_MS) {
        return;
    }
    s_last_ms = now;

    if (!s_started) {
        if ((uint32_t)(now - s_tstart) < SQUARE_START_DELAY_MS) {
            return;
        }
        s_started = 1u;
        sq_enter_wp(chassis);
        return;
    }

    if ((uint32_t)(now - s_t0) > SQUARE_STEP_TIMEOUT_MS) {
        s_state = SQUARE_TIMEOUT;
        Mecanum_Stop(chassis);
        return;
    }

    switch (s_phase) {
    case PH_MOVE:
    case PH_TURN:
        if (Mecanum_IsTargetReached(chassis)) {
            s_kp_save = chassis->pid_theta.kp;
            s_ki_save = chassis->pid_theta.ki;
            s_kd_save = chassis->pid_theta.kd;

            chassis->pid_theta.kp = s_kp_save * SQUARE_GUIWEI_KP_SCALE;
            chassis->pid_theta.ki = 0.0f;
            chassis->pid_theta.kd = 0.0f;

            sq_hold_wp(chassis);
            s_phase = PH_GUIWEI;
            s_t0 = now;
        }
        break;

    case PH_GUIWEI:
    default:
    {
        const SqWp_t *wp = &s_wps[s_wp];
        uint8_t reached = Mecanum_IsTargetReached(chassis);
        uint8_t time_ok = (uint8_t)((uint32_t)(now - s_t0) >= SQUARE_GUIWEI_MS);
        uint8_t hard_timeout = (uint8_t)((uint32_t)(now - s_t0) >= (SQUARE_GUIWEI_MS * 3u));
        float eyaw = sq_wrap_pi(chassis->theta_target - chassis->theta_pos);
        uint8_t yaw_ok = (uint8_t)(fabsf(eyaw) < (SQUARE_TOL_RAD * 1.2f));
        uint8_t done = 0u;

        if (time_ok && reached && yaw_ok) {
            done = 1u;
        } else if (hard_timeout && yaw_ok) {
            done = 1u;
        } else if (hard_timeout && (wp->kind == WP_MOVE) && reached) {
            done = 1u;
        } else if (hard_timeout && (wp->kind == WP_TURN) && (fabsf(eyaw) < 0.05f)) {
            done = 1u;
        }

        if (done) {
            chassis->pid_theta.kp = s_kp_save;
            chassis->pid_theta.ki = s_ki_save;
            chassis->pid_theta.kd = s_kd_save;

            s_wp++;
            if (s_wp >= SQ_WP_COUNT) {
                s_state = SQUARE_DONE;
                Mecanum_Stop(chassis);
                return;
            }
            sq_enter_wp(chassis);
        }
    }
        break;
    }
}

uint8_t Square_GetState(void)
{
    return s_state;
}

/* Map to old 0..7 square index for OLED; PRE=0, HOME=8 */
uint8_t Square_GetStep(void)
{
    if (s_wp == 0u) {
        return 0u;
    }
    if (s_wp >= (SQ_WP_COUNT - 1u)) {
        return 8u;
    }
    return (uint8_t)(s_wp - 1u);
}

uint8_t Square_IsRunning(void)
{
    return (uint8_t)((s_state == SQUARE_RUNNING) ? 1u : 0u);
}

uint8_t Square_IsDone(void)
{
    return (uint8_t)((s_state == SQUARE_DONE) ? 1u : 0u);
}

/* ================= VOFA+ ���Σ�������MOTOR_TUNE=1 ʱ�ã� ================= */
volatile uint8_t g_tune_mode = 0;

#ifndef TUNE_STEP
#define TUNE_STEP       300.0f
#endif
#ifndef TUNE_HIGH_MS
#define TUNE_HIGH_MS    3000u
#endif
#ifndef TUNE_LOW_MS
#define TUNE_LOW_MS     3000u
#endif
#ifndef TUNE_WAIT_MS
#define TUNE_WAIT_MS    1000u
#endif
#ifndef TUNE_SEND_MS
#define TUNE_SEND_MS    10u
#endif

static UART_HandleTypeDef s_huart1;

static Motor_t  *s_tm;
static uint32_t  s_tm0;
static uint8_t   s_tmph;
static uint32_t  s_tmsend;

void VOFA_Init(void)
{
    GPIO_InitTypeDef g = {0};

    /* PD5/PD6 USART2 owned by ULog only (TF logger); VOFA uses USART1 */
    ULog_Init();

    /* USART1: PA9=TX / PA10=RX (board Type-C serial1) */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    g.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);

    s_huart1.Instance          = USART1;
    s_huart1.Init.BaudRate     = 115200;
    s_huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart1.Init.StopBits     = UART_STOPBITS_1;
    s_huart1.Init.Parity       = UART_PARITY_NONE;
    s_huart1.Init.Mode         = UART_MODE_TX_RX;
    s_huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_huart1);
}

void VOFA_PlotMotor(const Motor_t *m)
{
    char buf[48];
    int  n;
    /* VOFA motor tuning must never pollute USART2 TF logging stream. */
    n = snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f\n",
                 m->target_speed, m->actual_speed, m->output);
    if (n > 0 && (size_t)n < sizeof(buf)) {
        (void)HAL_UART_Transmit(&s_huart1, (uint8_t *)buf, (uint16_t)n, 20);
    }
}


void Tune_Start(Motor_t *m)
{
    s_tm     = m;
    s_tm0    = HAL_GetTick();
    s_tmph   = 0u;
    s_tmsend = s_tm0;
    g_tune_mode = 1u;
    Motor_SetTargetSpeed(m, 0.0f);
}

void Tune_Poll(void)
{
    uint32_t now = HAL_GetTick();

    if (s_tm == NULL) {
        return;
    }

    switch (s_tmph) {
    case 0u:
        Motor_SetTargetSpeed(s_tm, 0.0f);
        if ((uint32_t)(now - s_tm0) >= TUNE_WAIT_MS) {
            s_tmph = 1u;
            s_tm0  = now;
        }
        break;

    case 1u:
        Motor_SetTargetSpeed(s_tm, TUNE_STEP);
        if ((uint32_t)(now - s_tm0) >= TUNE_HIGH_MS) {
            s_tmph = 2u;
            s_tm0  = now;
        }
        break;

    default:
        Motor_SetTargetSpeed(s_tm, 0.0f);
        if ((uint32_t)(now - s_tm0) >= TUNE_LOW_MS) {
            s_tmph = 1u;
            s_tm0  = now;
        }
        break;
    }

    if ((uint32_t)(now - s_tmsend) >= TUNE_SEND_MS) {
        s_tmsend = now;
        VOFA_PlotMotor(s_tm);
    }
}
