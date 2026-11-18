/**
  ******************************************************************************
  * @file    bsp_d36_cmd.c
  * @brief   USART3 command parser -> AppStepper open-loop API
  ******************************************************************************
  */
#include "bsp_d36_cmd.h"
#include "bsp_stepper.h"
#include "app_mode.h"
#include "stepper_config.h"
#include "usart.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#if (APP_D36_UART_CMD_ENABLE != 0) && (APP_RUN_MODE != APP_MODE_RACE)

#define D36_CMD_RX_SIZE     256U
#define D36_CMD_LINE_SIZE    96U

static volatile uint8_t  s_rx_buf[D36_CMD_RX_SIZE];
static volatile uint16_t s_rx_wr;
static volatile uint16_t s_rx_rd;
static uint8_t           s_rx_byte;
static char              s_line[D36_CMD_LINE_SIZE];
static uint16_t          s_line_len;
static uint8_t           s_inited;
static D36CmdStepperOps s_stepper_ops;
static void             *s_stepper_user;

void D36Cmd_SetStepperOps(const D36CmdStepperOps *ops, void *user)
{
    if (ops == 0) {
        memset(&s_stepper_ops, 0, sizeof(s_stepper_ops));
        s_stepper_user = 0;
    } else {
        s_stepper_ops = *ops;
        s_stepper_user = user;
    }
}

static void D36Cmd_SendRaw(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0U))
    {
        return;
    }
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 100);
}

void D36Cmd_Printf(const char *s)
{
    if (s == 0)
    {
        return;
    }
    D36Cmd_SendRaw((const uint8_t *)s, (uint16_t)strlen(s));
}

static void D36Cmd_Help(void)
{
    char buf[80];
    /* Keep short: blocking UART must not stall square/IMU for tens of ms */
    D36Cmd_Printf("D36: M1 <steps> [f] | V1 <+1|-1|0> [f]\r\n");
    D36Cmd_Printf("     E1/E 1|0 | S | X/STOP | H\r\n");
    (void)sprintf(buf, "def f=%uHz ppr=%u\r\n",
                  (unsigned)APP_STEPPER_DEFAULT_FREQ_HZ,
                  (unsigned)APP_STEPPER_PULSES_PER_REV);
    D36Cmd_Printf(buf);
}

static void D36Cmd_Status(void)
{
    char buf[128];
    (void)sprintf(buf,
                   "LIFT en=%u busy=%u remain=%lu\r\n",
                   (unsigned)Stepper_GetEnable(STEPPER_AXIS_1),
                   (unsigned)((s_stepper_ops.is_busy != 0) &&
                              s_stepper_ops.is_busy(1U, s_stepper_user)),
                   (unsigned long)((s_stepper_ops.get_remaining_steps != 0) ?
                       s_stepper_ops.get_remaining_steps(1U, s_stepper_user) : 0U));
    D36Cmd_Printf(buf);
}

static void D36Cmd_ToUpper(char *s)
{
    while (*s != '\0')
    {
        if ((*s >= 'a') && (*s <= 'z'))
        {
            *s = (char)(*s - 'a' + 'A');
        }
        s++;
    }
}

static int D36Cmd_ParseInt(const char *s, int32_t *out)
{
    char *end = 0;
    long v;
    if ((s == 0) || (out == 0))
    {
        return 0;
    }
    v = strtol(s, &end, 10);
    if (end == s)
    {
        return 0;
    }
    *out = (int32_t)v;
    return 1;
}

static const char *D36Cmd_SkipSpaces(const char *s)
{
    while ((*s == ' ') || (*s == '\t'))
    {
        s++;
    }
    return s;
}

static const char *D36Cmd_NextToken(const char *s)
{
    s = D36Cmd_SkipSpaces(s);
    while ((*s != '\0') && (*s != ' ') && (*s != '\t'))
    {
        s++;
    }
    return D36Cmd_SkipSpaces(s);
}

static void D36Cmd_HandleLine(char *line)
{
    const char *p;
    int32_t a;
    int32_t freq_i;
    uint32_t freq;
    HAL_StatusTypeDef st;

    D36Cmd_ToUpper(line);
    p = D36Cmd_SkipSpaces(line);
    if (*p == '\0')
    {
        return;
    }

    freq = APP_STEPPER_DEFAULT_FREQ_HZ;

    if (((p[0] == 'H') && (p[1] == '\0')) || (p[0] == '?'))
    {
        D36Cmd_Help();
        return;
    }

    if ((p[0] == 'S') && (p[1] == '\0'))
    {
        D36Cmd_Status();
        return;
    }

    if (((p[0] == 'X') && (p[1] == '\0')) ||
        (strcmp(p, "STOP") == 0) ||
        ((p[0] == '0') && (p[1] == '\0')))
    {
        if (s_stepper_ops.stop_all != 0) s_stepper_ops.stop_all(s_stepper_user);
        D36Cmd_Printf("OK STOP\r\n");
        return;
    }

    if ((p[0] == '1') && (p[1] == '\0'))
    {
        st = (s_stepper_ops.start_continuous != 0) ?
             s_stepper_ops.start_continuous(1U, 1, freq, s_stepper_user) : HAL_ERROR;
        D36Cmd_Printf((st == HAL_OK) ? "OK CW\r\n" : "ERR CW\r\n");
        return;
    }

    if ((p[0] == '2') && (p[1] == '\0'))
    {
        st = (s_stepper_ops.start_continuous != 0) ?
             s_stepper_ops.start_continuous(1U, -1, freq, s_stepper_user) : HAL_ERROR;
        D36Cmd_Printf((st == HAL_OK) ? "OK CCW\r\n" : "ERR CCW\r\n");
        return;
    }

    if ((p[0] == 'M') && (p[1] == '1'))
    {
        const char *t1 = D36Cmd_NextToken(p + 2);
        const char *t2;
        if (D36Cmd_ParseInt(t1, &a) == 0)
        {
            D36Cmd_Printf("ERR usage: M1 <steps> [freq]\r\n");
            return;
        }
        t2 = D36Cmd_NextToken(t1);
        if (D36Cmd_ParseInt(t2, &freq_i) != 0)
        {
            if (freq_i > 0)
            {
                freq = (uint32_t)freq_i;
            }
        }
        st = (s_stepper_ops.move_steps != 0) ?
             s_stepper_ops.move_steps(1U, a, freq, s_stepper_user) : HAL_ERROR;
        D36Cmd_Printf((st == HAL_OK) ? "OK MOVE\r\n" : "ERR MOVE\r\n");
        return;
    }

    if ((p[0] == 'V') && (p[1] == '1'))
    {
        const char *t1 = D36Cmd_NextToken(p + 2);
        const char *t2;
        int8_t dir;
        if (D36Cmd_ParseInt(t1, &a) == 0)
        {
            D36Cmd_Printf("ERR usage: V1 <+1|-1|0> [freq]\r\n");
            return;
        }
        t2 = D36Cmd_NextToken(t1);
        if (D36Cmd_ParseInt(t2, &freq_i) != 0)
        {
            if (freq_i > 0)
            {
                freq = (uint32_t)freq_i;
            }
        }
        if (a == 0)
        {
            if (s_stepper_ops.stop != 0) s_stepper_ops.stop(1U, s_stepper_user);
            D36Cmd_Printf("OK STOP AXIS\r\n");
            return;
        }
        dir = (a > 0) ? 1 : -1;
        st = (s_stepper_ops.start_continuous != 0) ?
             s_stepper_ops.start_continuous(1U, dir, freq, s_stepper_user) : HAL_ERROR;
        D36Cmd_Printf((st == HAL_OK) ? "OK SPIN\r\n" : "ERR SPIN\r\n");
        return;
    }


    if ((p[0] == 'E') && ((p[1] == '1') || (p[1] == ' ') || (p[1] == '\t') || (p[1] == '\0')))
    {
        const char *t1;
        int32_t en_v;
        if (p[1] == '1')
        {
            t1 = D36Cmd_NextToken(p + 2);
            if (D36Cmd_ParseInt(t1, &en_v) == 0)
            {
                D36Cmd_Printf("ERR usage: E1 1|0\r\n");
                return;
            }
            if (s_stepper_ops.set_enable != 0)
                s_stepper_ops.set_enable(1U, (en_v != 0) ? 1U : 0U, s_stepper_user);
            D36Cmd_Printf("OK EN AXIS\r\n");
            return;
        }
        t1 = D36Cmd_NextToken(p + 1);
        if (D36Cmd_ParseInt(t1, &en_v) == 0)
        {
            D36Cmd_Printf("ERR usage: E 1|0\r\n");
            return;
        }
        if (s_stepper_ops.set_enable_all != 0)
            s_stepper_ops.set_enable_all((en_v != 0) ? 1U : 0U, s_stepper_user);
        D36Cmd_Printf("OK EN ALL\r\n");
        return;
    }

    D36Cmd_Printf("ERR unknown cmd, send H\r\n");
}


static uint8_t D36Cmd_GetByte(uint8_t *byte)
{
    uint16_t rd;
    if (byte == 0)
    {
        return 0U;
    }
    rd = s_rx_rd;
    if (rd == s_rx_wr)
    {
        return 0U;
    }
    *byte = s_rx_buf[rd];
    s_rx_rd = (uint16_t)((rd + 1U) & (D36_CMD_RX_SIZE - 1U));
    return 1U;
}

void D36Cmd_Init(void)
{
    s_rx_wr = 0U;
    s_rx_rd = 0U;
    s_line_len = 0U;
    s_inited = 1U;
    (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1U);
    D36Cmd_Printf("D36 ready. H=help\r\n");
}

void D36Cmd_Poll(void)
{
    uint8_t ch;

    if (s_inited == 0U)
    {
        return;
    }

    while (D36Cmd_GetByte(&ch) != 0U)
    {
        if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n'))
        {
            if (s_line_len > 0U)
            {
                s_line[s_line_len] = '\0';
                D36Cmd_HandleLine(s_line);
                s_line_len = 0U;
            }
        }
        else if (s_line_len < (D36_CMD_LINE_SIZE - 1U))
        {
            s_line[s_line_len++] = (char)ch;
        }
        else
        {
            s_line_len = 0U;
            D36Cmd_Printf("ERR line too long\r\n");
        }
    }
}

void D36Cmd_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != USART3)
    {
        return;
    }

    next = (uint16_t)((s_rx_wr + 1U) & (D36_CMD_RX_SIZE - 1U));
    if (next != s_rx_rd)
    {
        s_rx_buf[s_rx_wr] = s_rx_byte;
        s_rx_wr = next;
    }
    (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1U);
}

void D36Cmd_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1U);
    }
}

#else /* APP_D36_UART_CMD_ENABLE == 0 */

void D36Cmd_SetStepperOps(const D36CmdStepperOps *ops, void *user)
{ (void)ops; (void)user; }
void D36Cmd_Init(void) {}
void D36Cmd_Poll(void) {}
void D36Cmd_Printf(const char *s) { (void)s; }
void D36Cmd_RxCpltCallback(UART_HandleTypeDef *huart) { (void)huart; }
void D36Cmd_ErrorCallback(UART_HandleTypeDef *huart) { (void)huart; }

#endif

