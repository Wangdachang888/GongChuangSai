#include "board.h"
#include "vision.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart3 = {USART3};
static uint32_t s_irq;
static uint32_t s_now;
static MecanumChassis_t s_chassis;
static char s_last_tx[192];
static size_t s_last_tx_length;

uint32_t __get_PRIMASK(void) { return s_irq; }
void __disable_irq(void) { s_irq = 1U; }
void __set_PRIMASK(uint32_t mask) { s_irq = mask; }
uint32_t HAL_GetTick(void) { return s_now; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length)
{ (void)uart; (void)byte; (void)length; return HAL_OK; }
int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length)
{ (void)uart; (void)byte; (void)length; return HAL_OK; }

void Mecanum_Stop(MecanumChassis_t *chassis) { chassis->mode = 0U; }
void Mecanum_SetPositionMode(MecanumChassis_t *chassis, float x, float y, float yaw)
{
    chassis->x_target = x;
    chassis->y_target = y;
    chassis->theta_target = yaw;
    chassis->mode = 1U;
    chassis->target_reached = 0U;
}
void Mecanum_GetPosition(MecanumChassis_t *chassis, float *x, float *y, float *theta)
{ *x = chassis->x_pos; *y = chassis->y_pos; *theta = chassis->theta_pos; }
void Mecanum_GetVelocity(MecanumChassis_t *chassis, float *vx, float *vy, float *omega)
{ *vx = chassis->vx_actual; *vy = chassis->vy_actual; *omega = chassis->omega_actual; }
uint8_t Mecanum_IsTargetReached(MecanumChassis_t *chassis)
{ return chassis->target_reached; }

static void tx_capture(const uint8_t *data, size_t length, void *user)
{
    (void)user;
    assert(length < sizeof(s_last_tx));
    memcpy(s_last_tx, data, length);
    s_last_tx[length] = '\0';
    s_last_tx_length = length;
}

static unsigned char checksum(const char *text)
{
    unsigned char result = 0U;
    while (*text != '\0') result ^= (unsigned char)*text++;
    return result;
}

static void receive_frame(VisionContext *vision, unsigned sequence, const char *body)
{
    char header[160];
    char frame[192];
    int frame_length;
    unsigned index;
    snprintf(header, sizeof(header), "%u,%u,%s", sequence,
             (unsigned)strlen(body), body);
    frame_length = snprintf(frame, sizeof(frame), "@%s*%02X\n", header,
                             (unsigned)checksum(header));
    for (index = 0U; index < (unsigned)frame_length; index++)
        Vision_RxByte(vision, (uint8_t)frame[index], s_now);
    Vision_Poll(vision, s_now);
}

static void receive_position(VisionContext *vision, unsigned sequence,
                             unsigned step, int dx, int dy)
{
    char body[96];
    snprintf(body, sizeof(body), "POS,RING,2,%d,%d,160,120,1,%u", dx, dy, step);
    receive_frame(vision, sequence, body);
}

int main(void)
{
    VisionContext vision;
    memset(&s_chassis, 0, sizeof(s_chassis));
    Vision_Init(&vision, tx_capture, 0, 0);
    vision.ready = 1U;
    Vision_SetRingChassis(&vision, &s_chassis);
    Vision_SetRingCalibration(&vision, 100.0f, 0.0f, 0.0f, 100.0f,
                              1.0f, 0.10f, 0.003f, 0.12f, 0.35f);
    assert(Vision_StartRingAlign(&vision, 2U, 0U));
    assert(strstr(s_last_tx, "ALIGN,2,1") != 0);
    s_now = 1U;
    receive_frame(&vision, 1U, "ACK,ALIGN,2,1");
    assert(Vision_GetRingState(&vision) == VISION_RING_WAIT_POS);
    s_now = 2U;
    receive_position(&vision, 2U, 0U, 20, 0);
    assert(Vision_GetRingState(&vision) == VISION_RING_MOVING);
    assert(s_chassis.mode == 1U && s_chassis.x_target < 0.0f);
    receive_position(&vision, 2U, 0U, 20, 0);
    assert(Vision_GetRingState(&vision) == VISION_RING_MOVING);
    s_chassis.target_reached = 1U;
    s_now = 300U;
    Vision_Poll(&vision, s_now);
    s_now = 500U;
    Vision_Poll(&vision, s_now);
    assert(strstr(s_last_tx, "STEP_DONE,0,1") != 0);
    assert(Vision_GetRingState(&vision) == VISION_RING_WAIT_POS);
    s_now = 600U;
    receive_position(&vision, 3U, 1U, 3, -2);
    receive_position(&vision, 4U, 2U, 3, -2);
    receive_position(&vision, 5U, 3U, 3, -2);
    receive_position(&vision, 6U, 4U, 3, -2);
    assert(Vision_GetRingState(&vision) == VISION_RING_WAIT_IDLE);
    receive_frame(&vision, 7U, "IDLE");
    assert(Vision_RingSucceeded(&vision));
    puts("PASS: ring ACK/session, duplicate step rejection, calibrated move, stable STEP_DONE, ARRIVED/IDLE");
    return 0;
}
