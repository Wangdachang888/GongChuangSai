#include "usart.h"
#include "vision.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart3 = {USART3};
static uint8_t *s_destination;
static int s_receive_error, s_transmit_error;
static uint32_t s_irq;
static uint8_t s_command;
static uint8_t s_tx_data[192];
static uint16_t s_tx_length;
static uint32_t s_now;
static void local_tx(const uint8_t *data, size_t length, void *user)
{ (void)data; (void)length; (void)user; }

static void local_frame(VisionContext *vision, uint16_t sequence,
                        const char *body)
{
    char header[192];
    char frame[224];
    unsigned char checksum = 0U;
    size_t index;
    int length = snprintf(header, sizeof(header), "%u,%u,%s",
                          (unsigned)sequence, (unsigned)strlen(body), body);
    assert(length > 0 && (size_t)length < sizeof(header));
    for (index = 0U; index < (size_t)length; index++)
        checksum ^= (unsigned char)header[index];
    length = snprintf(frame, sizeof(frame), "@%s*%02X\n", header,
                      (unsigned)checksum);
    assert(length > 0 && (size_t)length < sizeof(frame));
    for (index = 0U; index < (size_t)length; index++)
        Vision_RxByte(vision, (uint8_t)frame[index], s_now);
    Vision_Poll(vision, s_now);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);
uint32_t __get_PRIMASK(void) { return s_irq; }
void __disable_irq(void) { s_irq = 1U; }
void __set_PRIMASK(uint32_t mask) { s_irq = mask; }

int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length)
{
    assert(uart == &huart3 && length == 1U);
    s_destination = byte;
    return s_receive_error;
}

int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length)
{
    assert(uart == &huart3 && length > 0U && length < sizeof(s_tx_data));
    memcpy(s_tx_data, byte, length);
    s_tx_length = length;
    s_command = *byte;
    return s_transmit_error;
}
uint32_t HAL_GetTick(void) { return s_now; }
void Mecanum_Stop(MecanumChassis_t *chassis) { (void)chassis; }
void Mecanum_SetPositionMode(MecanumChassis_t *chassis, float x, float y, float yaw)
{ (void)chassis; (void)x; (void)y; (void)yaw; }
void Mecanum_GetPosition(MecanumChassis_t *chassis, float *x, float *y, float *theta)
{ (void)chassis; if (x) *x = 0.0f; if (y) *y = 0.0f; if (theta) *theta = 0.0f; }
void Mecanum_GetVelocity(MecanumChassis_t *chassis, float *vx, float *vy, float *omega)
{ (void)chassis; if (vx) *vx = 0.0f; if (vy) *vy = 0.0f; if (omega) *omega = 0.0f; }
uint8_t Mecanum_IsTargetReached(MecanumChassis_t *chassis) { (void)chassis; return 1U; }

static void receive(const char *text)
{
    while (*text) {
        *s_destination = (uint8_t)*text++;
        HAL_UART_RxCpltCallback(&huart3);
    }
}

int main(void)
{
    const char *valid = "#SEQ,4,5,2,3,2,1,2,5,4,3,1,2\n";
    char code[RACE_TASK_CODE_SIZE];
    unsigned index;
    assert(RaceVision_Init());
    receive(valid);
    assert(!RaceVision_Poll(code));
    receive(valid);
    s_irq = 1U;
    assert(RaceVision_BeginScan());
    assert(s_irq == 1U && s_tx_length == 1U && s_tx_data[0] == 'R');
    assert(!RaceVision_Poll(code));
    receive("2,5,4,3,1,2\n");
    assert(!RaceVision_Poll(code));
    receive(valid);
    assert(RaceVision_Poll(code));
    assert(strcmp(code, "452+321+254+312") == 0);
    RaceVision_EndScan();

    {
        VisionContext material;
        const VisionArea *area;
        Vision_Init(&material, local_tx, NULL, NULL);
        material.ready = 1U;
        assert(Vision_StartJob(&material, 40U, 0U));
        local_frame(&material, 1U, "ACK,JOB,40");
        local_frame(&material, 2U, "AREA_OK,160,120,72");
        area = Vision_GetArea(&material, 2U);
        assert(area != NULL && area->cx == 160U && area->cy == 120U &&
               Vision_GetResult(&material) == VISION_RESULT_AREA_OK);
        local_frame(&material, 3U, "IDLE");
        assert(Vision_StartJob(&material, 41U, 4U));
        local_frame(&material, 4U, "ACK,JOB,41");
        local_frame(&material, 5U, "ARRIVED,MATERIAL,0,1,-2");
        assert(Vision_GetResult(&material) == VISION_RESULT_ARRIVED);
        assert(Vision_GetPosition(&material, 5U) != NULL);
    }
    HAL_UART_TxCpltCallback(&huart3);
    receive(valid);
    assert(!RaceVision_Poll(code));
    assert(RaceVision_BeginScan());
    HAL_UART_TxCpltCallback(&huart3);
    s_now = VISION_ACK_TIMEOUT_MS + 1U;
    assert(!RaceVision_Poll(code));
    assert(RaceVision_ScanFailed());
    RaceVision_EndScan();
    s_now = 0U;
    HAL_UART_TxCpltCallback(&huart3);
    assert(RaceVision_Init());
    for (index = 0U; index < 260U; index++) receive("x");
    assert(!RaceVision_Healthy());
    assert(!RaceVision_Poll(code));
    assert(!RaceVision_BeginScan());
    assert(RaceVision_Init());
    HAL_UART_ErrorCallback(&huart3);
    assert(!RaceVision_Healthy());
    s_receive_error = HAL_ERROR;
    assert(!RaceVision_Init());
    s_receive_error = HAL_OK;
    assert(RaceVision_Init());
    s_transmit_error = HAL_ERROR;
    assert(!RaceVision_BeginScan());
    receive(valid);
    assert(!RaceVision_Poll(code));
    puts("PASS: UART scan gate, queued/partial stale frames, reset command, IRQ preservation, overflow/error/arming failures");
    return 0;
}
