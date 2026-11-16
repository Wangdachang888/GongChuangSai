#ifndef GONGCHUANG_VISION_H
#define GONGCHUANG_VISION_H

#include <stddef.h>
#include <stdint.h>
#include "usart.h"
#include "race_fsm.h"
#include "mecanum_chassis.h"

#define VISION_FRAME_MAX             192U
#define VISION_BODY_MAX              128U
#define VISION_LEGACY_LINE_MAX       96U
#define VISION_RX_BUFFER_SIZE        256U
#define VISION_TX_QUEUE_DEPTH        4U
#define VISION_POS_TIMEOUT_MS        450U
#define VISION_ACK_TIMEOUT_MS        1600U
#define VISION_RING_STEP_TIMEOUT_MS  2500U
#define VISION_RING_TOTAL_TIMEOUT_MS 35000U
#define VISION_RING_STABLE_MS        180U
#define VISION_RING_STABLE_COUNT     4U

typedef void (*VisionTx)(const uint8_t *data, size_t length, void *user);
typedef void (*VisionStop)(void *user);

typedef enum {
    VISION_IDLE = 0,
    VISION_WAIT_ACK,
    VISION_RUNNING,
    VISION_WAIT_IDLE
} VisionPhase;

typedef enum {
    VISION_RESULT_NONE = 0,
    VISION_RESULT_ARRIVED,
    VISION_RESULT_QR_DONE,
    VISION_RESULT_TIMEOUT,
    VISION_RESULT_CANCELLED,
    VISION_RESULT_ERROR
} VisionResult;

typedef enum {
    VISION_RING_IDLE = 0,
    VISION_RING_WAIT_ACK,
    VISION_RING_WAIT_POS,
    VISION_RING_CALCULATE,
    VISION_RING_MOVING,
    VISION_RING_WAIT_IDLE,
    VISION_RING_SUCCESS,
    VISION_RING_ERROR
} VisionRingState;

typedef struct {
    int16_t dx;
    int16_t dy;
    uint16_t cx;
    uint16_t cy;
    uint8_t target;
    char kind[20];
    uint32_t time_ms;
    uint8_t valid;
} VisionPosition;

typedef struct {
    VisionRingState state;
    uint8_t target;
    uint16_t session_id;
    uint16_t step_id;
    uint8_t good_count;
    uint8_t correction_count;
    uint8_t stable_started;
    uint8_t calibration_valid;
    uint32_t started_ms;
    uint32_t step_started_ms;
    uint32_t stable_since_ms;
    uint32_t last_pos_ms;
    float target_x;
    float target_y;
    float target_theta;
    float matrix_00;
    float matrix_01;
    float matrix_10;
    float matrix_11;
    float gain;
    float max_step_m;
    float min_step_m;
    float max_speed_mps;
    float max_omega_radps;
    float tolerance_x_px;
    float tolerance_y_px;
    float saved_max_speed;
    float saved_max_omega;
    uint8_t speed_limited;
} VisionRingAlign;

typedef struct {
    VisionTx tx;
    VisionStop stop;
    void *user;
    char rx_frame[VISION_FRAME_MAX];
    uint16_t rx_frame_len;
    char legacy_line[VISION_LEGACY_LINE_MAX];
    uint8_t legacy_len;
    uint8_t legacy_collecting;
    uint8_t legacy_dropping;
    volatile uint8_t rx_buffer[VISION_RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint8_t rx_fault;
    uint8_t uart_byte;
    uint8_t ready;
    uint8_t tx_fault;
    uint8_t rx_seq_valid;
    uint16_t rx_seq;
    uint16_t tx_seq;
    uint8_t tx_active;
    uint8_t tx_head;
    uint8_t tx_tail;
    uint16_t tx_length[VISION_TX_QUEUE_DEPTH];
    uint8_t tx_data[VISION_TX_QUEUE_DEPTH][VISION_FRAME_MAX];
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t last_request_ms;
    uint32_t last_probe_ms;
    uint8_t job_code;
    VisionPhase phase;
    VisionResult result;
    VisionPosition pos;
    uint8_t qr_ready;
    char qr_code[RACE_TASK_CODE_SIZE];
    char wait_reason[32];
    char last_error[48];
    VisionRingAlign ring;
    MecanumChassis_t *ring_chassis;
} VisionContext;

typedef struct {
    char line[VISION_LEGACY_LINE_MAX];
    uint8_t length;
    uint8_t collecting;
    uint8_t dropping;
} RaceVisionDecoder;

void Vision_Init(VisionContext *vision, VisionTx tx, VisionStop stop, void *user);
VisionContext *Vision_GetDefault(void);
uint8_t Vision_UartInit(VisionContext *vision);
void Vision_RxCpltCallback(UART_HandleTypeDef *uart);
void Vision_TxCpltCallback(UART_HandleTypeDef *uart);
void Vision_ErrorCallback(UART_HandleTypeDef *uart);
void Vision_RxByte(VisionContext *vision, uint8_t byte, uint32_t now_ms);
void Vision_Poll(VisionContext *vision, uint32_t now_ms);

int Vision_SendBody(VisionContext *vision, const char *body);
int Vision_SendRaw(VisionContext *vision, const uint8_t *data, size_t length);
int Vision_StartJob(VisionContext *vision, unsigned code, uint32_t now_ms);
int Vision_StartJobTol(VisionContext *vision, unsigned code,
                       unsigned tolerance_x_px, unsigned tolerance_y_px,
                       uint32_t now_ms);
int Vision_Cancel(VisionContext *vision);
int Vision_Reset(VisionContext *vision);
int Vision_QueryStatus(VisionContext *vision);
const VisionPosition *Vision_GetPosition(const VisionContext *vision,
                                         uint32_t now_ms);
uint8_t Vision_TakeQr(VisionContext *vision, char code[RACE_TASK_CODE_SIZE]);

void Vision_SetRingChassis(VisionContext *vision, MecanumChassis_t *chassis);
void Vision_SetRingCalibration(VisionContext *vision,
                               float matrix_00, float matrix_01,
                               float matrix_10, float matrix_11,
                               float gain, float max_step_m,
                               float min_step_m, float max_speed_mps,
                               float max_omega_radps);
int Vision_StartRingAlign(VisionContext *vision, uint8_t target,
                          uint32_t now_ms);
VisionRingState Vision_GetRingState(const VisionContext *vision);
uint8_t Vision_RingActive(const VisionContext *vision);
uint8_t Vision_RingSucceeded(const VisionContext *vision);
uint8_t Vision_RingFailed(const VisionContext *vision);
void Vision_RingPoll(VisionContext *vision, uint32_t now_ms);

/* Compatibility adapters used by the existing QR route. */
uint8_t RaceVision_Init(void);
uint8_t RaceVision_BeginScan(void);
uint8_t RaceVision_Poll(char code[RACE_TASK_CODE_SIZE]);
uint8_t RaceVision_Healthy(void);
void RaceVision_EndScan(void);
uint8_t RaceVision_ParseSeq(const char *line, char code[RACE_TASK_CODE_SIZE]);
void RaceVisionDecoder_Reset(RaceVisionDecoder *decoder);
uint8_t RaceVisionDecoder_Push(RaceVisionDecoder *decoder, uint8_t byte,
                               char code[RACE_TASK_CODE_SIZE]);

#endif
