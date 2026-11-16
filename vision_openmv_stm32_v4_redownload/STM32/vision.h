#ifndef GONGCHUANG_VISION_H
#define GONGCHUANG_VISION_H
/*
 * STM32 <-> OpenMV 单步视觉任务接口；不依赖 HAL 具体串口编号。
 * 使用前在原 STM32 项目的 main.c 中调用 Vision_Init，
 * 把接收到的 UART 字节交给 Vision_RxByte，并周期调用 Vision_Poll。
 * 串口 115200 8N1；帧：@SEQ,LEN,BODY*XOR\n
 * 本文件为纯通信层，不自动驱动底盘或夹爪。
 */
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define VISION_FRAME_MAX           192u
#define VISION_BODY_MAX            128u
#define VISION_POS_TIMEOUT_MS      450u
#define VISION_ACK_TIMEOUT_MS      1600u

/* 回调由用户已有 STM32 程序实现；不要在串口 IRQ 内执行阻塞 TX 或电机操作 */
typedef void (*VisionTx)(const uint8_t *data, size_t length, void *user);
typedef void (*VisionStop)(void *user);

typedef enum {
    VISION_IDLE = 0,          /* 已收到 OpenMV IDLE，可执行下一视觉任务 */
    VISION_WAIT_ACK,          /* JOB 已发送，等待 ACK,JOB */
    VISION_RUNNING,           /* 可接收 POS */
    VISION_WAIT_IDLE         /* 已完成/取消，等待 OpenMV 最终 IDLE */
} VisionPhase;

typedef enum {
    VISION_RESULT_NONE = 0,
    VISION_RESULT_ARRIVED,
    VISION_RESULT_PICK_READY,
    VISION_RESULT_QR_DONE,
    VISION_RESULT_AREA_DONE,
    VISION_RESULT_TIMEOUT,
    VISION_RESULT_CANCELLED,
    VISION_RESULT_ERROR
} VisionResult;

typedef struct {
    int16_t dx, dy;          /* 相对参考点的像素偏差（不是毫米） */
    uint16_t cx, cy;         /* 图像坐标, 320x240 */
    uint8_t target;          /* 颜色 1-6 / 圆环 1-3 / 0=任意 */
    char kind[20];           /* MATERIAL, RING, EDGE, BLUE */
    uint32_t time_ms;
    uint8_t valid;
} VisionPosition;

typedef struct {
    VisionTx tx;
    VisionStop stop;
    void *user;
    char rx[VISION_FRAME_MAX];
    uint16_t rx_len;
    uint16_t tx_seq;
    uint32_t frames_ok, frames_bad;
    uint32_t last_request_ms;
    uint32_t last_probe_ms;
    uint8_t ready;
    uint8_t job_code;
    VisionPhase phase;
    VisionResult result;
    VisionPosition pos;
    uint8_t area_confirmed;
    uint8_t has_qr;
    char qr_code[16];
    uint8_t pick_ready;
    uint8_t pick_color, pick_batch, pick_index;
    uint8_t pick_ack_pending;
    char wait_reason[32];
    char last_error[48];
} VisionContext;

/* now_ms 可以直接传 HAL_GetTick()；tx 回调负责将字节发往 OpenMV。 */
void Vision_Init(VisionContext *v, VisionTx tx, VisionStop stop, void *user);
void Vision_RxByte(VisionContext *v, uint8_t byte, uint32_t now_ms);
void Vision_Poll(VisionContext *v, uint32_t now_ms);

/* 只有 ready 且 IDLE 才能启动任务。返回 1 表示已成功交给 UART 发送。 */
int Vision_StartJob(VisionContext *v, unsigned code, uint32_t now_ms);
int Vision_StartJobTol(VisionContext *v, unsigned code,
                       unsigned tol_x_px, unsigned tol_y_px, uint32_t now_ms);
int Vision_Cancel(VisionContext *v);
int Vision_ConfirmPick(VisionContext *v);   /* 机械爪实际抓取成功后调用 */
int Vision_RejectPick(VisionContext *v);    /* 实际抓取失败后调用 */
int Vision_DoneGroup(VisionContext *v, unsigned group); /* LCD 完成第 1..4 组 */
int Vision_ResetArea(VisionContext *v);
int Vision_Reset(VisionContext *v);
int Vision_QueryStatus(VisionContext *v);

/* 视觉任务发送的是 BODY，不必自己计算长度/XOR。 */
int Vision_SendBody(VisionContext *v, const char *body);
/* 目标位置在 450ms 内有效，且当前任务仍运行时返回非 NULL。 */
const VisionPosition *Vision_GetPosition(const VisionContext *v, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
