#include "vision.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static VisionContext s_default;
static VisionContext *s_uart_vision;
static uint8_t s_scan_active;

static uint16_t ring_index(uint16_t value)
{
    return (uint16_t)(value % VISION_RX_BUFFER_SIZE);
}

static void copy_text(char *destination, size_t capacity, const char *source)
{
    if (capacity == 0U) return;
    if (source == NULL) source = "";
    (void)snprintf(destination, capacity, "%s", source);
}

static int parse_long(const char *text, long *value)
{
    char *end;
    long parsed;
    if (text == NULL || *text == '\0' || value == NULL) return 0;
    parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0') return 0;
    *value = parsed;
    return 1;
}

static int parse_unsigned(const char *text, unsigned long *value)
{
    char *end;
    unsigned long parsed;
    if (text == NULL || *text == '\0' || value == NULL ||
        !isdigit((unsigned char)text[0])) return 0;
    parsed = strtoul(text, &end, 10);
    if (end == text || *end != '\0') return 0;
    *value = parsed;
    return 1;
}

static unsigned char xor_bytes(const char *text, size_t length)
{
    unsigned char result = 0U;
    size_t index;
    for (index = 0U; index < length; index++) result ^= (unsigned char)text[index];
    return result;
}

static int split_tokens(char *text, char **fields, int capacity)
{
    int count = 0;
    char *comma;
    while (count < capacity && text != NULL && *text != '\0') {
        fields[count++] = text;
        comma = strchr(text, ',');
        if (comma == NULL) return count;
        *comma = '\0';
        text = comma + 1;
    }
    return 0;
}

static int newer_sequence(uint16_t sequence, uint16_t previous)
{
    uint16_t distance = (uint16_t)(sequence - previous);
    return distance != 0U && distance < 0x8000U;
}

static int job_code_valid(unsigned code)
{
    return code == 1U || (code >= 11U && code <= 16U) ||
           (code >= 21U && code <= 23U) || code == 31U || code == 32U ||
           code == 40U || code == 41U || code == 42U;
}

/* QR task-code validation is a protocol concern. Keep it local to Vision so
 * the hardware protocol parser does not depend on the App race FSM. */
static uint8_t vision_task_code_valid(const char *code)
{
    uint8_t group, index, mask, color_masks[2] = {0U, 0U};
    if (code == NULL || strlen(code) != 15U) return 0U;
    for (group = 0U; group < 4U; group++) {
        mask = 0U;
        for (index = 0U; index < 3U; index++) {
            char digit = code[group * 4U + index];
            uint8_t value;
            if (digit < '1' || digit > ((group & 1U) ? '3' : '6')) return 0U;
            value = (uint8_t)(digit - '0');
            if (mask & (uint8_t)(1U << value)) return 0U;
            mask |= (uint8_t)(1U << value);
        }
        if (group < 3U && code[group * 4U + 3U] != '+') return 0U;
        if ((group & 1U) == 0U) color_masks[group / 2U] = mask;
    }
    if (color_masks[0] != color_masks[1]) return 0U;
    return 1U;
}

static int ring_target_valid(long target)
{
    return target >= 1L && target <= 3L;
}

static void stop_vehicle(VisionContext *vision)
{
    if (vision == NULL) return;
    vision->pos.valid = 0U;
    if (vision->stop != NULL) vision->stop(vision->user);
}

static void ring_fail(VisionContext *vision, const char *reason)
{
    if (vision == NULL) return;
    copy_text(vision->last_error, sizeof(vision->last_error), reason);
    vision->result = VISION_RESULT_ERROR;
    vision->ring.state = VISION_RING_ERROR;
    vision->phase = VISION_IDLE;
    stop_vehicle(vision);
}

static void start_next_tx(VisionContext *vision)
{
    uint8_t index;
    if (vision == NULL || vision->tx != NULL || vision->tx_active ||
        vision->tx_tail == vision->tx_head) return;
    index = vision->tx_tail;
    if (HAL_UART_Transmit_IT(&huart3, vision->tx_data[index],
                             vision->tx_length[index]) != HAL_OK) {
        vision->tx_fault = 1U;
        return;
    }
    vision->tx_active = 1U;
}

static int queue_uart_frame(VisionContext *vision, const uint8_t *data,
                            uint16_t length)
{
    uint8_t next;
    uint8_t index;
    if (vision == NULL || data == NULL || length == 0U ||
        length >= VISION_FRAME_MAX) return 0;
    next = (uint8_t)((vision->tx_head + 1U) % VISION_TX_QUEUE_DEPTH);
    if (next == vision->tx_tail) {
        vision->tx_fault = 1U;
        return 0;
    }
    index = vision->tx_head;
    memcpy(vision->tx_data[index], data, length);
    vision->tx_length[index] = length;
    vision->tx_head = next;
    start_next_tx(vision);
    return vision->tx_fault ? 0 : 1;
}

static int decode_frame(char *line, uint16_t *sequence, char **body)
{
    char *star;
    char *first;
    char *second;
    unsigned long parsed_sequence;
    unsigned long parsed_length;
    unsigned checksum;
    if (line == NULL || sequence == NULL || body == NULL || line[0] != '@') return 0;
    star = strchr(line, '*');
    if (star == NULL || strlen(star + 1U) != 2U ||
        !isxdigit((unsigned char)star[1]) || !isxdigit((unsigned char)star[2]) ||
        sscanf(star + 1U, "%2x", &checksum) != 1) return 0;
    if (xor_bytes(line + 1U, (size_t)(star - line - 1)) != (unsigned char)checksum) return 0;
    *star = '\0';
    first = strchr(line + 1U, ',');
    if (first == NULL) return 0;
    *first = '\0';
    if (!parse_unsigned(line + 1U, &parsed_sequence) || parsed_sequence > 65535UL) return 0;
    second = strchr(first + 1U, ',');
    if (second == NULL) return 0;
    *second = '\0';
    if (!parse_unsigned(first + 1U, &parsed_length) || parsed_length > VISION_BODY_MAX) return 0;
    *body = second + 1U;
    if (parsed_length == 0UL || strlen(*body) != parsed_length) return 0;
    *sequence = (uint16_t)parsed_sequence;
    return 1;
}

static uint8_t parse_legacy_qr(const char *line, char code[VISION_TASK_CODE_SIZE])
{
    char candidate[VISION_TASK_CODE_SIZE];
    const char *cursor;
    uint8_t index;
    if (line == NULL || strncmp(line, "#SEQ,", 5U) != 0) return 0U;
    cursor = line + 5U;
    for (index = 0U; index < 12U; index++) {
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (*cursor < '1' || *cursor > '6') return 0U;
        candidate[index + index / 3U] = *cursor++;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (index != 11U) {
            if (*cursor++ != ',') return 0U;
        } else if (*cursor != '\0') {
            return 0U;
        }
    }
    candidate[3] = '+';
    candidate[7] = '+';
    candidate[11] = '+';
    candidate[15] = '\0';
    if (!vision_task_code_valid(candidate)) return 0U;
    memcpy(code, candidate, sizeof(candidate));
    return 1U;
}

static void accept_position(VisionContext *vision, const char *kind,
                            long target, long dx, long dy, long cx, long cy,
                            uint32_t now_ms)
{
    vision->pos.dx = (int16_t)dx;
    vision->pos.dy = (int16_t)dy;
    vision->pos.cx = (uint16_t)cx;
    vision->pos.cy = (uint16_t)cy;
    vision->pos.target = (uint8_t)target;
    copy_text(vision->pos.kind, sizeof(vision->pos.kind), kind);
    vision->pos.time_ms = now_ms;
    vision->pos.valid = 1U;
}

static int valid_position(long dx, long dy, long cx, long cy)
{
    return dx >= -320L && dx <= 320L && dy >= -240L && dy <= 240L &&
           cx >= 0L && cx < 320L && cy >= 0L && cy < 240L;
}

static void handle_body(VisionContext *vision, char *body, uint32_t now_ms)
{
    char *fields[12];
    int count;
    long target, dx, dy, cx, cy, session, step;
    if (vision == NULL || body == NULL) return;
    count = split_tokens(body, fields, 12);
    if (count <= 0) return;

    if (!strcmp(fields[0], "READY") && count == 2 && !strcmp(fields[1], "IDLE")) {
        if (Vision_RingActive(vision)) ring_fail(vision, "OPENMV_RESTART");
        vision->ready = 1U;
        vision->rx_seq_valid = 0U;
        vision->phase = VISION_IDLE;
        return;
    }
    if (!strcmp(fields[0], "ACK") && count == 3 && !strcmp(fields[1], "JOB")) {
        if (parse_long(fields[2], &target) && vision->phase == VISION_WAIT_ACK &&
            target == vision->job_code) vision->phase = VISION_RUNNING;
        return;
    }
    if (!strcmp(fields[0], "ACK") && count == 4 && !strcmp(fields[1], "ALIGN")) {
        if (parse_long(fields[2], &target) && parse_long(fields[3], &session) &&
            Vision_RingActive(vision) && vision->ring.state == VISION_RING_WAIT_ACK &&
            target == vision->ring.target && session == vision->ring.session_id) {
            vision->phase = VISION_RUNNING;
            vision->ring.state = VISION_RING_WAIT_POS;
            vision->ring.step_started_ms = now_ms;
        }
        return;
    }
    if (!strcmp(fields[0], "POS") && count == 7) {
        if (vision->phase != VISION_RUNNING || !parse_long(fields[2], &target) ||
            !parse_long(fields[3], &dx) || !parse_long(fields[4], &dy) ||
            !parse_long(fields[5], &cx) || !parse_long(fields[6], &cy) ||
            !valid_position(dx, dy, cx, cy)) return;
        accept_position(vision, fields[1], target, dx, dy, cx, cy, now_ms);
        return;
    }
    if (!strcmp(fields[0], "POS") && count == 9 && !strcmp(fields[1], "RING")) {
        if (!parse_long(fields[2], &target) || !parse_long(fields[3], &dx) ||
            !parse_long(fields[4], &dy) || !parse_long(fields[5], &cx) ||
            !parse_long(fields[6], &cy) || !parse_long(fields[7], &session) ||
            !parse_long(fields[8], &step) || !ring_target_valid(target) ||
            !valid_position(dx, dy, cx, cy) || !Vision_RingActive(vision) ||
            vision->ring.state != VISION_RING_WAIT_POS ||
            target != vision->ring.target || session != vision->ring.session_id ||
            step != vision->ring.step_id) return;
        accept_position(vision, fields[1], target, dx, dy, cx, cy, now_ms);
        vision->ring.state = VISION_RING_CALCULATE;
        return;
    }
    if (!strcmp(fields[0], "QR") && count == 2 &&
        strlen(fields[1]) == 15U) {
        if (vision_task_code_valid(fields[1])) {
            copy_text(vision->qr_code, sizeof(vision->qr_code), fields[1]);
            vision->qr_ready = 1U;
        }
        return;
    }
    if (!strcmp(fields[0], "DONE") && count == 2 && !strcmp(fields[1], "QR")) {
        vision->result = VISION_RESULT_QR_DONE;
        return;
    }
    if (!strcmp(fields[0], "LOST") && count >= 2) {
        copy_text(vision->wait_reason, sizeof(vision->wait_reason), fields[1]);
        return;
    }
    if (!strcmp(fields[0], "ERR") && count >= 2) {
        copy_text(vision->last_error, sizeof(vision->last_error), fields[1]);
        if (Vision_RingActive(vision)) ring_fail(vision, fields[1]);
        else {
            vision->result = VISION_RESULT_ERROR;
            vision->phase = VISION_IDLE;
        }
        return;
    }
    if (!strcmp(fields[0], "TIMEOUT")) {
        vision->result = VISION_RESULT_TIMEOUT;
        if (Vision_RingActive(vision)) ring_fail(vision, "OPENMV_TIMEOUT");
        else vision->phase = VISION_WAIT_IDLE;
        return;
    }
    if (!strcmp(fields[0], "IDLE") && count == 1) {
        if (vision->ring.state == VISION_RING_WAIT_IDLE) {
            vision->ring.state = VISION_RING_SUCCESS;
            vision->result = VISION_RESULT_ARRIVED;
        }
        vision->phase = VISION_IDLE;
        return;
    }
}

static void process_frame(VisionContext *vision, uint32_t now_ms)
{
    char *body;
    uint16_t sequence;
    int is_ready;
    if (!decode_frame(vision->rx_frame, &sequence, &body)) {
        vision->frames_bad++;
        return;
    }
    is_ready = !strncmp(body, "READY,IDLE", 10U);
    if (!is_ready && vision->rx_seq_valid && !newer_sequence(sequence, vision->rx_seq)) return;
    vision->rx_seq = sequence;
    vision->rx_seq_valid = 1U;
    vision->frames_ok++;
    handle_body(vision, body, now_ms);
}

static void process_legacy_line(VisionContext *vision, uint32_t now_ms)
{
    (void)now_ms;
    if (parse_legacy_qr(vision->legacy_line, vision->qr_code)) vision->qr_ready = 1U;
}

void Vision_Init(VisionContext *vision, VisionTx tx, VisionStop stop, void *user)
{
    if (vision == NULL) return;
    memset(vision, 0, sizeof(*vision));
    vision->tx = tx;
    vision->stop = stop;
    vision->user = user;
    vision->phase = VISION_IDLE;
    vision->ring.state = VISION_RING_IDLE;
}

VisionContext *Vision_GetDefault(void)
{
    return &s_default;
}

uint8_t Vision_UartInit(VisionContext *vision)
{
    if (vision == NULL) return 0U;
    s_uart_vision = vision;
    vision->rx_head = 0U;
    vision->rx_tail = 0U;
    vision->rx_fault = 0U;
    vision->tx_fault = 0U;
    vision->rx_frame_len = 0U;
    vision->legacy_len = 0U;
    vision->legacy_collecting = 0U;
    vision->legacy_dropping = 0U;
    vision->uart_byte = 0U;
    vision->ready = HAL_UART_Receive_IT(&huart3, &vision->uart_byte, 1U) == HAL_OK;
    return vision->ready;
}

void Vision_RxCpltCallback(UART_HandleTypeDef *uart)
{
    VisionContext *vision = s_uart_vision;
    uint16_t next;
    if (vision == NULL || uart == NULL || uart->Instance != USART3) return;
    next = ring_index((uint16_t)(vision->rx_head + 1U));
    if (next == vision->rx_tail) {
        vision->rx_fault = 1U;
    } else {
        vision->rx_buffer[vision->rx_head] = vision->uart_byte;
        vision->rx_head = next;
    }
    if (HAL_UART_Receive_IT(&huart3, &vision->uart_byte, 1U) != HAL_OK)
        vision->rx_fault = 1U;
}

void Vision_TxCpltCallback(UART_HandleTypeDef *uart)
{
    VisionContext *vision = s_uart_vision;
    if (vision == NULL || uart == NULL || uart->Instance != USART3) return;
    if (vision->tx_tail != vision->tx_head)
        vision->tx_tail = (uint8_t)((vision->tx_tail + 1U) % VISION_TX_QUEUE_DEPTH);
    vision->tx_active = 0U;
    start_next_tx(vision);
}

void Vision_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (s_uart_vision == NULL || uart == NULL || uart->Instance != USART3) return;
    s_uart_vision->rx_fault = 1U;
}

void Vision_RxByte(VisionContext *vision, uint8_t byte, uint32_t now_ms)
{
    if (vision == NULL) return;
    if (byte == '@') {
        vision->rx_frame[0] = '@';
        vision->rx_frame_len = 1U;
        vision->legacy_collecting = 0U;
        vision->legacy_len = 0U;
        return;
    }
    if (byte == '#') {
        vision->rx_frame_len = 0U;
        vision->legacy_collecting = 1U;
        vision->legacy_dropping = 0U;
        vision->legacy_len = 0U;
        vision->legacy_line[vision->legacy_len++] = '#';
        return;
    }
    if (byte == '\r') return;
    if (byte == '\n') {
        if (vision->rx_frame_len != 0U) {
            vision->rx_frame[vision->rx_frame_len] = '\0';
            process_frame(vision, now_ms);
            vision->rx_frame_len = 0U;
        }
        if (vision->legacy_collecting) {
            if (!vision->legacy_dropping) {
                vision->legacy_line[vision->legacy_len] = '\0';
                process_legacy_line(vision, now_ms);
            }
            vision->legacy_collecting = 0U;
            vision->legacy_len = 0U;
            vision->legacy_dropping = 0U;
        }
        return;
    }
    if (vision->rx_frame_len != 0U) {
        if (byte < 32U || byte > 126U ||
            vision->rx_frame_len >= VISION_FRAME_MAX - 1U) {
            vision->rx_frame_len = 0U;
            vision->frames_bad++;
        } else {
            vision->rx_frame[vision->rx_frame_len++] = (char)byte;
        }
    } else if (vision->legacy_collecting && !vision->legacy_dropping) {
        if (byte < 32U || byte > 126U ||
            vision->legacy_len >= VISION_LEGACY_LINE_MAX - 1U) {
            vision->legacy_dropping = 1U;
        } else {
            vision->legacy_line[vision->legacy_len++] = (char)byte;
        }
    }
}

static void drain_rx(VisionContext *vision, uint32_t now_ms)
{
    uint16_t tail;
    if (vision == NULL) return;
    while (vision->rx_tail != vision->rx_head) {
        tail = vision->rx_tail;
        vision->rx_tail = ring_index((uint16_t)(tail + 1U));
        Vision_RxByte(vision, vision->rx_buffer[tail], now_ms);
    }
}

int Vision_SendBody(VisionContext *vision, const char *body)
{
    char header[VISION_FRAME_MAX];
    char frame[VISION_FRAME_MAX];
    size_t length;
    int header_length;
    int frame_length;
    if (vision == NULL || body == NULL) return 0;
    length = strlen(body);
    if (length == 0U || length > VISION_BODY_MAX) return 0;
    if (strpbrk(body, "*@") != NULL) return 0;
    header_length = snprintf(header, sizeof(header), "%u,%u,%s",
                             (unsigned)vision->tx_seq, (unsigned)length, body);
    if (header_length <= 0 || header_length >= (int)sizeof(header)) return 0;
    frame_length = snprintf(frame, sizeof(frame), "@%s*%02X\n", header,
                            (unsigned)xor_bytes(header, (size_t)header_length));
    if (frame_length <= 0 || frame_length >= (int)sizeof(frame)) return 0;
    vision->tx_seq = (uint16_t)(vision->tx_seq + 1U);
    if (vision->tx != NULL) {
        vision->tx((const uint8_t *)frame, (size_t)frame_length, vision->user);
        return 1;
    }
    return queue_uart_frame(vision, (const uint8_t *)frame, (uint16_t)frame_length);
}

int Vision_SendRaw(VisionContext *vision, const uint8_t *data, size_t length)
{
    if (vision == NULL || data == NULL || length == 0U ||
        length >= VISION_FRAME_MAX) return 0;
    if (vision->tx != NULL) {
        vision->tx(data, length, vision->user);
        return 1;
    }
    return queue_uart_frame(vision, data, (uint16_t)length);
}

int Vision_StartJobTol(VisionContext *vision, unsigned code,
                       unsigned tolerance_x_px, unsigned tolerance_y_px,
                       uint32_t now_ms)
{
    char body[40];
    int written;
    if (vision == NULL || !vision->ready || vision->phase != VISION_IDLE ||
        Vision_RingActive(vision) || !job_code_valid(code) ||
        tolerance_x_px < 1U || tolerance_x_px > 100U ||
        tolerance_y_px < 1U || tolerance_y_px > 100U) return 0;
    written = snprintf(body, sizeof(body), "JOB,%u,%u,%u", code,
                       tolerance_x_px, tolerance_y_px);
    if (written <= 0 || written >= (int)sizeof(body)) return 0;
    vision->phase = VISION_WAIT_ACK;
    vision->job_code = (uint8_t)code;
    vision->last_request_ms = now_ms;
    vision->result = VISION_RESULT_NONE;
    vision->pos.valid = 0U;
    vision->qr_ready = 0U;
    if (!Vision_SendBody(vision, body)) {
        vision->phase = VISION_IDLE;
        return 0;
    }
    return 1;
}

int Vision_StartJob(VisionContext *vision, unsigned code, uint32_t now_ms)
{
    char body[24];
    int written;
    if (vision == NULL || !vision->ready || vision->phase != VISION_IDLE ||
        Vision_RingActive(vision) || !job_code_valid(code)) return 0;
    written = snprintf(body, sizeof(body), "JOB,%u", code);
    if (written <= 0 || written >= (int)sizeof(body)) return 0;
    vision->phase = VISION_WAIT_ACK;
    vision->job_code = (uint8_t)code;
    vision->last_request_ms = now_ms;
    vision->result = VISION_RESULT_NONE;
    vision->pos.valid = 0U;
    vision->qr_ready = 0U;
    if (!Vision_SendBody(vision, body)) {
        vision->phase = VISION_IDLE;
        return 0;
    }
    return 1;
}

int Vision_Cancel(VisionContext *vision)
{
    if (vision == NULL || !vision->ready) return 0;
    if (!Vision_SendBody(vision, "CANCEL")) return 0;
    vision->result = VISION_RESULT_CANCELLED;
    if (Vision_RingActive(vision)) ring_fail(vision, "CANCELLED");
    else {
        vision->phase = VISION_WAIT_IDLE;
        stop_vehicle(vision);
    }
    return 1;
}

int Vision_Reset(VisionContext *vision)
{
    if (vision == NULL || !vision->ready) return 0;
    if (!Vision_SendBody(vision, "RESET")) return 0;
    vision->phase = VISION_WAIT_IDLE;
    vision->qr_ready = 0U;
    if (Vision_RingActive(vision)) ring_fail(vision, "RESET");
    else stop_vehicle(vision);
    return 1;
}

int Vision_QueryStatus(VisionContext *vision)
{
    return vision != NULL ? Vision_SendBody(vision, "STATUS") : 0;
}

const VisionPosition *Vision_GetPosition(const VisionContext *vision,
                                         uint32_t now_ms)
{
    if (vision == NULL || !vision->pos.valid ||
        (uint32_t)(now_ms - vision->pos.time_ms) > VISION_POS_TIMEOUT_MS) return NULL;
    return &vision->pos;
}

uint8_t Vision_TakeQr(VisionContext *vision, char code[VISION_TASK_CODE_SIZE])
{
    if (vision == NULL || code == NULL || !vision->qr_ready) return 0U;
    memcpy(code, vision->qr_code, VISION_TASK_CODE_SIZE);
    vision->qr_ready = 0U;
    return 1U;
}

void Vision_SetStop(VisionContext *vision, VisionStop stop, void *user)
{
    if (vision == NULL) return;
    vision->stop = stop;
    vision->user = user;
}

int Vision_StartRingAlign(VisionContext *vision, uint8_t target, uint32_t now_ms)
{
    char body[40];
    int written;
    if (vision == NULL || !vision->ready || !ring_target_valid(target) ||
        vision->phase != VISION_IDLE || Vision_RingActive(vision)) return 0;
    if (vision->ring.session_id == 0U || vision->ring.session_id == 65535U)
        vision->ring.session_id = 1U;
    else vision->ring.session_id++;
    vision->ring.target = target;
    vision->ring.step_id = 0U;
    vision->ring.good_count = 0U;
    vision->ring.started_ms = now_ms;
    vision->ring.step_started_ms = now_ms;
    vision->ring.state = VISION_RING_WAIT_ACK;
    vision->phase = VISION_WAIT_ACK;
    vision->result = VISION_RESULT_NONE;
    vision->pos.valid = 0U;
    written = snprintf(body, sizeof(body), "ALIGN,%u,%u", (unsigned)target,
                       (unsigned)vision->ring.session_id);
    if (written <= 0 || written >= (int)sizeof(body) || !Vision_SendBody(vision, body)) {
        ring_fail(vision, "ALIGN_SEND");
        return 0;
    }
    return 1;
}

VisionRingState Vision_GetRingState(const VisionContext *vision)
{
    return vision == NULL ? VISION_RING_ERROR : vision->ring.state;
}

uint8_t Vision_RingActive(const VisionContext *vision)
{
    if (vision == NULL) return 0U;
    return vision->ring.state == VISION_RING_WAIT_ACK ||
           vision->ring.state == VISION_RING_WAIT_POS ||
           vision->ring.state == VISION_RING_CALCULATE ||
           vision->ring.state == VISION_RING_MOVING ||
           vision->ring.state == VISION_RING_WAIT_IDLE;
}

uint8_t Vision_RingSucceeded(const VisionContext *vision)
{
    return vision != NULL && vision->ring.state == VISION_RING_SUCCESS;
}

uint8_t Vision_RingFailed(const VisionContext *vision)
{
    return vision != NULL && vision->ring.state == VISION_RING_ERROR;
}

uint8_t Vision_RingTarget(const VisionContext *vision)
{
    return vision == NULL ? 0U : vision->ring.target;
}

uint16_t Vision_RingSession(const VisionContext *vision)
{
    return vision == NULL ? 0U : vision->ring.session_id;
}

uint16_t Vision_RingStep(const VisionContext *vision)
{
    return vision == NULL ? 0U : vision->ring.step_id;
}

uint32_t Vision_RingStartedAt(const VisionContext *vision)
{
    return vision == NULL ? 0U : vision->ring.started_ms;
}

uint32_t Vision_RingStepStartedAt(const VisionContext *vision)
{
    return vision == NULL ? 0U : vision->ring.step_started_ms;
}

void Vision_RingMarkMoving(VisionContext *vision, uint32_t now_ms)
{
    if (vision == NULL || vision->ring.state != VISION_RING_CALCULATE) return;
    vision->ring.step_started_ms = now_ms;
    vision->ring.state = VISION_RING_MOVING;
    vision->pos.valid = 0U;
}

static uint8_t send_ring_step(VisionContext *vision, uint32_t now_ms,
                              uint8_t moving)
{
    char body[40];
    int written;
    written = snprintf(body, sizeof(body), "STEP_DONE,%u,%u",
                       (unsigned)vision->ring.step_id, (unsigned)moving);
    if (written <= 0 || written >= (int)sizeof(body) ||
        !Vision_SendBody(vision, body)) {
        ring_fail(vision, "RING_STEP_SEND");
        return 0U;
    }
    vision->ring.step_id++;
    vision->ring.step_started_ms = now_ms;
    vision->ring.state = VISION_RING_WAIT_POS;
    vision->pos.valid = 0U;
    return 1U;
}

uint8_t Vision_RingMarkStepDone(VisionContext *vision, uint32_t now_ms)
{
    if (vision == NULL || vision->ring.state != VISION_RING_MOVING) return 0U;
    return send_ring_step(vision, now_ms, 1U);
}

uint8_t Vision_RingMarkStable(VisionContext *vision, uint32_t now_ms)
{
    char body[64];
    int written;
    if (vision == NULL || vision->ring.state != VISION_RING_CALCULATE) return 0U;
    vision->ring.good_count++;
    if (vision->ring.good_count < VISION_RING_STABLE_COUNT)
        return send_ring_step(vision, now_ms, 0U);
    written = snprintf(body, sizeof(body), "ARRIVED,RING,%u,%d,%d,%u,%u",
                       (unsigned)vision->ring.target, (int)vision->pos.dx,
                       (int)vision->pos.dy, (unsigned)vision->ring.session_id,
                       (unsigned)vision->ring.good_count);
    if (written <= 0 || written >= (int)sizeof(body) ||
        !Vision_SendBody(vision, body)) {
        ring_fail(vision, "RING_ARRIVED_SEND");
        return 0U;
    }
    vision->ring.state = VISION_RING_WAIT_IDLE;
    vision->phase = VISION_WAIT_IDLE;
    vision->pos.valid = 0U;
    return 1U;
}

void Vision_RingClearPosition(VisionContext *vision)
{
    if (vision != NULL) vision->pos.valid = 0U;
}

void Vision_RingFail(VisionContext *vision, const char *reason)
{
    ring_fail(vision, reason);
}

void Vision_Poll(VisionContext *vision, uint32_t now_ms)
{
    if (vision == NULL) return;
    drain_rx(vision, now_ms);
    if (!vision->ready && (uint32_t)(now_ms - vision->last_probe_ms) >= 1000U) {
        vision->last_probe_ms = now_ms;
        (void)Vision_SendBody(vision, "STATUS");
    }
    if (vision->phase == VISION_WAIT_ACK &&
        (uint32_t)(now_ms - vision->last_request_ms) > VISION_ACK_TIMEOUT_MS &&
        !Vision_RingActive(vision)) {
        copy_text(vision->last_error, sizeof(vision->last_error), "ACK_TIMEOUT");
        vision->result = VISION_RESULT_ERROR;
        vision->phase = VISION_WAIT_IDLE;
        stop_vehicle(vision);
        (void)Vision_SendBody(vision, "CANCEL");
    }
    if (vision->pos.valid &&
        (uint32_t)(now_ms - vision->pos.time_ms) > VISION_POS_TIMEOUT_MS)
        stop_vehicle(vision);
}

uint8_t RaceVision_Init(void)
{
    Vision_Init(&s_default, NULL, NULL, NULL);
    s_scan_active = 0U;
    return Vision_UartInit(&s_default);
}

uint8_t RaceVision_Healthy(void)
{
    return s_default.ready && !s_default.rx_fault && !s_default.tx_fault;
}

uint8_t RaceVision_BeginScan(void)
{
    static const uint8_t legacy_reset = 'R';
    uint32_t saved_irq = __get_PRIMASK();
    if (s_scan_active) return RaceVision_Healthy();
    __disable_irq();
    s_default.rx_tail = s_default.rx_head;
    __set_PRIMASK(saved_irq);
    s_default.legacy_collecting = 0U;
    s_default.legacy_len = 0U;
    if (!RaceVision_Healthy()) return 0U;
    if (!Vision_RingActive(&s_default)) s_default.phase = VISION_IDLE;
    if (!Vision_SendRaw(&s_default, &legacy_reset, 1U)) return 0U;
    s_scan_active = (uint8_t)Vision_StartJob(&s_default, 1U, HAL_GetTick());
    return s_scan_active;
}

uint8_t RaceVision_ScanFailed(void)
{
    if (!s_scan_active) return 0U;
    return s_default.result == VISION_RESULT_TIMEOUT ||
           s_default.result == VISION_RESULT_ERROR;
}

void RaceVision_EndScan(void)
{
    s_scan_active = 0U;
    s_default.qr_ready = 0U;
    s_default.legacy_collecting = 0U;
    s_default.legacy_len = 0U;
}

uint8_t RaceVision_Poll(char code[VISION_TASK_CODE_SIZE])
{
    Vision_Poll(&s_default, HAL_GetTick());
    if (!s_scan_active) return 0U;
    return Vision_TakeQr(&s_default, code);
}

uint8_t RaceVision_ParseSeq(const char *line, char code[VISION_TASK_CODE_SIZE])
{
    return parse_legacy_qr(line, code);
}

void RaceVisionDecoder_Reset(RaceVisionDecoder *decoder)
{
    if (decoder != NULL) memset(decoder, 0, sizeof(*decoder));
}

uint8_t RaceVisionDecoder_Push(RaceVisionDecoder *decoder, uint8_t byte,
                               char code[VISION_TASK_CODE_SIZE])
{
    uint8_t accepted = 0U;
    if (decoder == NULL || code == NULL) return 0U;
    if (byte == '\r' || byte == '\n') {
        if (decoder->collecting && !decoder->dropping) {
            decoder->line[decoder->length] = '\0';
            accepted = parse_legacy_qr(decoder->line, code);
        }
        RaceVisionDecoder_Reset(decoder);
        return accepted;
    }
    if (decoder->dropping) return 0U;
    if (!decoder->collecting) {
        if (byte != '#') return 0U;
        decoder->collecting = 1U;
    }
    if ((byte < 32U && byte != '\t') || byte > 126U ||
        decoder->length >= VISION_LEGACY_LINE_MAX - 1U) {
        decoder->dropping = 1U;
        return 0U;
    }
    decoder->line[decoder->length++] = (char)byte;
    return 0U;
}
