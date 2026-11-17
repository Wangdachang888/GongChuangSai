#ifndef RACE_VISION_PROTOCOL_H
#define RACE_VISION_PROTOCOL_H
#include "race_fsm.h"
#define RACE_VISION_LINE_MAX 96U
typedef struct {
    char line[RACE_VISION_LINE_MAX];
    uint8_t length, collecting, dropping;
} RaceVisionDecoder;
void RaceVisionDecoder_Reset(RaceVisionDecoder *decoder);
uint8_t RaceVisionDecoder_Push(RaceVisionDecoder *decoder, uint8_t byte,
                              char code[RACE_TASK_CODE_SIZE]);
uint8_t RaceVision_ParseSeq(const char *line, char code[RACE_TASK_CODE_SIZE]);
#endif
