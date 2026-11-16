#ifndef RACE_VISION_H
#define RACE_VISION_H
#include "race_fsm.h"
uint8_t RaceVision_Init(void);
uint8_t RaceVision_BeginScan(void);
uint8_t RaceVision_Poll(char code[RACE_TASK_CODE_SIZE]);
uint8_t RaceVision_Healthy(void);
void RaceVision_EndScan(void);
#endif
