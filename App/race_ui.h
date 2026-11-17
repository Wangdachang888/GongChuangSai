#ifndef RACE_UI_H
#define RACE_UI_H

#include <stdint.h>
#include "race_fsm.h"

#define RACE_INPUT_PRESS          0x01U
#define RACE_INPUT_SHORT_RELEASE  0x02U
#define RACE_INPUT_LONG          0x04U

typedef struct {
    uint8_t raw;
    uint8_t stable;
    uint8_t armed;
    uint8_t long_sent;
    uint8_t start_on_release;
    uint32_t changed_ms;
    uint32_t pressed_ms;
} RaceInput;

void RaceInput_Init(RaceInput *input, uint32_t now);
uint8_t RaceInput_Poll(RaceInput *input, uint32_t now,
                       uint8_t race_running, uint8_t can_start);
void RaceUi_Init(void);
void RaceUi_Poll(const RaceFsm *race, uint32_t now, uint8_t warmed_up);

#endif
