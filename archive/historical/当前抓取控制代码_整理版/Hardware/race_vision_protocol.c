#include "race_vision_protocol.h"
#include <string.h>

uint8_t RaceVision_ParseSeq(const char *line, char code[RACE_TASK_CODE_SIZE])
{
    char candidate[RACE_TASK_CODE_SIZE];
    uint8_t colors[2][3], positions[2][3], index;
    const char *cursor;
    if (strncmp(line, "#SEQ,", 5U) != 0) return 0U;
    cursor = line + 5;
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
    if (!Race_ParseTask(candidate, colors, positions)) return 0U;
    memcpy(code, candidate, sizeof(candidate));
    return 1U;
}

void RaceVisionDecoder_Reset(RaceVisionDecoder *decoder)
{
    memset(decoder, 0, sizeof(*decoder));
}

uint8_t RaceVisionDecoder_Push(RaceVisionDecoder *decoder, uint8_t byte,
                              char code[RACE_TASK_CODE_SIZE])
{
    uint8_t accepted = 0U;
    if (byte == '\r' || byte == '\n') {
        if (decoder->collecting && !decoder->dropping) {
            decoder->line[decoder->length] = '\0';
            accepted = RaceVision_ParseSeq(decoder->line, code);
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
        decoder->length >= RACE_VISION_LINE_MAX - 1U) {
        decoder->dropping = 1U;
        return 0U;
    }
    decoder->line[decoder->length++] = (char)byte;
    return 0U;
}
