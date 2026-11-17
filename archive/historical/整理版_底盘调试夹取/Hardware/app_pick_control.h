#ifndef APP_PICK_CONTROL_H
#define APP_PICK_CONTROL_H

#include <stdint.h>

/* PE0: short press = forward pick/place, long press = reverse pick/release. */
void PickControl_Init(void);
void PickControl_Poll(void);
uint8_t PickControl_IsBusy(void);

#endif
