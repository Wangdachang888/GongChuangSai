#ifndef APP_PICK_CONTROL_H
#define APP_PICK_CONTROL_H

#include <stdint.h>

/* PE0: short press = ground-to-chassis, long press = chassis-to-ground. */
void PickControl_Init(void);
void PickControl_Poll(void);
uint8_t PickControl_IsBusy(void);

#endif
