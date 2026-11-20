#ifndef APP_RACE_H
#define APP_RACE_H
#include "mecanum_chassis.h"
#include "race_fsm.h"
#include "race_task_plan.h"

typedef enum {
    APP_MATERIAL_IDLE = 0,
    APP_MATERIAL_COMMON_READY,
    APP_MATERIAL_MOVE_X2100,
    APP_MATERIAL_TURN_90,
    APP_MATERIAL_MOVE_Y1100,
    APP_MATERIAL_CAMERA_PREPARE,
    APP_MATERIAL_AREA_DETECT,
    APP_MATERIAL_AREA_ALIGN,
    APP_MATERIAL_MATERIAL_ALIGN,
    APP_MATERIAL_ALIGN_COMPLETE,
    APP_MATERIAL_FAULT
} AppMaterialState;

void AppRace_Init(MecanumChassis_t *chassis);
void AppRace_Poll(void);
uint8_t AppRace_ControlTick(void);
uint8_t AppRace_StartRingAlign(uint8_t target);
const RaceFsm *AppRace_Status(void);
const RaceTaskPlan *AppRace_TaskPlan(void);
/* Current executable flow has ended (also true for fault/abort).
 * RACE_DONE alone only means the material route has ended. */
uint8_t AppRace_LogFinished(void);
AppMaterialState AppRace_MaterialState(void);
#endif
