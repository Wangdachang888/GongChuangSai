#ifndef RACE_TASK_PLAN_H
#define RACE_TASK_PLAN_H

#include <stdint.h>

#define RACE_TASK_PLAN_MAX 40U

typedef enum {
    TASK_ACTION_PICK = 0,
    TASK_ACTION_GO_ROUGH,
    TASK_ACTION_PLACE_ROUGH,
    TASK_ACTION_RETRIEVE_ROUGH,
    TASK_ACTION_GO_STORAGE,
    TASK_ACTION_PLACE_STORAGE,
    TASK_ACTION_GO_MATERIAL,
    TASK_ACTION_STACK_STORAGE,
    TASK_ACTION_RETURN_START
} TaskActionKind;

typedef struct {
    TaskActionKind kind;
    uint8_t batch;
    uint8_t item;
    uint8_t color;
    uint8_t position;
} TaskAction;

typedef struct {
    uint8_t valid;
    uint8_t index;
    uint8_t count;
    TaskAction actions[RACE_TASK_PLAN_MAX];
} RaceTaskPlan;

uint8_t RaceTaskPlan_Build(RaceTaskPlan *plan,
                           const uint8_t colors[2][3],
                           const uint8_t positions[2][3]);
uint8_t RaceTaskPlan_Start(RaceTaskPlan *plan);
const TaskAction *RaceTaskPlan_Current(const RaceTaskPlan *plan);
uint8_t RaceTaskPlan_Advance(RaceTaskPlan *plan);
uint8_t RaceTaskPlan_Done(const RaceTaskPlan *plan);

#endif
