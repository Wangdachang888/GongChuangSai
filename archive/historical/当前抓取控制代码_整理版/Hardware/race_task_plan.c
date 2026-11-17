#include "race_task_plan.h"

static uint8_t add(RaceTaskPlan *plan, TaskActionKind kind, uint8_t batch,
                   uint8_t item, uint8_t color, uint8_t position)
{
    TaskAction *action;
    if (plan->count >= RACE_TASK_PLAN_MAX) return 0U;
    action = &plan->actions[plan->count++];
    action->kind = kind;
    action->batch = batch;
    action->item = item;
    action->color = color;
    action->position = position;
    return 1U;
}

static uint8_t add_batch(RaceTaskPlan *plan, uint8_t batch,
                         const uint8_t colors[2][3],
                         const uint8_t positions[2][3])
{
    uint8_t item;
    for (item = 0U; item < 3U; item++) {
        if (!add(plan, TASK_ACTION_PICK, batch, item,
                 colors[batch][item], 0U)) return 0U;
    }
    if (!add(plan, TASK_ACTION_GO_ROUGH, batch, 0U, 0U, 0U)) return 0U;
    for (item = 0U; item < 3U; item++) {
        if (!add(plan, TASK_ACTION_PLACE_ROUGH, batch, item,
                 colors[batch][item], positions[batch][item])) return 0U;
    }
    for (item = 0U; item < 3U; item++) {
        if (!add(plan, TASK_ACTION_RETRIEVE_ROUGH, batch, item,
                 colors[batch][item], positions[batch][item])) return 0U;
    }
    if (!add(plan, TASK_ACTION_GO_STORAGE, batch, 0U, 0U, 0U)) return 0U;
    for (item = 0U; item < 3U; item++) {
        if (!add(plan, TASK_ACTION_PLACE_STORAGE, batch, item,
                 colors[batch][item], positions[batch][item])) return 0U;
    }
    return 1U;
}

uint8_t RaceTaskPlan_Build(RaceTaskPlan *plan,
                           const uint8_t colors[2][3],
                           const uint8_t positions[2][3])
{
    uint8_t item;
    if (plan == 0 || colors == 0 || positions == 0) return 0U;
    plan->valid = 0U;
    plan->index = 0U;
    plan->count = 0U;
    if (!add_batch(plan, 0U, colors, positions)) return 0U;
    if (!add(plan, TASK_ACTION_GO_MATERIAL, 1U, 0U, 0U, 0U)) return 0U;
    if (!add_batch(plan, 1U, colors, positions)) return 0U;
    if (!add(plan, TASK_ACTION_GO_STORAGE, 1U, 0U, 0U, 0U)) return 0U;
    for (item = 0U; item < 3U; item++) {
        /* Second-batch item is stacked over the first-batch item with
         * the same color; position is the first batch's storage slot. */
        uint8_t first;
        for (first = 0U; first < 3U; first++) {
            if (colors[1][item] == colors[0][first]) break;
        }
        if (first >= 3U || !add(plan, TASK_ACTION_STACK_STORAGE, 1U, item,
                                colors[1][item], positions[0][first])) return 0U;
    }
    if (!add(plan, TASK_ACTION_RETURN_START, 1U, 0U, 0U, 0U)) return 0U;
    plan->valid = 1U;
    return 1U;
}

uint8_t RaceTaskPlan_Start(RaceTaskPlan *plan)
{
    if (plan == 0 || !plan->valid || plan->count == 0U) return 0U;
    plan->index = 0U;
    return 1U;
}

const TaskAction *RaceTaskPlan_Current(const RaceTaskPlan *plan)
{
    if (plan == 0 || !plan->valid || plan->index >= plan->count) return 0;
    return &plan->actions[plan->index];
}

uint8_t RaceTaskPlan_Advance(RaceTaskPlan *plan)
{
    if (plan == 0 || !plan->valid || plan->index >= plan->count) return 0U;
    plan->index++;
    return 1U;
}

uint8_t RaceTaskPlan_Done(const RaceTaskPlan *plan)
{
    return plan != 0 && plan->valid && plan->index >= plan->count;
}
