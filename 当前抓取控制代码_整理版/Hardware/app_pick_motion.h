#ifndef APP_PICK_MOTION_H
#define APP_PICK_MOTION_H

#include "app_pick_fsm.h"

typedef enum
{
    PICK_GRIPPER_OPEN = 0,
    PICK_GRIPPER_PRECLOSE,
    PICK_GRIPPER_CLOSED
} PickGripperPose;

typedef enum
{
    PICK_HOLDER_HOME = 0,
    PICK_HOLDER_PLACE
} PickHolderPose;

typedef enum
{
    PICK_LIFT_FORWARD_APPROACH = 0,
    PICK_LIFT_FORWARD_TO_GRAB,
    PICK_LIFT_FORWARD_RAISE,
    PICK_LIFT_FORWARD_RESET,
    PICK_LIFT_REVERSE_RAISE,
    PICK_LIFT_REVERSE_TO_GRAB,
    PICK_LIFT_REVERSE_RAISE_AFTER_CLOSE,
    PICK_LIFT_REVERSE_TO_GROUND,
    PICK_LIFT_REVERSE_RESET
} PickLiftMove;

void PickMotion_SetGripper(const PickConfig *config, PickGripperPose pose);
void PickMotion_SetHolder(const PickConfig *config, PickHolderPose pose);
void PickMotion_SetTrayHome(const PickConfig *config);
uint8_t PickMotion_SetForwardTray(const PickConfig *config, uint8_t cycle_index);
void PickMotion_SetReverseTray(const PickConfig *config, float angle_deg);

uint8_t PickMotion_StartLift(const PickConfig *config,
                             PickLiftMove move,
                             uint8_t trap_brake);
void PickMotion_StopLift(void);

#endif /* APP_PICK_MOTION_H */
