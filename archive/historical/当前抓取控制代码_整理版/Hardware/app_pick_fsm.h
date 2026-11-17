#ifndef APP_PICK_FSM_H
#define APP_PICK_FSM_H

#include <stdint.h>

typedef enum
{
    PICK_IDLE = 0,
    PICK_RESETTING,
    PICK_FORWARD_HOME,
    PICK_FORWARD_TO_APPROACH,
    PICK_FORWARD_TO_GRAB,
    PICK_FORWARD_CLOSE,
    PICK_FORWARD_TURNTABLE_FIRST,
    PICK_FORWARD_RAISE,
    PICK_FORWARD_OPEN,
    PICK_FORWARD_RESET_ROTATE,
    PICK_REVERSE_HOME,
    PICK_REVERSE_RAISE,
    PICK_REVERSE_TURN,
    PICK_REVERSE_CLOSE,
    PICK_REVERSE_RAISE_AFTER_CLOSE,
    PICK_REVERSE_HOME_ROTATE,
    PICK_REVERSE_RELEASE,
    PICK_REVERSE_RESET_ZERO,
    PICK_DONE,
    PICK_FAULT,
    PICK_ABORTED
} PickState;

typedef enum
{
    PICK_ERROR_NONE = 0,
    PICK_ERROR_CONFIG,
    PICK_ERROR_SERVO_TIMEOUT,
    PICK_ERROR_STEPPER_TIMEOUT,
    PICK_ERROR_COMMAND
} PickError;

typedef struct
{
    float rotate_home_deg;
    float rotate_place_deg;
    float gripper_open_deg;
    float gripper_preclose_deg;
    float gripper_close_deg;
    int32_t forward_approach_steps;
    int32_t forward_to_grab_steps;
    int32_t forward_raise_steps;
    int32_t forward_reset_steps;
    int32_t reverse_raise_steps;
    int32_t reverse_to_grab_steps;
    int32_t reverse_raise_after_close_steps;
    int32_t reverse_to_ground_steps;
    int32_t reverse_reset_steps;
    uint32_t lift_frequency_hz;
    float lift_accel_rpms;
    float rotate_speed_dps;
    float rotate_accel_dps2;
    float gripper_speed_dps;
    float gripper_accel_dps2;
    uint32_t lift_before_rotate_done_ms;
    uint32_t servo_timeout_ms;
    uint32_t lift_timeout_ms;
    uint32_t settle_ms;
    uint32_t direction_settle_ms;
    uint32_t forward_rotate_trigger_steps;
    uint32_t reverse_rotate_trigger_steps;
    uint32_t reverse_home_rotate_trigger_steps;
    uint8_t forward_cycle_count;
    float turntable_servo_speed_dps;
    float turntable_servo_accel_dps2;
} PickConfig;

typedef struct
{
    PickState state;
    PickError error;
    uint32_t state_since_ms;
    uint32_t rotate_started_ms;
    uint8_t lift_started;
    uint8_t rotate_started;
    uint8_t gripper_started;
    uint8_t turntable_started;
    uint8_t turntable_home_started;
    uint8_t reverse_turntable_started;
    uint8_t forward_cycle_index;
    float reverse_turntable_angle_deg;
    PickConfig config;
} PickFsm;

void PickFsm_Init(PickFsm *fsm, const PickConfig *config);
uint8_t PickFsm_Start(PickFsm *fsm, uint32_t now);
uint8_t PickFsm_StartReverse(PickFsm *fsm, uint32_t now);
uint8_t PickFsm_SetReverseTurntableAngle(PickFsm *fsm, float angle_deg);
void PickFsm_Reset(PickFsm *fsm, uint32_t now);
void PickFsm_Poll(PickFsm *fsm, uint32_t now);
void PickFsm_Abort(PickFsm *fsm);
uint8_t PickFsm_IsActive(const PickFsm *fsm);
uint8_t PickFsm_IsDone(const PickFsm *fsm);
uint8_t PickFsm_IsReady(const PickFsm *fsm);

#endif /* APP_PICK_FSM_H */
