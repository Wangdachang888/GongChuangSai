#include "app_pick_motion.h"
#include "app_stepper_openloop.h"
#include "bsp_servo.h"

#define PICK_GRIPPER_SERVO  SERVO_1
#define PICK_HOLDER_SERVO   SERVO_2
#define PICK_TRAY_SERVO     SERVO_3
#define PICK_LIFT_AXIS      STEPPER_AXIS_1

static void set_servo(uint8_t id, float angle_deg,
                      float speed_dps, float accel_dps2)
{
    Servo_SetProfile(id, speed_dps, accel_dps2);
    Servo_SetAngle(id, angle_deg);
}

void PickMotion_SetGripper(const PickConfig *config, PickGripperPose pose)
{
    float angle_deg;

    if (config == 0) return;
    if (pose == PICK_GRIPPER_OPEN) {
        angle_deg = config->gripper_open_deg;
    } else if (pose == PICK_GRIPPER_PRECLOSE) {
        angle_deg = config->gripper_preclose_deg;
    } else {
        angle_deg = config->gripper_close_deg;
    }
    set_servo(PICK_GRIPPER_SERVO, angle_deg,
              config->gripper_speed_dps, config->gripper_accel_dps2);
}

void PickMotion_SetHolder(const PickConfig *config, PickHolderPose pose)
{
    float angle_deg;

    if (config == 0) return;
    angle_deg = (pose == PICK_HOLDER_PLACE) ?
                config->rotate_place_deg : config->rotate_home_deg;
    set_servo(PICK_HOLDER_SERVO, angle_deg,
              config->rotate_speed_dps, config->rotate_accel_dps2);
}

void PickMotion_SetTrayHome(const PickConfig *config)
{
    if (config == 0) return;
    set_servo(PICK_TRAY_SERVO, 0.0f,
              config->turntable_servo_speed_dps,
              config->turntable_servo_accel_dps2);
}

uint8_t PickMotion_SetForwardTray(const PickConfig *config, uint8_t cycle_index)
{
    if (config == 0 || cycle_index >= config->forward_cycle_count) return 0U;
    Servo_SetProfile(PICK_TRAY_SERVO,
                     config->turntable_servo_speed_dps,
                     config->turntable_servo_accel_dps2);
    if (cycle_index == 0U) {
        Servo_SetAngle(PICK_TRAY_SERVO, 90.0f);
    } else if (cycle_index == 1U) {
        Servo_SetAngle(PICK_TRAY_SERVO, 0.0f);
    } else {
        Servo_SetAngleRelativeLong(PICK_TRAY_SERVO, 270.0f);
    }
    return 1U;
}

void PickMotion_SetReverseTray(const PickConfig *config, float angle_deg)
{
    if (config == 0) return;
    set_servo(PICK_TRAY_SERVO, angle_deg,
              config->turntable_servo_speed_dps,
              config->turntable_servo_accel_dps2);
}

static int32_t lift_steps(const PickConfig *config, PickLiftMove move)
{
    switch (move) {
    case PICK_LIFT_FORWARD_APPROACH:
        return config->forward_approach_steps;
    case PICK_LIFT_FORWARD_TO_GRAB:
        return config->forward_to_grab_steps;
    case PICK_LIFT_FORWARD_RAISE:
        return config->forward_raise_steps;
    case PICK_LIFT_FORWARD_RESET:
        return config->forward_reset_steps;
    case PICK_LIFT_REVERSE_RAISE:
        return config->reverse_raise_steps;
    case PICK_LIFT_REVERSE_TO_GRAB:
        return config->reverse_to_grab_steps;
    case PICK_LIFT_REVERSE_RAISE_AFTER_CLOSE:
        return config->reverse_raise_after_close_steps;
    case PICK_LIFT_REVERSE_TO_GROUND:
        return config->reverse_to_ground_steps;
    case PICK_LIFT_REVERSE_RESET:
        return config->reverse_reset_steps;
    default:
        return 0;
    }
}

uint8_t PickMotion_StartLift(const PickConfig *config,
                             PickLiftMove move,
                             uint8_t trap_brake)
{
    int32_t steps;

    if (config == 0) return 0U;
    steps = lift_steps(config, move);
    if (steps == 0) return 0U;
    AppStepper_SetBrakeMode(PICK_LIFT_AXIS, trap_brake);
    return (AppStepper_MoveStepsProfile(PICK_LIFT_AXIS,
                                        steps,
                                        config->lift_frequency_hz,
                                        config->lift_accel_rpms) == HAL_OK) ?
           1U : 0U;
}

void PickMotion_StopLift(void)
{
    AppStepper_Stop(PICK_LIFT_AXIS);
}
