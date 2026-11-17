#ifndef APP_PICK_CONFIG_H
#define APP_PICK_CONFIG_H

/* The tray is the PC7 servo. The second D36 stepper axis is never used. */
#define PICK_TURNTABLE_ENABLE                 1U
#define PICK_TURNTABLE_ONLY                   0U

#define PICK_ROTATE_HOME_DEG                  0.0f
#define PICK_ROTATE_PLACE_DEG                 180.0f
#define PICK_GRIPPER_OPEN_DEG                0.0f
#define PICK_GRIPPER_PRECLOSE_DEG           110.0f
#define PICK_GRIPPER_CLOSE_DEG              140.0f

/* Lift axis: 800 pulses per 12 mm, four microsteps, zero is manual. */
#define PICK_LIFT_APPROACH_STEPS           (-5333)
#define PICK_LIFT_TO_GRAB_STEPS             3733
#define PICK_LIFT_FORWARD_RAISE_STEPS      (-9733)
#define PICK_LIFT_FORWARD_RESET_STEPS      11333
#define PICK_LIFT_REVERSE_RAISE_STEPS     (-11667)
#define PICK_LIFT_REVERSE_TO_GRAB_STEPS     2267
#define PICK_LIFT_REVERSE_RAISE_AFTER_CLOSE_STEPS 2267
#define PICK_LIFT_REVERSE_TO_GROUND_STEPS  (-10067)
#define PICK_LIFT_REVERSE_RESET_STEPS       1600

#define PICK_LIFT_FREQUENCY_HZ              5000U
#define PICK_LIFT_ACCEL_RPMS                750.0f
#define PICK_ROTATE_SPEED_DPS               127.764f
#define PICK_ROTATE_ACCEL_DPS2              191.646f
#define PICK_GRIPPER_SPEED_DPS              720.0f
#define PICK_GRIPPER_ACCEL_DPS2             1000.0f
#define PICK_TURNTABLE_SPEED_DPS             60.0f
#define PICK_TURNTABLE_ACCEL_DPS2            120.0f

#define PICK_LIFT_BEFORE_ROTATE_DONE_MS    1000U
#define PICK_SERVO_TIMEOUT_MS             10000U
#define PICK_LIFT_TIMEOUT_MS              60000U
#define PICK_SETTLE_MS                      500U
#define PICK_DIRECTION_SETTLE_MS             30U
#define PICK_FORWARD_ROTATE_TRIGGER_STEPS 4000U
#define PICK_REVERSE_ROTATE_TRIGGER_STEPS 5000U
#define PICK_REVERSE_HOME_ROTATE_TRIGGER_STEPS 1000U
#define PICK_FORWARD_CYCLE_COUNT             3U

/* Reverse operation selects which tray position supplies the material. */
#define PICK_REVERSE_TURNTABLE_ANGLE_DEG     0.0f

#endif
