#include "board.h"
#include "app_race.h"
#include "app_stepper_openloop.h"
#include "race_config.h"
#include "vision.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

uint8_t OLED_GRAM[128][8];
static Motor_t s_motors[4];
static MecanumChassis_t s_chassis;
static uint32_t s_now, s_frames, s_irq, s_position_calls, s_stops, s_scans;
static uint8_t s_key_down, s_imu_ok = 1U, s_bias_ready = 1U, s_link_ok = 1U, s_qr;
static VisionContext s_vision;

uint32_t __get_PRIMASK(void) { return s_irq; }
void __disable_irq(void) { s_irq = 1U; }
void __set_PRIMASK(uint32_t mask) { s_irq = mask; }
uint32_t HAL_GetTick(void) { return s_now; }
int HAL_GPIO_ReadPin(void *port, uint16_t pin) { (void)port; (void)pin; return !s_key_down; }
bool app_imu_ok(void) { return s_imu_ok; }
bool app_imu_bias_ready(void) { return s_bias_ready; }
uint32_t app_imu_frames(void) { return s_frames; }
void Motor_Stop(Motor_t *motor) { motor->output = 0.0f; s_stops++; }
void Mecanum_Stop(MecanumChassis_t *chassis) { chassis->mode = 0U; }
void Mecanum_SetSpeedLimit(MecanumChassis_t *chassis, float linear, float angular)
{ chassis->max_speed = linear; chassis->max_omega = angular; }
void Mecanum_SetTolerance(MecanumChassis_t *chassis, float position, float yaw)
{ chassis->position_tol = position; chassis->theta_tol = yaw; }
void Mecanum_SetPositionMode(MecanumChassis_t *chassis, float x, float y, float yaw)
{
    assert(s_irq == 1U && chassis->body_frame == 0U);
    chassis->x_target = x;
    chassis->y_target = y;
    chassis->theta_target = yaw;
    chassis->mode = 1U;
    s_position_calls++;
}
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{ (void)x; (void)y; (void)chr; (void)size; (void)mode; }
void OLED_WR_Byte(uint8_t value, uint8_t command) { (void)value; (void)command; }
uint8_t RaceVision_Init(void) { return s_link_ok; }
uint8_t RaceVision_Healthy(void) { return s_link_ok; }
uint8_t RaceVision_BeginScan(void) { s_scans++; return s_link_ok; }
void RaceVision_EndScan(void) { s_qr = 0U; }
uint8_t RaceVision_Poll(char code[RACE_TASK_CODE_SIZE])
{
    if (!s_qr) return 0U;
    s_qr = 0U;
    memcpy(code, "452+321+254+312", RACE_TASK_CODE_SIZE);
    return 1U;
}
VisionContext *Vision_GetDefault(void) { return &s_vision; }
void Vision_SetRingChassis(VisionContext *vision, MecanumChassis_t *chassis)
{ (void)vision; (void)chassis; }
void Vision_SetRingCalibration(VisionContext *vision, float a, float b, float c,
                               float d, float e, float f, float g, float h, float i)
{ (void)vision; (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)g; (void)h; (void)i; }
uint8_t Vision_RingActive(const VisionContext *vision) { (void)vision; return 0U; }
uint8_t Vision_RingFailed(const VisionContext *vision) { (void)vision; return 0U; }
uint8_t Vision_RingSucceeded(const VisionContext *vision) { (void)vision; return 0U; }
int Vision_StartRingAlign(VisionContext *vision, uint8_t target, uint32_t now)
{ (void)vision; (void)target; (void)now; return 0; }
int Vision_Cancel(VisionContext *vision) { (void)vision; return 0; }
void Servo_Init(void) {}
void Servo_SetAngle(uint8_t id, float angle) { (void)id; (void)angle; }
void Servo_SetAngleRelativeLong(uint8_t id, float angle) { (void)id; (void)angle; }
void Servo_SetProfile(uint8_t id, float speed, float accel)
{ (void)id; (void)speed; (void)accel; }
void Servo_SetSpeedLimit(float speed) { (void)speed; }
void Servo_SetAccel(float accel) { (void)accel; }
void Servo_Disable(uint8_t id) { (void)id; }
void Servo_Poll(void) {}
uint8_t Servo_IsBusy(uint8_t id) { (void)id; return 0U; }
void AppStepper_Init(void) {}
HAL_StatusTypeDef AppStepper_MoveSteps(StepperAxis_t axis, int32_t steps, uint32_t frequency)
{ (void)axis; (void)steps; (void)frequency; return 0; }
HAL_StatusTypeDef AppStepper_MoveStepsProfile(StepperAxis_t axis, int32_t steps,
                                              uint32_t frequency, float accel)
{ (void)axis; (void)steps; (void)frequency; (void)accel; return 0; }
void AppStepper_Poll(void) {}
void AppStepper_Stop(StepperAxis_t axis) { (void)axis; }
void AppStepper_SetBrakeMode(StepperAxis_t axis, uint8_t trap_brake)
{ (void)axis; (void)trap_brake; }
uint8_t AppStepper_IsBusy(StepperAxis_t axis) { (void)axis; return 0U; }
uint32_t AppStepper_GetRemainingSteps(StepperAxis_t axis)
{ (void)axis; return 0U; }

static void tick(uint32_t duration)
{
    while (duration--) {
        s_now++;
        if (s_imu_ok) s_frames++;
        (void)AppRace_ControlTick();
        AppRace_Poll();
    }
}

static void init(uint8_t held)
{
    memset(&s_chassis, 0, sizeof(s_chassis));
    s_chassis.motor_lf = &s_motors[0];
    s_chassis.motor_rf = &s_motors[1];
    s_chassis.motor_lb = &s_motors[2];
    s_chassis.motor_rb = &s_motors[3];
    s_chassis.body_frame = 1U;
    s_key_down = held;
    s_now = s_frames = s_irq = s_position_calls = s_scans = 0U;
    s_bias_ready = s_imu_ok = s_link_ok = 1U;
    AppRace_Init(&s_chassis);
}

static void click(void)
{
    s_key_down = 1U;
    tick(40U);
    s_key_down = 0U;
    tick(40U);
}

static void start(void)
{
    init(0U);
    tick(RACE_WARMUP_MS + 1U);
    click();
    assert(AppRace_Status()->state == RACE_RUNNING);
}

int main(void)
{
    uint32_t calls, stops;
    init(1U);
    tick(RACE_WARMUP_MS + 1U);
    s_key_down = 0U;
    tick(40U);
    assert(AppRace_Status()->state == RACE_READY && s_position_calls == 0U);
    s_key_down = 1U;
    tick(750U);
    s_key_down = 0U;
    tick(40U);
    assert(AppRace_Status()->zone == 2U && s_position_calls == 0U);
    click();
    assert(AppRace_Status()->state == RACE_RUNNING && s_position_calls == 1U);
    tick(50U);
    assert(s_position_calls == 1U);
    click();
    assert(AppRace_Status()->state == RACE_ABORTED && !AppRace_ControlTick());
    click();
    assert(AppRace_Status()->state == RACE_ABORTED);
    init(0U);
    tick(40U);
    click();
    assert(AppRace_Status()->state == RACE_READY);
    s_bias_ready = 0U;
    tick(RACE_WARMUP_MS);
    click();
    assert(AppRace_Status()->state == RACE_READY);
    start();
    calls = s_position_calls;
    stops = s_stops;
    s_motors[0].output = 100.0f;
    s_now += RACE_IMU_STALE_MS;
    assert(!AppRace_ControlTick());
    assert(s_stops == stops + 4U && s_motors[0].output == 0.0f);
    AppRace_Poll();
    assert(AppRace_Status()->error == RACE_ERROR_LOOP_STALL && s_position_calls == calls);
    start();
    s_imu_ok = 0U;
    tick(1U);
    assert(AppRace_Status()->error == RACE_ERROR_IMU && !AppRace_ControlTick());
    start();
    s_link_ok = 0U;
    tick(1U);
    assert(AppRace_Status()->error == RACE_ERROR_LINK && !AppRace_ControlTick());
    start();
    while (AppRace_Status()->action_index < 4U) {
        uint8_t index = AppRace_Status()->action_index;
        const RaceFsm *race = AppRace_Status();
        s_chassis.x_pos = race->target_x;
        s_chassis.y_pos = race->target_y;
        s_chassis.theta_pos = race->target_yaw;
        tick(race->actions[index].hold_ms + 2U);
        assert(AppRace_Status()->action_index == index + 1U);
    }
    /* Complete the scan action before exercising the material route. */
    {
        const RaceFsm *race = AppRace_Status();
        s_chassis.x_pos = race->target_x;
        s_chassis.y_pos = race->target_y;
        s_chassis.theta_pos = race->target_yaw;
        s_qr = 1U;
        tick(1U);
        assert(AppRace_Status()->action_index == 5U);
    }
    while (AppRace_Status()->action_index < 7U) {
        uint8_t index = AppRace_Status()->action_index;
        const RaceFsm *race = AppRace_Status();
        s_chassis.x_pos = race->target_x;
        s_chassis.y_pos = race->target_y;
        s_chassis.theta_pos = race->target_yaw;
        tick(race->actions[index].hold_ms + 2U);
        assert(AppRace_Status()->action_index == index + 1U);
    }
    {
        const RaceFsm *race = AppRace_Status();
        s_chassis.x_pos = race->target_x;
        s_chassis.y_pos = race->target_y;
        s_chassis.theta_pos = race->target_yaw;
        tick(race->actions[7].hold_ms + 2U);
    }
    assert(s_scans == 1U && s_position_calls == 5U && s_chassis.mode == 0U);
    assert(AppRace_Status()->state == RACE_DONE && !AppRace_ControlTick());
    assert(strcmp(AppRace_Status()->task_code, "452+321+254+312") == 0);
    puts("PASS: boot-held/warmup keys, zone select, one-shot start/abort, PID transition ownership, stall/IMU/link stops, scan completion");
    return 0;
}
