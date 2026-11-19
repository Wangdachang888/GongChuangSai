#include "race_fsm.h"
#include "race_config.h"
#include "vision.h"
#include "race_task_plan.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart3 = {USART3};
static uint32_t s_irq;
uint32_t __get_PRIMASK(void) { return s_irq; }
void __disable_irq(void) { s_irq = 1U; }
void __set_PRIMASK(uint32_t mask) { s_irq = mask; }
uint32_t HAL_GetTick(void) { return 0U; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length)
{ (void)uart; (void)byte; (void)length; return HAL_OK; }
int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length)
{ (void)uart; (void)byte; (void)length; return HAL_OK; }
void Mecanum_Stop(MecanumChassis_t *chassis) { (void)chassis; }
void Mecanum_SetPositionMode(MecanumChassis_t *chassis, float x, float y, float yaw)
{ (void)chassis; (void)x; (void)y; (void)yaw; }
void Mecanum_GetPosition(MecanumChassis_t *chassis, float *x, float *y, float *theta)
{ (void)chassis; if (x) *x = 0.0f; if (y) *y = 0.0f; if (theta) *theta = 0.0f; }
void Mecanum_GetVelocity(MecanumChassis_t *chassis, float *vx, float *vy, float *omega)
{ (void)chassis; if (vx) *vx = 0.0f; if (vy) *vy = 0.0f; if (omega) *omega = 0.0f; }
uint8_t Mecanum_IsTargetReached(MecanumChassis_t *chassis) { (void)chassis; return 1U; }

static RaceFeedback stationary(void)
{
    RaceFeedback value = {0};
    value.imu_valid = 1U;
    return value;
}

static void at_target(const RaceFsm *race, RaceFeedback *current)
{
    current->x = race->target_x;
    current->y = race->target_y;
    current->yaw = race->target_yaw;
    current->vx = current->vy = current->omega = 0.0f;
}

static void test_routes(void)
{
    uint8_t zone;
    for (zone = 1U; zone <= 2U; zone++) {
        RaceFsm race;
        RaceFeedback current = stationary();
        uint32_t now = 50U;
        Race_Init(&race);
        assert(Race_SelectZone(&race, zone));
        assert(Race_StartQrRoute(&race, &current, now));
        assert(fabsf(race.target_y - 0.140f) < 0.0001f);
        assert(race.actions[2].scan_start != 0U && race.actions[2].scan_leg == 1U);
        assert(race.actions[4].scan_leg == 2U);
        assert(!Race_SelectZone(&race, 3U - zone));
        assert(!Race_SubmitQr(&race, race.scan_session, "452+321+254+312"));
        while (race.state == RACE_RUNNING) {
            uint8_t previous = race.action_index;
            if (race.actions[previous].scan_start != 0U) {
                assert(!Race_SubmitQr(&race, race.scan_session,
                                      "452+321+253+312"));
                assert(Race_SubmitQr(&race, race.scan_session,
                                     "452+321+254+312"));
            }
            at_target(&race, &current);
            Race_Tick(&race, &current, now);
            if (race.state != RACE_RUNNING) break;
            if (race.actions[previous].scan_leg != 1U || race.qr_valid)
                assert(race.action_index >= previous);
            else
                assert(race.action_index == previous);
            if (race.action_index != previous) {
                now++;
                continue;
            }
            now += race.actions[previous].hold_ms;
            at_target(&race, &current);
            Race_Tick(&race, &current, now);
            assert(race.action_index > previous || race.state == RACE_DONE);
            now++;
        }
        assert(race.state == RACE_DONE);
        assert(race.scan_session != 0U);
        assert(!Race_SubmitQr(&race, race.scan_session + 1U, "452+321+254+312"));
        assert(!Race_SubmitQr(&race, race.scan_session, "452+321+254+312"));
        assert(race.colors[1][0] == 2U && race.positions[0][0] == 3U);
        assert(Race_QrCandidateZone(&race) == 2U);
        assert(!Race_StartQrRoute(&race, &current, now + 2U));
        current.imu_valid = 0U;
        Race_Tick(&race, &current, now + 100000U);
        assert(race.state == RACE_DONE);
    }
}

static void test_motion_guards(void)
{
    RaceFsm race;
    RaceFeedback current = stationary();
    RaceAction action = {RACE_ACTION_MOVE, 1.0f, 0.0f, 0.0f,
                         1000U, 200U, 0U, 0U};
    Race_Init(&race);
    assert(Race_RunActions(&race, &action, 1U, &current, 0U));
    at_target(&race, &current);
    current.vx = 0.1f;
    Race_Tick(&race, &current, 100U);
    assert(!race.holding);
    current.vx = 0.0f;
    Race_Tick(&race, &current, 200U);
    current.y = 0.1f;
    Race_Tick(&race, &current, 300U);
    assert(!race.holding);
    current.y = 0.0f;
    Race_Tick(&race, &current, 400U);
    Race_Tick(&race, &current, 599U);
    assert(race.state == RACE_RUNNING);
    Race_Tick(&race, &current, 600U);
    assert(race.state == RACE_DONE);
    Race_Init(&race);
    current = stationary();
    current.x = 2.0f;
    current.y = 3.0f;
    current.yaw = 1.57079632679f;
    assert(Race_RunActions(&race, &action, 1U, &current, 0U));
    assert(fabsf(race.target_x - 2.0f) < 0.0001f);
    assert(fabsf(race.target_y - 4.0f) < 0.0001f);
    Race_Abort(&race);
    Race_Tick(&race, &current, 2000U);
    assert(race.state == RACE_ABORTED);
    assert(!Race_StartQrRoute(&race, &current, 2000U));
}

static void test_faults(void)
{
    RaceFsm race;
    RaceFeedback current = stationary();
    RaceAction action = {RACE_ACTION_SCAN, 0.0f, 0.0f, 0.0f,
                         1000U, 0U, 0U, 0U};
    uint32_t start = 0xFFFFFF00U;
    Race_Init(&race);
    assert(Race_RunActions(&race, &action, 1U, &current, start));
    Race_Tick(&race, &current, start + 999U);
    assert(race.state == RACE_RUNNING);
    Race_Tick(&race, &current, start + 1000U);
    assert(race.error == RACE_ERROR_SCAN_TIMEOUT);
    Race_Init(&race);
    action.kind = RACE_ACTION_MOVE;
    action.x = 1.0f;
    assert(Race_RunActions(&race, &action, 1U, &current, 0U));
    Race_Tick(&race, &current, 1000U);
    assert(race.error == RACE_ERROR_MOVE_TIMEOUT);
    Race_Init(&race);
    assert(Race_StartQrRoute(&race, &current, 0U));
    current.imu_valid = 0U;
    Race_Tick(&race, &current, 1U);
    assert(race.error == RACE_ERROR_IMU);
    Race_Init(&race);
    current = stationary();
    current.x = NAN;
    assert(!Race_StartQrRoute(&race, &current, 0U));
    assert(race.error == RACE_ERROR_IMU);
    Race_Init(&race);
    current = stationary();
    current.vx = 0.1f;
    assert(!Race_StartQrRoute(&race, &current, 0U));
    assert(race.state == RACE_READY);
    current.vx = 0.0f;
    action.hold_ms = action.timeout_ms;
    assert(!Race_RunActions(&race, &action, 1U, &current, 0U));
    assert(race.error == RACE_ERROR_CONFIG);
}

static void test_protocol(void)
{
    static const char *const invalid[] = {
        "#SEQ,3,2,1,2,1,3,3,1,2",
        "#SEQ,452,321,254,312",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,2,1",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,2junk",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,22",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,-2",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,0",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,6",
        "#SEQ,4,5,2,3,2,1,2,5,3,3,1,2",
        "#SEQ,4,4,2,3,2,1,2,4,4,3,1,2",
        "#DONE", "#OBJ,1,160,120,40,35,1400,215", ""
    };
    const char *valid = "#SEQ,4,5,2,3,2,1,2,5,4,3,1,2\r\n";
    char code[RACE_TASK_CODE_SIZE] = "unchanged";
    RaceVisionDecoder decoder;
    unsigned index, accepted = 0U;
    for (index = 0U; index < sizeof(invalid) / sizeof(invalid[0]); index++) {
        assert(!RaceVision_ParseSeq(invalid[index], code));
        assert(strcmp(code, "unchanged") == 0);
    }
    assert(RaceVision_ParseSeq("#SEQ, 4,5,2,3,2,1,2,5,4,3,1,2 ", code));
    assert(strcmp(code, "452+321+254+312") == 0);
    RaceVisionDecoder_Reset(&decoder);
    for (index = 0U; valid[index]; index++) accepted += RaceVisionDecoder_Push(&decoder, valid[index], code);
    assert(accepted == 1U);
    RaceVisionDecoder_Reset(&decoder);
    assert(!RaceVisionDecoder_Push(&decoder, '#', code));
    for (index = 0U; index < 200U; index++) assert(!RaceVisionDecoder_Push(&decoder, 'X', code));
    for (index = 0U; valid[index]; index++) assert(!RaceVisionDecoder_Push(&decoder, valid[index], code));
    accepted = 0U;
    for (index = 0U; valid[index]; index++) accepted += RaceVisionDecoder_Push(&decoder, valid[index], code);
    assert(accepted == 1U);
    RaceVisionDecoder_Reset(&decoder);
    assert(!RaceVisionDecoder_Push(&decoder, '#', code));
    assert(!RaceVisionDecoder_Push(&decoder, 0U, code));
    for (index = 0U; valid[index]; index++) assert(!RaceVisionDecoder_Push(&decoder, valid[index], code));
}

static void test_task_plan(void)
{
    static const uint8_t colors[2][3] = {{4U, 5U, 2U}, {2U, 5U, 4U}};
    static const uint8_t positions[2][3] = {{3U, 2U, 1U}, {2U, 1U, 3U}};
    RaceTaskPlan plan;
    const TaskAction *action;
    uint8_t pick_count = 0U;
    assert(RaceTaskPlan_Build(&plan, colors, positions));
    assert(plan.count == 34U);
    assert(RaceTaskPlan_Start(&plan));
    action = RaceTaskPlan_Current(&plan);
    assert(action->kind == TASK_ACTION_PICK && action->batch == 0U &&
           action->color == 4U);
    while (!RaceTaskPlan_Done(&plan)) {
        action = RaceTaskPlan_Current(&plan);
        if (action->kind == TASK_ACTION_PICK) pick_count++;
        if (action->kind == TASK_ACTION_STACK_STORAGE) {
            if (action->color == 2U) assert(action->position == 1U);
            if (action->color == 5U) assert(action->position == 2U);
            if (action->color == 4U) assert(action->position == 3U);
        }
        assert(RaceTaskPlan_Advance(&plan));
    }
    assert(pick_count == 6U && RaceTaskPlan_Current(&plan) == 0);
}

int main(void)
{
    test_routes();
    test_motion_guards();
    test_faults();
    test_protocol();
    test_task_plan();
    puts("PASS: both routes, pose transform, dwell/speed, abort/latch, time wrap, faults, strict protocol, stream recovery");
    return 0;
}
