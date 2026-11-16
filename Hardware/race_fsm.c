#include "race_fsm.h"
#include "race_config.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static float wrap_yaw(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

static uint8_t valid_feedback(const RaceFeedback *feedback)
{
    return feedback != NULL && feedback->imu_valid &&
           isfinite(feedback->x) && isfinite(feedback->y) && isfinite(feedback->yaw) &&
           isfinite(feedback->vx) && isfinite(feedback->vy) && isfinite(feedback->omega);
}

static uint8_t still(const RaceFeedback *feedback)
{
    return hypotf(feedback->vx, feedback->vy) < RACE_STILL_SPEED_MPS &&
           fabsf(feedback->omega) < RACE_STILL_OMEGA_RADPS;
}

static void enter_action(RaceFsm *race, uint32_t now)
{
    const RaceAction *action;
    float cosine, sine;
    race->holding = 0U;
    race->entered_ms = now;
    if (race->action_index >= race->action_count) {
        race->state = RACE_DONE;
        return;
    }
    action = &race->actions[race->action_index];
    if (action->kind == RACE_ACTION_MOVE || action->kind == RACE_ACTION_TURN) {
        cosine = cosf(race->origin_yaw);
        sine = sinf(race->origin_yaw);
        race->target_x = race->origin_x + cosine * action->x - sine * action->y;
        race->target_y = race->origin_y + sine * action->x + cosine * action->y;
        race->target_yaw = wrap_yaw(race->origin_yaw + action->yaw);
    }
    if (action->kind == RACE_ACTION_SCAN) {
        race->scan_session++;
        if (race->scan_session == 0U) race->scan_session = 1U;
        race->qr_valid = 0U;
    }
}

void Race_Init(RaceFsm *race)
{
    memset(race, 0, sizeof(*race));
    race->zone = RACE_DEFAULT_START_ZONE;
    race->state = RACE_READY;
}

uint8_t Race_SelectZone(RaceFsm *race, uint8_t zone)
{
    if (race->state != RACE_READY || zone < 1U || zone > 2U) return 0U;
    race->zone = zone;
    return 1U;
}

void Race_Fail(RaceFsm *race, RaceError error)
{
    race->error = error;
    race->state = RACE_FAULT;
}

void Race_Abort(RaceFsm *race)
{
    race->state = RACE_ABORTED;
}

uint8_t Race_RunActions(RaceFsm *race, const RaceAction *actions, uint8_t count,
                       const RaceFeedback *feedback, uint32_t now)
{
    uint8_t index;
    if (race->state != RACE_READY) return 0U;
    if (!valid_feedback(feedback)) {
        Race_Fail(race, RACE_ERROR_IMU);
        return 0U;
    }
    if (!still(feedback)) return 0U;
    if (actions == NULL || count == 0U || count > RACE_MAX_ACTIONS) {
        Race_Fail(race, RACE_ERROR_CONFIG);
        return 0U;
    }
    for (index = 0U; index < count; index++) {
        const RaceAction *action = &actions[index];
        if ((unsigned)action->kind > (unsigned)RACE_ACTION_SCAN ||
            action->timeout_ms == 0U || action->timeout_ms >= 0x80000000U ||
            action->hold_ms >= action->timeout_ms ||
            !isfinite(action->x) || !isfinite(action->y) || !isfinite(action->yaw)) {
            Race_Fail(race, RACE_ERROR_CONFIG);
            return 0U;
        }
    }
    memcpy(race->actions, actions, count * sizeof(*actions));
    race->action_count = count;
    race->action_index = 0U;
    race->origin_x = feedback->x;
    race->origin_y = feedback->y;
    race->origin_yaw = feedback->yaw;
    race->target_x = feedback->x;
    race->target_y = feedback->y;
    race->target_yaw = feedback->yaw;
    race->task_code[0] = '\0';
    race->qr_valid = 0U;
    race->error = RACE_ERROR_NONE;
    race->state = RACE_RUNNING;
    enter_action(race, now);
    return 1U;
}

uint8_t Race_StartQrRoute(RaceFsm *race, const RaceFeedback *feedback, uint32_t now)
{
    RaceAction actions[8];
    float along, inward, heading, cosine, sine;
    float local_x, local_y;
    if (race->zone < 1U || race->zone > 2U) return 0U;
    along = RACE_QR_FROM_BOTTOM_M - ((race->zone == 1U) ?
            RACE_FIELD_SIZE_M - RACE_START_CENTER_M : RACE_START_CENTER_M);
    inward = RACE_CENTER_FROM_WALL_M - RACE_START_CENTER_M;
    heading = race->zone == 1U ? RACE_ZONE1_HEADING_RAD : RACE_ZONE2_HEADING_RAD;
    cosine = cosf(heading);
    sine = sinf(heading);
    actions[0] = (RaceAction){RACE_ACTION_MOVE, sine * inward, cosine * inward, 0.0f,
                             RACE_MOVE_TIMEOUT_MS, RACE_ARRIVAL_HOLD_MS};
    actions[1] = (RaceAction){RACE_ACTION_MOVE, cosine * along + sine * inward,
                             -sine * along + cosine * inward, 0.0f,
                             RACE_MOVE_TIMEOUT_MS, RACE_ARRIVAL_HOLD_MS};
    actions[2] = actions[1];
    actions[2].kind = RACE_ACTION_TURN;
    actions[2].yaw = -heading;
    actions[3] = (RaceAction){RACE_ACTION_SETTLE, 0.0f, 0.0f, 0.0f, 3000U, RACE_SETTLE_MS};
    actions[4] = (RaceAction){RACE_ACTION_SCAN, 0.0f, 0.0f, 0.0f, RACE_SCAN_TIMEOUT_MS, 0U};
    local_x = cosine * (RACE_MATERIAL_APPROACH_X_M -
                        ((race->zone == 1U) ? RACE_FIELD_SIZE_M - RACE_START_CENTER_M : RACE_START_CENTER_M)) +
              sine * RACE_MATERIAL_CORRIDOR_Y_M;
    local_y = -sine * (RACE_MATERIAL_APPROACH_X_M -
                       ((race->zone == 1U) ? RACE_FIELD_SIZE_M - RACE_START_CENTER_M : RACE_START_CENTER_M)) +
              cosine * RACE_MATERIAL_CORRIDOR_Y_M;
    actions[5] = (RaceAction){RACE_ACTION_MOVE, local_x, local_y, 0.0f,
                              RACE_MATERIAL_TIMEOUT_MS, RACE_ARRIVAL_HOLD_MS};
    local_x = cosine * (RACE_MATERIAL_APPROACH_X_M -
                        ((race->zone == 1U) ? RACE_FIELD_SIZE_M - RACE_START_CENTER_M : RACE_START_CENTER_M)) +
              sine * RACE_MATERIAL_CENTER_Y_M;
    local_y = -sine * (RACE_MATERIAL_APPROACH_X_M -
                       ((race->zone == 1U) ? RACE_FIELD_SIZE_M - RACE_START_CENTER_M : RACE_START_CENTER_M)) +
              cosine * RACE_MATERIAL_CENTER_Y_M;
    actions[6] = (RaceAction){RACE_ACTION_MOVE, local_x, local_y, -heading,
                              RACE_MATERIAL_TIMEOUT_MS, RACE_ARRIVAL_HOLD_MS};
    actions[7] = (RaceAction){RACE_ACTION_SETTLE, 0.0f, 0.0f, 0.0f, 3000U, RACE_SETTLE_MS};
    return Race_RunActions(race, actions, 8U, feedback, now);
}

void Race_Tick(RaceFsm *race, const RaceFeedback *feedback, uint32_t now)
{
    const RaceAction *action;
    uint8_t reached;
    if (race->state != RACE_RUNNING) return;
    if (!valid_feedback(feedback)) {
        Race_Fail(race, RACE_ERROR_IMU);
        return;
    }
    action = &race->actions[race->action_index];
    if ((uint32_t)(now - race->entered_ms) >= action->timeout_ms) {
        Race_Fail(race, action->kind == RACE_ACTION_SCAN ?
                  RACE_ERROR_SCAN_TIMEOUT : RACE_ERROR_MOVE_TIMEOUT);
        return;
    }
    reached = still(feedback) &&
              hypotf(race->target_x - feedback->x, race->target_y - feedback->y) <= RACE_POSITION_TOL_M &&
              fabsf(wrap_yaw(race->target_yaw - feedback->yaw)) <= RACE_YAW_TOL_RAD;
    if (!reached) {
        race->holding = 0U;
        return;
    }
    if (action->kind == RACE_ACTION_SCAN) {
        if (!race->qr_valid) return;
    } else {
        if (!race->holding) {
            race->holding = 1U;
            race->hold_since_ms = now;
        }
        if ((uint32_t)(now - race->hold_since_ms) < action->hold_ms) return;
    }
    race->action_index++;
    enter_action(race, now);
}

uint8_t Race_ParseTask(const char *code, uint8_t colors[2][3], uint8_t positions[2][3])
{
    uint8_t group, index, mask, color_masks[2] = {0U, 0U};
    if (code == NULL || strlen(code) != 15U) return 0U;
    for (group = 0U; group < 4U; group++) {
        mask = 0U;
        for (index = 0U; index < 3U; index++) {
            char digit = code[group * 4U + index];
            uint8_t value;
            if (digit < '1' || digit > ((group & 1U) ? '3' : '6')) return 0U;
            value = (uint8_t)(digit - '0');
            if (mask & (1U << value)) return 0U;
            mask |= (uint8_t)(1U << value);
        }
        if (group < 3U && code[group * 4U + 3U] != '+') return 0U;
        if (!(group & 1U)) color_masks[group / 2U] = mask;
    }
    if (color_masks[0] != color_masks[1]) return 0U;
    for (group = 0U; group < 2U; group++) {
        for (index = 0U; index < 3U; index++) {
            colors[group][index] = (uint8_t)(code[group * 8U + index] - '0');
            positions[group][index] = (uint8_t)(code[group * 8U + 4U + index] - '0');
        }
    }
    return 1U;
}

uint8_t Race_SubmitQr(RaceFsm *race, uint32_t session, const char *code)
{
    if (race->state != RACE_RUNNING || race->actions[race->action_index].kind != RACE_ACTION_SCAN ||
        session != race->scan_session || race->qr_valid) return 0U;
    if (!Race_ParseTask(code, race->colors, race->positions)) return 0U;
    memcpy(race->task_code, code, RACE_TASK_CODE_SIZE);
    race->qr_valid = 1U;
    return 1U;
}
