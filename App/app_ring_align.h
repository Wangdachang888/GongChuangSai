#ifndef APP_RING_ALIGN_H
#define APP_RING_ALIGN_H

#include <stdint.h>
#include "mecanum_chassis.h"
#include "vision.h"

typedef struct
{
    float matrix_00;
    float matrix_01;
    float matrix_10;
    float matrix_11;
    float gain;
    float max_step_m;
    float min_step_m;
    float max_speed_mps;
    float max_omega_radps;
    float tolerance_x_px;
    float tolerance_y_px;
} RingAlignConfig;

typedef struct
{
    VisionContext *vision;
    MecanumChassis_t *chassis;
    RingAlignConfig config;
    uint32_t stable_since_ms;
    uint8_t active;
    uint8_t succeeded;
    uint8_t failed;
    uint8_t calibration_valid;
    uint8_t correction_count;
    uint8_t stable_started;
    uint8_t speed_limited;
    float saved_max_speed;
    float saved_max_omega;
} AppRingAlign;

void AppRingAlign_Init(AppRingAlign *align, VisionContext *vision,
                       MecanumChassis_t *chassis,
                       const RingAlignConfig *config);
uint8_t AppRingAlign_Start(AppRingAlign *align, uint8_t target, uint32_t now_ms);
void AppRingAlign_Poll(AppRingAlign *align, uint32_t now_ms);
void AppRingAlign_Cancel(AppRingAlign *align);
uint8_t AppRingAlign_IsActive(const AppRingAlign *align);
uint8_t AppRingAlign_Succeeded(const AppRingAlign *align);
uint8_t AppRingAlign_Failed(const AppRingAlign *align);

#endif
