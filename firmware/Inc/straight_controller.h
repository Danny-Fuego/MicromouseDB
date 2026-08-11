#ifndef STRAIGHT_CONTROLLER_H
#define STRAIGHT_CONTROLLER_H
#include <stdint.h>
#include "sensors.h"

void StraightController_Reset(void);

void StraightController_Update(float baseSpeed_mm_s,
                               float leftDist_mm,
                               float rightDist_mm,
                               uint8_t isDecelerating,
                               float *leftTarget_mm_s,
                               float *rightTarget_mm_s,
                               float dt_sec);

float StraightController_GetError(void);
float StraightController_GetCorrection(void);

#endif
