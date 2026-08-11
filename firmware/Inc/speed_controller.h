#ifndef SPPED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "main.h"
#include <stdint.h>
#include <math.h>
#include "motor.h"
#include "encoder.h"

void Controller_Init(void);
void Controller_Reset(void);

void Controller_UpdateSpeed(float targetLeft_mm_s,
                             float targetRight_mm_s,
                             float dt_sec,
                             uint8_t isDecelerating,
                             float targetSpeed_mm_s,
                             float maxCruiseSpeed_mm_s,
                             float totalDist_mm);

float Controller_GetLeftMeasuredSpeed(void);
float Controller_GetRightMeasuredSpeed(void);

int16_t Controller_GetLeftPWM(void);
int16_t Controller_GetRightPWM(void);

#endif
