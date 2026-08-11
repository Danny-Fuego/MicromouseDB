#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>

void MotionProfile_Reset(void);

void MotionProfile_SetMove(float distance_mm, float maxSpeed_mm_s_input);

float MotionProfile_Update(float avgDist_mm,
                           float actualSpeed_mm_s,
                           float dt_sec);

float MotionProfile_GetTargetDistance(void);
float MotionProfile_GetMaxSpeed(void);
float MotionProfile_GetProfileSpeed(void);

uint8_t MotionProfile_IsDecelerating(void);

#endif
