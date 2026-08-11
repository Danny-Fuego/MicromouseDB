#include "motion_profile.h"
#include <math.h>

#define DEFAULT_TARGET_DIST_MM 300.0f
#define DEFAULT_MAX_SPEED_MM_S 800.0f

#define ACCEL_MM_S2 800.0f
#define DECEL_MM_S2 1000.0f

#define APPROACH_SPEED_MM_S 550.0f
#define FINAL_BRAKE_DIST_MM 25.0f

#define BRAKE_MARGIN_MM 30.0f
#define MIN_MOVE_SPEED_MM_S 80.0f
#define STOP_DIST_TOL_MM 2.0f

static float targetDist_mm = DEFAULT_TARGET_DIST_MM;
static float requestedMaxSpeed_mm_s = DEFAULT_MAX_SPEED_MM_S;
static float maxSpeed_mm_s = DEFAULT_MAX_SPEED_MM_S;

static float profileSpeed = 0.0f;
static uint8_t isDecelerating = 0;

void MotionProfile_Reset(void)
{
    profileSpeed = 0.0f;
    isDecelerating = 0;
}

void MotionProfile_SetMove(float distance_mm, float maxSpeed_mm_s_input)
{
    float reachableSpeed;
    float denom;

    if (distance_mm < 0.0f)
    {
        distance_mm = 0.0f;
    }

    if (maxSpeed_mm_s_input < 0.0f)
    {
        maxSpeed_mm_s_input = -maxSpeed_mm_s_input;
    }

    targetDist_mm = distance_mm;
    requestedMaxSpeed_mm_s = maxSpeed_mm_s_input;

    if (requestedMaxSpeed_mm_s < APPROACH_SPEED_MM_S)
    {
        requestedMaxSpeed_mm_s = APPROACH_SPEED_MM_S;
    }

    denom = (1.0f / (2.0f * ACCEL_MM_S2)) +
            (1.0f / (2.0f * DECEL_MM_S2));

    if (targetDist_mm > 0.0f && denom > 0.0f)
    {
        reachableSpeed = sqrtf(targetDist_mm / denom);
    }
    else
    {
        reachableSpeed = APPROACH_SPEED_MM_S;
    }

    maxSpeed_mm_s = requestedMaxSpeed_mm_s;

    if (reachableSpeed < maxSpeed_mm_s)
    {
        maxSpeed_mm_s = reachableSpeed;
    }

    if (maxSpeed_mm_s < APPROACH_SPEED_MM_S)
    {
        maxSpeed_mm_s = APPROACH_SPEED_MM_S;
    }

    MotionProfile_Reset();
}

float MotionProfile_Update(float avgDist_mm, float actualSpeed_mm_s, float dt_sec)
{
    float remaining;
    float actualSpeedAbs;
    float decelDistToApproach;

    remaining = targetDist_mm - avgDist_mm;

    if (remaining < 0.0f)
    {
        remaining = 0.0f;
    }

    actualSpeedAbs = fabsf(actualSpeed_mm_s);

    if (remaining <= FINAL_BRAKE_DIST_MM || remaining <= STOP_DIST_TOL_MM)
    {
        profileSpeed = 0.0f;
        isDecelerating = 0;
        return profileSpeed;
    }

    if (actualSpeedAbs > APPROACH_SPEED_MM_S)
    {
        decelDistToApproach =
            ((actualSpeedAbs * actualSpeedAbs) -
             (APPROACH_SPEED_MM_S * APPROACH_SPEED_MM_S))
            / (2.0f * DECEL_MM_S2);
    }
    else
    {
        decelDistToApproach = 0.0f;
    }

    if ((decelDistToApproach + BRAKE_MARGIN_MM + FINAL_BRAKE_DIST_MM) >= remaining)
    {
        isDecelerating = 1;

        profileSpeed -= DECEL_MM_S2 * dt_sec;

        if (profileSpeed < APPROACH_SPEED_MM_S)
        {
            profileSpeed = APPROACH_SPEED_MM_S;
        }
    }
    else
    {
        isDecelerating = 0;

        profileSpeed += ACCEL_MM_S2 * dt_sec;

        if (profileSpeed > maxSpeed_mm_s)
        {
            profileSpeed = maxSpeed_mm_s;
        }
    }

    if (profileSpeed < MIN_MOVE_SPEED_MM_S)
    {
        profileSpeed = MIN_MOVE_SPEED_MM_S;
    }

    return profileSpeed;
}

float MotionProfile_GetTargetDistance(void)
{
    return targetDist_mm;
}

float MotionProfile_GetMaxSpeed(void)
{
    return maxSpeed_mm_s;
}

float MotionProfile_GetProfileSpeed(void)
{
    return profileSpeed;
}

uint8_t MotionProfile_IsDecelerating(void)
{
    return isDecelerating;
}
