#include "motion_guard.h"

#define FRONT_ABORT_DIST_IN 3.0f

static volatile uint8_t motionAbort = 0;
static volatile uint8_t guardEnabled = 0;

void MotionGuard_Update(void)
{
    if (!guardEnabled)
    {
        return;
    }

    if (IR_WallApproachingFront_BG() &&
        IR_GetDistanceF_BG() <= FRONT_ABORT_DIST_IN)
    {
        Motor_Stop(MOTOR_BRAKE);
        motionAbort = 1;
    }
}

void MotionGuard_SetEnabled(uint8_t enable)
{
    guardEnabled = enable ? 1 : 0;

    if (!guardEnabled)
    {
        motionAbort = 0;
    }
}

uint8_t MotionGuard_WasAborted(void)
{
    return motionAbort;
}

void MotionGuard_ClearAbort(void)
{
    motionAbort = 0;
}
