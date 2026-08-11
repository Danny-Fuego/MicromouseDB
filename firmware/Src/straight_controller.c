#include "straight_controller.h"

#define STRAIGHT_K_DIST              2.0f
#define STRAIGHT_K_DIST_DECEL        0.05f
#define STRAIGHT_K_INT               0.5f

#define STRAIGHT_MAX_CORR_MM_S       500.0f
#define STRAIGHT_MAX_INT             200.0f
#define ENABLE_WALL_FOLLOWING        0


// Left wall following
#define STRAIGHT_WALL_TARGET_IN          3.0f
#define STRAIGHT_WALL_KP                 10.0f
#define STRAIGHT_WALL_KD                 0.0f
#define STRAIGHT_WALL_MAX_CORR           40.0f

#define STRAIGHT_WALL_DANGER_IN          1.0f
#define STRAIGHT_WALL_DANGER_KP          45.0f
#define STRAIGHT_WALL_DANGER_MAX_CORR    120.0f

static float straightError      = 0.0f;
static float straightIntegral   = 0.0f;
static float straightCorrection = 0.0f;

static float wallError          = 0.0f;
static float prevWallError      = 0.0f;
static float wallCorrection     = 0.0f;

static float ClampFloat(float x, float minVal, float maxVal)
{
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
}

void StraightController_Reset(void)
{
    straightError      = 0.0f;
    straightIntegral   = 0.0f;
    straightCorrection = 0.0f;

    wallError          = 0.0f;
    prevWallError      = 0.0f;
    wallCorrection     = 0.0f;
}

void StraightController_Update(float baseSpeed_mm_s,
                               float leftDist_mm,
                               float rightDist_mm,
                               uint8_t isDecelerating,
                               float *leftTarget_mm_s,
                               float *rightTarget_mm_s,
                               float dt_sec)
{
    float kDist;
    float maxCorr;

    straightError = rightDist_mm - leftDist_mm;

    if (baseSpeed_mm_s >= 520.0f)
    {
        straightIntegral += straightError * dt_sec;

        straightIntegral = ClampFloat(straightIntegral,
                                      -STRAIGHT_MAX_INT,
                                       STRAIGHT_MAX_INT);
    }

//    if (isDecelerating)
//    {
//        kDist = STRAIGHT_K_DIST_DECEL;
//    }
//    else
//    {
//        kDist = STRAIGHT_K_DIST;
//    }

    kDist = STRAIGHT_K_DIST;
    straightCorrection =
        (kDist * straightError) +
        (STRAIGHT_K_INT * straightIntegral);

    maxCorr = baseSpeed_mm_s * 0.3f;

    if (maxCorr > STRAIGHT_MAX_CORR_MM_S)
    {
        maxCorr = STRAIGHT_MAX_CORR_MM_S;
    }

    straightCorrection = ClampFloat(straightCorrection,
                                    -maxCorr,
                                     maxCorr);

    /*
       Left wall following:
       Only active while NOT decelerating.
    */
    wallCorrection = 0.0f;

    if (ENABLE_WALL_FOLLOWING && IR_WallLeft_BG() && !isDecelerating)
    {
        float leftWallDist_in;
        float wallDerivative;

        leftWallDist_in = IR_GetDistanceL_BG();

        wallError = STRAIGHT_WALL_TARGET_IN - leftWallDist_in;

        wallDerivative = (wallError - prevWallError) / dt_sec;

        if (leftWallDist_in <= STRAIGHT_WALL_DANGER_IN)
        {
            wallCorrection =
                (STRAIGHT_WALL_DANGER_KP * wallError) +
                (STRAIGHT_WALL_KD * wallDerivative);

            wallCorrection = ClampFloat(wallCorrection,
                                        -STRAIGHT_WALL_DANGER_MAX_CORR,
                                         STRAIGHT_WALL_DANGER_MAX_CORR);
        }
        else
        {
            wallCorrection =
                (STRAIGHT_WALL_KP * wallError) +
                (STRAIGHT_WALL_KD * wallDerivative);

            wallCorrection = ClampFloat(wallCorrection,
                                        -STRAIGHT_WALL_MAX_CORR,
                                         STRAIGHT_WALL_MAX_CORR);
        }

        prevWallError = wallError;
    }
    else
    {
        wallError      = 0.0f;
        prevWallError  = 0.0f;
        wallCorrection = 0.0f;
    }

    straightCorrection += wallCorrection;

    straightCorrection = ClampFloat(straightCorrection,
                                    -maxCorr,
                                     maxCorr);

    *leftTarget_mm_s  = baseSpeed_mm_s + straightCorrection;
    *rightTarget_mm_s = baseSpeed_mm_s - straightCorrection;


    if (*leftTarget_mm_s < 0.0f)
    {
        *leftTarget_mm_s = 0.0f;
    }

    if (*rightTarget_mm_s < 0.0f)
    {
        *rightTarget_mm_s = 0.0f;
    }
}

float StraightController_GetError(void)
{
    return straightError;
}

float StraightController_GetCorrection(void)
{
    return straightCorrection;
}
