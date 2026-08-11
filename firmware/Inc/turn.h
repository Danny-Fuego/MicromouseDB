#ifndef TURN_H
#define TURN_H

#include "main.h"
#include "motor.h"
#include "bno055.h"
#include "speed_controller.h"
#include "encoder.h"

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

typedef enum
{
    TURN_LEFT  = -1,
    TURN_RIGHT =  1
} TurnDirection;

typedef enum
{
    TURN_OK = 0,
    TURN_TIMEOUT = 1,
    TURN_IMU_ERR = 2
} TurnResult;

#define TURN_BOOST_SPEED_MM_S   720.0f
#define TURN_SPEED_MM_S         699.0f
#define TURN_BOOST_TIME_MS      8U
#define TURN_CONTROL_DT_MS      5U
#define TURN_CONTROL_DT_SEC     0.005f
#define TURN_FINAL_WAIT_MS      500U

TurnResult Turn_Execute(I2C_HandleTypeDef *hi2c,
                        float angle_deg,
                        TurnDirection direction);

#endif
