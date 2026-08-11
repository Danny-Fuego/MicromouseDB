#include "turn.h"

extern UART_HandleTypeDef huart2;

static void Turn_Log(const char *fmt, ...)
{
    char buf[180];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 500);
}

static float Turn_ClampFloat(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

TurnResult Turn_Execute(I2C_HandleTypeDef *hi2c,
                        float angle_deg,
                        TurnDirection direction)
{
    float startHeading = 0.0f;
    float prevHeading = 0.0f;
    float currentHeading = 0.0f;

    float headingStep = 0.0f;
    float accumulatedTurn = 0.0f;
    float turned = 0.0f;
    float remaining = 0.0f;

    float brakeBefore = 0.0f;

    float turnSpeed = 0.0f;
    float leftTargetSpeed = 0.0f;
    float rightTargetSpeed = 0.0f;

    uint32_t turnStartTime = 0;

    BNO055_Status st;

    if (angle_deg <= 0.0f)
    {
        return TURN_OK;
    }

    brakeBefore = angle_deg * 0.04f;
    brakeBefore = Turn_ClampFloat(brakeBefore, 3.0f, 25.0f);

    st = BNO055_ReadHeading(hi2c, &startHeading);
    if (st != BNO055_OK)
    {
        return TURN_IMU_ERR;
    }

    prevHeading = startHeading;

    Turn_Log("===== SPEED TURN START =====\r\n");
    Turn_Log("start=%.2f angle=%.2f dir=%d boost=%.1f cruise=%.1f brakeBefore=%.2f\r\n",
             startHeading,
             angle_deg,
             (int)direction,
             TURN_BOOST_SPEED_MM_S,
             TURN_SPEED_MM_S,
             brakeBefore);

    Controller_Reset();
    Encoder_Update();

    turnStartTime = HAL_GetTick();

    while (1)
    {
        Encoder_Update();

        if ((HAL_GetTick() - turnStartTime) < TURN_BOOST_TIME_MS)
        {
            turnSpeed = TURN_BOOST_SPEED_MM_S;
        }
        else
        {
            turnSpeed = TURN_SPEED_MM_S;
        }

        if (direction == TURN_RIGHT)
        {
            leftTargetSpeed  =  turnSpeed;
            rightTargetSpeed = -turnSpeed;
        }
        else
        {
            leftTargetSpeed  = -turnSpeed;
            rightTargetSpeed =  turnSpeed;
        }

        Controller_UpdateSpeed(leftTargetSpeed,
                               rightTargetSpeed,
                               TURN_CONTROL_DT_SEC,
                               0,
                               turnSpeed,
                               TURN_BOOST_SPEED_MM_S,
                               0.0f);  // totalDist_mm not applicable to turns — isDecelerating is always 0 here, so this is never read

        st = BNO055_ReadHeading(hi2c, &currentHeading);
        if (st != BNO055_OK)
        {
            Motor_Stop(MOTOR_BRAKE);
            Controller_Reset();
            return TURN_IMU_ERR;
        }

        headingStep = BNO055_AngleError(currentHeading, prevHeading);
        accumulatedTurn += headingStep;
        prevHeading = currentHeading;

        turned = ((float)direction) * accumulatedTurn;
        remaining = angle_deg - turned;

        if (remaining <= brakeBefore)
        {
            break;
        }

        HAL_Delay(TURN_CONTROL_DT_MS);
    }

    Motor_Stop(MOTOR_BRAKE);

    Turn_Log("BRAKE heading=%.2f turned=%.2f remaining=%.2f brakeBefore=%.2f\r\n",
             currentHeading,
             turned,
             remaining,
             brakeBefore);

    HAL_Delay(TURN_FINAL_WAIT_MS);

    st = BNO055_ReadHeading(hi2c, &currentHeading);
    if (st != BNO055_OK)
    {
        Controller_Reset();
        return TURN_IMU_ERR;
    }

    headingStep = BNO055_AngleError(currentHeading, prevHeading);
    accumulatedTurn += headingStep;
    prevHeading = currentHeading;

    turned = ((float)direction) * accumulatedTurn;

    Turn_Log("===== SPEED TURN DONE =====\r\n");
    Turn_Log("target=%.2f finalTurned=%.2f overshoot=%.2f finalHeading=%.2f\r\n",
             angle_deg,
             turned,
             turned - angle_deg,
             currentHeading);

    Controller_Reset();

    return TURN_OK;
}
