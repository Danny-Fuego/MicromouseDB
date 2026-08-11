#include "motor.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

// Left motor
#define L_TIM    (&htim2)
#define L_CH_IN1 TIM_CHANNEL_1
#define L_CH_IN2 TIM_CHANNEL_2

// Right motor
#define R_TIM    (&htim3)
#define R_CH_IN1 TIM_CHANNEL_2
#define R_CH_IN2 TIM_CHANNEL_1

static int16_t Motor_ClampCommand(int16_t pwm)
{
    if (pwm > 1000) return 1000;
    if (pwm < -1000) return -1000;
    return pwm;
}

static void Motor_ApplyPWM(TIM_HandleTypeDef *htim, uint32_t ch, int16_t duty_0_to_1000)
{
    uint32_t arr;
    uint32_t ccr;

    if (duty_0_to_1000 < 0) duty_0_to_1000 = 0;
    if (duty_0_to_1000 > 1000) duty_0_to_1000 = 1000;

    arr = __HAL_TIM_GET_AUTORELOAD(htim);
    ccr = (arr * (uint32_t)duty_0_to_1000) / 1000U;

    __HAL_TIM_SET_COMPARE(htim, ch, ccr);
}

static void Motor_Apply(TIM_HandleTypeDef *htim, uint32_t ch1, uint32_t ch2, int16_t pwm)
{
    pwm = Motor_ClampCommand(pwm);

    if (pwm > 0)
    {
        Motor_ApplyPWM(htim, ch1, pwm);
        Motor_ApplyPWM(htim, ch2, 0);
    }
    else if (pwm < 0)
    {
        Motor_ApplyPWM(htim, ch1, 0);
        Motor_ApplyPWM(htim, ch2, -pwm);
    }
    else
    {
        Motor_ApplyPWM(htim, ch1, 0);
        Motor_ApplyPWM(htim, ch2, 0);
    }
}

static void Motor_ApplyCoast(void)
{
    Motor_ApplyPWM(L_TIM, L_CH_IN1, 0);
    Motor_ApplyPWM(L_TIM, L_CH_IN2, 0);

    Motor_ApplyPWM(R_TIM, R_CH_IN1, 0);
    Motor_ApplyPWM(R_TIM, R_CH_IN2, 0);
}

static void Motor_ApplyBrake(void)
{
    Motor_ApplyPWM(L_TIM, L_CH_IN1, 1000);
    Motor_ApplyPWM(L_TIM, L_CH_IN2, 1000);

    Motor_ApplyPWM(R_TIM, R_CH_IN1, 1000);
    Motor_ApplyPWM(R_TIM, R_CH_IN2, 1000);
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(L_TIM, L_CH_IN1);
    HAL_TIM_PWM_Start(L_TIM, L_CH_IN2);

    HAL_TIM_PWM_Start(R_TIM, R_CH_IN1);
    HAL_TIM_PWM_Start(R_TIM, R_CH_IN2);

    Motor_Stop(MOTOR_COAST);
    Motor_Disable();
}

void Motor_Enable(void)
{
    HAL_GPIO_WritePin(MOTOR_SLEEP_GPIO_Port, MOTOR_SLEEP_Pin, GPIO_PIN_SET);
}

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(MOTOR_SLEEP_GPIO_Port, MOTOR_SLEEP_Pin, GPIO_PIN_RESET);
}

void Motor_SetRawPWM(int16_t leftPWM, int16_t rightPWM)
{
    Motor_Apply(L_TIM, L_CH_IN1, L_CH_IN2, leftPWM);
    Motor_Apply(R_TIM, R_CH_IN1, R_CH_IN2, rightPWM);
}

void Motor_Stop(MotorStopMode mode)
{
    if (mode == MOTOR_BRAKE)
    {
        Motor_ApplyBrake();
    }
    else
    {
        Motor_ApplyCoast();
    }
}
