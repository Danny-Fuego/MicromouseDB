#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>

typedef enum{
    MOTOR_COAST = 0,
    MOTOR_BRAKE
} MotorStopMode;

void Motor_Init(void);
void Motor_Enable(void);
void Motor_Disable(void);

void Motor_SetRawPWM(int16_t leftPWM, int16_t rightPWM);
void Motor_Stop(MotorStopMode mode);

#endif
