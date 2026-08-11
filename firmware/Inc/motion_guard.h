#ifndef MOTION_GUARD_H
#define MOTION_GUARD_H

#include "main.h"
#include <stdint.h>
#include "sensors.h"
#include "motor.h"

void MotionGuard_Update(void);
uint8_t MotionGuard_WasAborted(void);
void MotionGuard_ClearAbort(void);
void MotionGuard_SetEnabled(uint8_t enable);

#endif
