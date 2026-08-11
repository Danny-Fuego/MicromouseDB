#ifndef ENCODER_H
#define ENCODER_H
#include "main.h"
#include <stdint.h>

void Encoder_Init(void);
void Encoder_ResetAll(void);
void Encoder_ResetLeft(void);
void Encoder_ResetRight(void);

int16_t Encoder_GetLeftRaw(void);
int16_t Encoder_GetRightRaw(void);

int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);

void Encoder_Update(void);

#endif
