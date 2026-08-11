#ifndef BUTTON_H
#define BUTTON_H

#include "main.h"
#include <stdint.h>

void Button_Init(void);

uint8_t Button_IsPressed(void);
uint8_t Button_WasPressed(void);

#endif
