#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

// ── Wall detection thresholds (inches) ───────────────────────────────────────
#define IR_WALL_ON_IN          4.5f
#define IR_WALL_OFF_IN         5.5f
#define IR_APPROACH_ON_IN      3.0f
#define IR_APPROACH_OFF_IN     3.5f

typedef struct {
    uint16_t l;
    uint16_t f;
    uint16_t r;
} IR_Readings;

// Init / reset
void     IR_Init(void);
void     IR_BackgroundReset(void);

// Background state machine — call from SysTick
void     IR_BackgroundServiceStep(void);

// Blocking full scan (use only when motors stopped)
void     IR_ScanAll(IR_Readings *out);

// Raw ADC getters
uint16_t IR_GetL(void);
uint16_t IR_GetF(void);
uint16_t IR_GetR(void);

// Distance conversion
float    IR_ToDistance(uint16_t raw);
float    IR_ToDistanceL(uint16_t raw);
float    IR_ToDistanceF(uint16_t raw);
float    IR_ToDistanceR(uint16_t raw);

// Blocking wall states
uint8_t  IR_WallLeft(void);
uint8_t  IR_WallFront(void);
uint8_t  IR_WallRight(void);
uint8_t  IR_WallApproachingLeft(void);
uint8_t  IR_WallApproachingFront(void);
uint8_t  IR_WallApproachingRight(void);

// Background (non-blocking) getters
uint16_t IR_GetL_BG(void);
uint16_t IR_GetF_BG(void);
uint16_t IR_GetR_BG(void);

float    IR_GetDistanceL_BG(void);
float    IR_GetDistanceF_BG(void);
float    IR_GetDistanceR_BG(void);

uint8_t  IR_WallLeft_BG(void);
uint8_t  IR_WallFront_BG(void);
uint8_t  IR_WallRight_BG(void);
uint8_t  IR_WallApproachingLeft_BG(void);
uint8_t  IR_WallApproachingFront_BG(void);
uint8_t  IR_WallApproachingRight_BG(void);

#endif // SENSORS_H
