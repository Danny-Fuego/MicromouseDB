#include "main.h"
#include "sensors.h"

extern ADC_HandleTypeDef hadc1;

// ── Pin mapping (from IOC) ────────────────────────────────────────────────────
// IR_EMITTER_L  → PA5  (GPIO output)
// IR_EMITTER_F  → PA4  (GPIO output)
// IR_EMITTER_R  → PC13 (GPIO output)
// IR_RECEIVER_L → PA7  (ADC1 channel 7)
// IR_RECEIVER_F → PA4  (ADC1 channel 4)
// IR_RECEIVER_R → PA1  (ADC1 channel 1)
// ─────────────────────────────────────────────────────────────────────────────

#define IR_LEFT_RECEIVER_ADC_CH   ADC_CHANNEL_7   // PA7
#define IR_FRONT_RECEIVER_ADC_CH  ADC_CHANNEL_4   // PA4
#define IR_RIGHT_RECEIVER_ADC_CH  ADC_CHANNEL_1   // PA1

// ── Per-sensor calibration data ───────────────────────────────────────────────
// Calibrated 2026-06-25. Raw ADC (ambient-subtracted) at each distance.
// Each entry: { raw_adc, distance_inches }
// ─────────────────────────────────────────────────────────────────────────────

typedef struct { uint16_t raw; float dist_in; } IR_CalPoint;

static const IR_CalPoint cal_left[] = {
    { 327, 1.0f },
    { 122, 2.0f },
    {  66, 3.0f },
    {  44, 4.0f },
    {  33, 5.0f },
    {  27, 6.0f },
    {  24, 7.0f },
    {  21, 8.0f },
    {  20, 9.0f },
    {  18, 10.0f },
};

static const IR_CalPoint cal_front[] = {
    { 225, 1.0f },
    { 107, 2.0f },
    {  63, 3.0f },
    {  43, 4.0f },
    {  34, 5.0f },
    {  30, 6.0f },
    {  26, 7.0f },
    {  23, 8.0f },
    {  22, 9.0f },
    {  20, 10.0f },
};

static const IR_CalPoint cal_right[] = {
    { 180, 1.0f },
    {  86, 2.0f },
    {  56, 3.0f },
    {  38, 4.0f },
    {  31, 5.0f },
    {  27, 6.0f },
    {  24, 7.0f },
    {  22, 8.0f },
    {  21, 9.0f },
    {  20, 10.0f },
};

#define CAL_SIZE  10
#define IR_BG_FILTER_ALPHA  0.30f

// Interpolate distance from raw ADC using a calibration table.
// Table is ordered high-raw (close) to low-raw (far).
static float IR_Interpolate(uint16_t raw, const IR_CalPoint *cal, int n)
{
    // Beyond max range (raw too low) — no wall
    if (raw <= cal[n-1].raw) return 999.0f;

    // Closer than minimum calibrated distance
    if (raw >= cal[0].raw) return cal[0].dist_in;

    // Find surrounding points and interpolate
    for (int i = 0; i < n - 1; i++)
    {
        if (raw <= cal[i].raw && raw >= cal[i+1].raw)
        {
            float t = (float)(cal[i].raw - raw) /
                      (float)(cal[i].raw - cal[i+1].raw);
            return cal[i].dist_in + t * (cal[i+1].dist_in - cal[i].dist_in);
        }
    }

    return 999.0f;
}


// ── Internal state ─────────────────────────────────────────────────────────────

static IR_Readings readings    = {0};
static IR_Readings bg_readings = {0};

static float    bg_dist_l = 999.0f;
static float    bg_dist_f = 999.0f;
static float    bg_dist_r = 999.0f;

static uint16_t bg_active_l = 0;
static uint16_t bg_active_f = 0;
static uint16_t bg_active_r = 0;

// blocking hysteresis
static uint8_t wall_left_state    = 0;
static uint8_t wall_front_state   = 0;
static uint8_t wall_right_state   = 0;
static uint8_t approach_left_state  = 0;
static uint8_t approach_front_state = 0;
static uint8_t approach_right_state = 0;

// background hysteresis
static uint8_t bg_wall_left_state    = 0;
static uint8_t bg_wall_front_state   = 0;
static uint8_t bg_wall_right_state   = 0;
static uint8_t bg_approach_left_state  = 0;
static uint8_t bg_approach_front_state = 0;
static uint8_t bg_approach_right_state = 0;

// ── Helpers ────────────────────────────────────────────────────────────────────

static inline void emitter_on(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static inline void emitter_off(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static void all_emitters_off(void)
{
    emitter_off(IR_EMITTER_L_GPIO_Port,  IR_EMITTER_L_Pin);
    emitter_off(IR_EMITTER_F_GPIO_Port, IR_EMITTER_F_Pin);
    emitter_off(IR_EMITTER_R_GPIO_Port, IR_EMITTER_R_Pin);
}

static uint16_t adc_read(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        Error_Handler();

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 5);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

static uint8_t hysteresis_update(float dist, uint8_t prev,
                                 float on_thresh, float off_thresh)
{
    if (prev)
        return (dist >= off_thresh) ? 0U : 1U;
    else
        return (dist <= on_thresh)  ? 1U : 0U;
}

static float bg_filter(float old_val, float new_val)
{
    return ((1.0f - IR_BG_FILTER_ALPHA) * old_val) +
           (IR_BG_FILTER_ALPHA * new_val);
}

// ── Public API ─────────────────────────────────────────────────────────────────

float IR_ToDistance(uint16_t raw)
{
    // Generic — uses left table as default (for blocking scan compatibility)
    return IR_Interpolate(raw, cal_left, CAL_SIZE);
}

float IR_ToDistanceL(uint16_t raw)
{
    return IR_Interpolate(raw, cal_left, CAL_SIZE);
}

float IR_ToDistanceF(uint16_t raw)
{
    return IR_Interpolate(raw, cal_front, CAL_SIZE);
}

float IR_ToDistanceR(uint16_t raw)
{
    return IR_Interpolate(raw, cal_right, CAL_SIZE);
}

void IR_Init(void)
{
    all_emitters_off();

    wall_left_state = wall_front_state = wall_right_state = 0;
    approach_left_state = approach_front_state = approach_right_state = 0;

    bg_wall_left_state = bg_wall_front_state = bg_wall_right_state = 0;
    bg_approach_left_state = bg_approach_front_state = bg_approach_right_state = 0;

    readings.l = readings.f = readings.r = 0;
    bg_readings.l = bg_readings.f = bg_readings.r = 0;

    bg_dist_l = bg_dist_f = bg_dist_r = 999.0f;
    bg_active_l = bg_active_f = bg_active_r = 0;

    IR_BackgroundReset();
}

void IR_ScanAll(IR_Readings *out)
{
    uint16_t ambL = adc_read(IR_LEFT_RECEIVER_ADC_CH);
    uint16_t ambF = adc_read(IR_FRONT_RECEIVER_ADC_CH);
    uint16_t ambR = adc_read(IR_RIGHT_RECEIVER_ADC_CH);

    emitter_on(IR_EMITTER_L_GPIO_Port,  IR_EMITTER_L_Pin);
    HAL_Delay(2);
    uint16_t litL = adc_read(IR_LEFT_RECEIVER_ADC_CH);
    emitter_off(IR_EMITTER_L_GPIO_Port, IR_EMITTER_L_Pin);

    emitter_on(IR_EMITTER_F_GPIO_Port,  IR_EMITTER_F_Pin);
    HAL_Delay(2);
    uint16_t litF = adc_read(IR_FRONT_RECEIVER_ADC_CH);
    emitter_off(IR_EMITTER_F_GPIO_Port, IR_EMITTER_F_Pin);

    emitter_on(IR_EMITTER_R_GPIO_Port,  IR_EMITTER_R_Pin);
    HAL_Delay(2);
    uint16_t litR = adc_read(IR_RIGHT_RECEIVER_ADC_CH);
    emitter_off(IR_EMITTER_R_GPIO_Port, IR_EMITTER_R_Pin);

    readings.l = (litL > ambL) ? (litL - ambL) : 0;
    readings.f = (litF > ambF) ? (litF - ambF) : 0;
    readings.r = (litR > ambR) ? (litR - ambR) : 0;

    if (out) *out = readings;

    float dl = IR_ToDistanceL(readings.l);
    float df = IR_ToDistanceF(readings.f);
    float dr = IR_ToDistanceR(readings.r);

    wall_left_state  = hysteresis_update(dl, wall_left_state,  IR_WALL_ON_IN, IR_WALL_OFF_IN);
    wall_front_state = hysteresis_update(df, wall_front_state, IR_WALL_ON_IN, IR_WALL_OFF_IN);
    wall_right_state = hysteresis_update(dr, wall_right_state, IR_WALL_ON_IN, IR_WALL_OFF_IN);

    approach_left_state  = hysteresis_update(dl, approach_left_state,  IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
    approach_front_state = hysteresis_update(df, approach_front_state, IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
    approach_right_state = hysteresis_update(dr, approach_right_state, IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
}

// ── Background state machine ───────────────────────────────────────────────────
// Called every 1ms from SysTick. One step per call.
// Full cycle = 9 steps = ~9ms for all three sensors.

typedef enum {
    IR_BG_LEFT_EMIT_ON = 0,
    IR_BG_LEFT_READ_LIT,
    IR_BG_LEFT_READ_AMBIENT,
    IR_BG_FRONT_EMIT_ON,
    IR_BG_FRONT_READ_LIT,
    IR_BG_FRONT_READ_AMBIENT,
    IR_BG_RIGHT_EMIT_ON,
    IR_BG_RIGHT_READ_LIT,
    IR_BG_RIGHT_READ_AMBIENT
} IR_BG_State;

static volatile IR_BG_State ir_bg_state = IR_BG_LEFT_EMIT_ON;

void IR_BackgroundReset(void)
{
    ir_bg_state = IR_BG_LEFT_EMIT_ON;
    all_emitters_off();
}

void IR_BackgroundServiceStep(void)
{
    switch (ir_bg_state)
    {
    case IR_BG_LEFT_EMIT_ON:
        emitter_on(IR_EMITTER_L_GPIO_Port, IR_EMITTER_L_Pin);
        ir_bg_state = IR_BG_LEFT_READ_LIT;
        break;

    case IR_BG_LEFT_READ_LIT:
        bg_active_l = adc_read(IR_LEFT_RECEIVER_ADC_CH);
        emitter_off(IR_EMITTER_L_GPIO_Port, IR_EMITTER_L_Pin);
        ir_bg_state = IR_BG_LEFT_READ_AMBIENT;
        break;

    case IR_BG_LEFT_READ_AMBIENT:
    {
        uint16_t amb = adc_read(IR_LEFT_RECEIVER_ADC_CH);
        bg_readings.l = (bg_active_l > amb) ? (bg_active_l - amb) : 0;
        bg_dist_l = bg_filter(bg_dist_l, IR_ToDistanceL(bg_readings.l));
        bg_wall_left_state    = hysteresis_update(bg_dist_l, bg_wall_left_state,    IR_WALL_ON_IN,     IR_WALL_OFF_IN);
        bg_approach_left_state = hysteresis_update(bg_dist_l, bg_approach_left_state, IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
        ir_bg_state = IR_BG_FRONT_EMIT_ON;
        break;
    }

    case IR_BG_FRONT_EMIT_ON:
        emitter_on(IR_EMITTER_F_GPIO_Port, IR_EMITTER_F_Pin);
        ir_bg_state = IR_BG_FRONT_READ_LIT;
        break;

    case IR_BG_FRONT_READ_LIT:
        bg_active_f = adc_read(IR_FRONT_RECEIVER_ADC_CH);
        emitter_off(IR_EMITTER_F_GPIO_Port, IR_EMITTER_F_Pin);
        ir_bg_state = IR_BG_FRONT_READ_AMBIENT;
        break;

    case IR_BG_FRONT_READ_AMBIENT:
    {
        uint16_t amb = adc_read(IR_FRONT_RECEIVER_ADC_CH);
        bg_readings.f = (bg_active_f > amb) ? (bg_active_f - amb) : 0;
        bg_dist_f = bg_filter(bg_dist_f, IR_ToDistanceF(bg_readings.f));
        bg_wall_front_state    = hysteresis_update(bg_dist_f, bg_wall_front_state,    IR_WALL_ON_IN,     IR_WALL_OFF_IN);
        bg_approach_front_state = hysteresis_update(bg_dist_f, bg_approach_front_state, IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
        ir_bg_state = IR_BG_RIGHT_EMIT_ON;
        break;
    }

    case IR_BG_RIGHT_EMIT_ON:
        emitter_on(IR_EMITTER_R_GPIO_Port, IR_EMITTER_R_Pin);
        ir_bg_state = IR_BG_RIGHT_READ_LIT;
        break;

    case IR_BG_RIGHT_READ_LIT:
        bg_active_r = adc_read(IR_RIGHT_RECEIVER_ADC_CH);
        emitter_off(IR_EMITTER_R_GPIO_Port, IR_EMITTER_R_Pin);
        ir_bg_state = IR_BG_RIGHT_READ_AMBIENT;
        break;

    case IR_BG_RIGHT_READ_AMBIENT:
    {
        uint16_t amb = adc_read(IR_RIGHT_RECEIVER_ADC_CH);
        bg_readings.r = (bg_active_r > amb) ? (bg_active_r - amb) : 0;
        bg_dist_r = bg_filter(bg_dist_r, IR_ToDistanceR(bg_readings.r));
        bg_wall_right_state    = hysteresis_update(bg_dist_r, bg_wall_right_state,    IR_WALL_ON_IN,     IR_WALL_OFF_IN);
        bg_approach_right_state = hysteresis_update(bg_dist_r, bg_approach_right_state, IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
        ir_bg_state = IR_BG_LEFT_EMIT_ON;
        break;
    }

    default:
        ir_bg_state = IR_BG_LEFT_EMIT_ON;
        all_emitters_off();
        break;
    }
}

// ── Getters ────────────────────────────────────────────────────────────────────

uint16_t IR_GetL(void) { return readings.l; }
uint16_t IR_GetF(void) { return readings.f; }
uint16_t IR_GetR(void) { return readings.r; }

uint8_t IR_WallLeft(void)           { return wall_left_state; }
uint8_t IR_WallFront(void)          { return wall_front_state; }
uint8_t IR_WallRight(void)          { return wall_right_state; }
uint8_t IR_WallApproachingLeft(void)  { return approach_left_state; }
uint8_t IR_WallApproachingFront(void) { return approach_front_state; }
uint8_t IR_WallApproachingRight(void) { return approach_right_state; }

uint16_t IR_GetL_BG(void) { return bg_readings.l; }
uint16_t IR_GetF_BG(void) { return bg_readings.f; }
uint16_t IR_GetR_BG(void) { return bg_readings.r; }

float    IR_GetDistanceL_BG(void) { return bg_dist_l; }
float    IR_GetDistanceF_BG(void) { return bg_dist_f; }
float    IR_GetDistanceR_BG(void) { return bg_dist_r; }

uint8_t IR_WallLeft_BG(void)           { return bg_wall_left_state; }
uint8_t IR_WallFront_BG(void)          { return bg_wall_front_state; }
uint8_t IR_WallRight_BG(void)          { return bg_wall_right_state; }
uint8_t IR_WallApproachingLeft_BG(void)  { return bg_approach_left_state; }
uint8_t IR_WallApproachingFront_BG(void) { return bg_approach_front_state; }
uint8_t IR_WallApproachingRight_BG(void) { return bg_approach_right_state; }
