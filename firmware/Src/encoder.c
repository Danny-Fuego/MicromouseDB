#include "encoder.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

static int16_t prevLeftRaw = 0;
static int16_t prevRightRaw = 0;

static int32_t leftCount = 0;
static int32_t rightCount = 0;

void Encoder_Init(void){
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);

	prevLeftRaw = 0;
	prevRightRaw = 0;

	leftCount = 0;
	rightCount = 0;
}

void Encoder_ResetLeft(void){
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	prevLeftRaw = 0;
	leftCount = 0;
}

void Encoder_ResetRight(void){
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	prevRightRaw = 0;
	rightCount = 0;
}

void Encoder_ResetAll(void){
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);

	prevLeftRaw = 0;
	leftCount = 0;
	prevRightRaw = 0;
	rightCount = 0;

}

int16_t Encoder_GetLeftRaw(void){
	return (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

int16_t Encoder_GetRightRaw(void){
	return (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

void Encoder_Update(void){
	int16_t currentLeftRaw = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
	int16_t currentRightRaw = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

	int16_t deltaLeft = currentLeftRaw - prevLeftRaw;
	int16_t deltaRight = currentRightRaw - prevRightRaw;

	leftCount += deltaLeft;
	rightCount += deltaRight;

	prevLeftRaw = currentLeftRaw;
	prevRightRaw = currentRightRaw;
}

int32_t Encoder_GetLeftCount(void){
	return leftCount;
}

int32_t Encoder_GetRightCount(void){
	return rightCount;
}
