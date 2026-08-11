#include "button.h"

static uint8_t last_state = GPIO_PIN_SET;

void Button_Init(void){
	last_state = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
}

uint8_t ButtonIsPressed(void){
	return (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET);
}

uint8_t Button_WasPressed(void){
	static uint32_t last_time = 0;
	uint32_t now = HAL_GetTick();

	uint8_t current = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

	if(current == GPIO_PIN_RESET && last_state == GPIO_PIN_SET){
		if(now - last_time > 50){
			last_time = now;
			last_state = current;
			return 1;
		}
	}

	last_state = current;
	return 0;
}
