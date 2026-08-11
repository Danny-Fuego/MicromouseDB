/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IR_EMITTER_R_Pin GPIO_PIN_13
#define IR_EMITTER_R_GPIO_Port GPIOC
#define IR_RECEIVER_R_Pin GPIO_PIN_1
#define IR_RECEIVER_R_GPIO_Port GPIOA
#define IR_RECEIVER_F_Pin GPIO_PIN_4
#define IR_RECEIVER_F_GPIO_Port GPIOA
#define IR_EMITTER_F_Pin GPIO_PIN_5
#define IR_EMITTER_F_GPIO_Port GPIOA
#define IR_EMITTER_L_Pin GPIO_PIN_6
#define IR_EMITTER_L_GPIO_Port GPIOA
#define IR_RECEIVER_L_Pin GPIO_PIN_7
#define IR_RECEIVER_L_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_12
#define BUTTON_GPIO_Port GPIOB
#define ENCODER_LEFT_B_Pin GPIO_PIN_8
#define ENCODER_LEFT_B_GPIO_Port GPIOA
#define ENCODER_LEFT_A_Pin GPIO_PIN_9
#define ENCODER_LEFT_A_GPIO_Port GPIOA
#define MOTOR_SLEEP_Pin GPIO_PIN_10
#define MOTOR_SLEEP_GPIO_Port GPIOA
#define GYRO_INT_Pin GPIO_PIN_11
#define GYRO_INT_GPIO_Port GPIOA
#define GYRO_INT_EXTI_IRQn EXTI15_10_IRQn
#define GYRO_RST_Pin GPIO_PIN_12
#define GYRO_RST_GPIO_Port GPIOA
#define MOTOR_LEFT_IN1_Pin GPIO_PIN_15
#define MOTOR_LEFT_IN1_GPIO_Port GPIOA
#define MOTOR_LEFT_IN2_Pin GPIO_PIN_3
#define MOTOR_LEFT_IN2_GPIO_Port GPIOB
#define MOTOR_RIGHT_IN1_Pin GPIO_PIN_4
#define MOTOR_RIGHT_IN1_GPIO_Port GPIOB
#define MOTOR_RIGHT_IN2_Pin GPIO_PIN_5
#define MOTOR_RIGHT_IN2_GPIO_Port GPIOB
#define ENCODER_RIGHT_A_Pin GPIO_PIN_6
#define ENCODER_RIGHT_A_GPIO_Port GPIOB
#define ENCODER_RIGHT_B_Pin GPIO_PIN_7
#define ENCODER_RIGHT_B_GPIO_Port GPIOB
#define GYRO_SCL_Pin GPIO_PIN_8
#define GYRO_SCL_GPIO_Port GPIOB
#define GYRO_SDA_Pin GPIO_PIN_9
#define GYRO_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
