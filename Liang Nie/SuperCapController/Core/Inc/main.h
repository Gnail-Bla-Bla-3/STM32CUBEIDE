/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define Iref_Pin GPIO_PIN_6
#define Iref_GPIO_Port GPIOA
#define Vbus_Pin GPIO_PIN_4
#define Vbus_GPIO_Port GPIOC
#define IbridgeA_Pin GPIO_PIN_1
#define IbridgeA_GPIO_Port GPIOB
#define IbridgeB_Pin GPIO_PIN_11
#define IbridgeB_GPIO_Port GPIOF
#define Vcap_Pin GPIO_PIN_13
#define Vcap_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */
typedef enum {
	x = 0,
	X = 0,
	y = 1,
	Y = 1,
	z = 2,
	Z = 2,
} axis_t;

typedef enum {
	r = 0,
	R = 0,
	Red = 0,
	red = 0,
	RED = 0,
	g = 1,
	G = 1,
	Green = 1,
	green = 1,
	GREEN = 1,
	b = 2,
	B = 2,
	Blue = 2,
	blue = 2,
	BLUE = 2,
} color_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
