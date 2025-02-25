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
#include "stm32f1xx_hal.h"

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
#define ledPC13_Pin GPIO_PIN_13
#define ledPC13_GPIO_Port GPIOC
#define SSSTART_Pin GPIO_PIN_0
#define SSSTART_GPIO_Port GPIOA
#define SS2_Pin GPIO_PIN_1
#define SS2_GPIO_Port GPIOA
#define SS3_Pin GPIO_PIN_2
#define SS3_GPIO_Port GPIOA
#define SS4_Pin GPIO_PIN_3
#define SS4_GPIO_Port GPIOA
#define SS5_Pin GPIO_PIN_4
#define SS5_GPIO_Port GPIOA
#define SS6_Pin GPIO_PIN_5
#define SS6_GPIO_Port GPIOA
#define SS7_Pin GPIO_PIN_6
#define SS7_GPIO_Port GPIOA
#define SS8_Pin GPIO_PIN_7
#define SS8_GPIO_Port GPIOA
#define ENCO1_Pin GPIO_PIN_0
#define ENCO1_GPIO_Port GPIOB
#define ENCO2_Pin GPIO_PIN_1
#define ENCO2_GPIO_Port GPIOB
#define MODE_Pin GPIO_PIN_2
#define MODE_GPIO_Port GPIOB
#define BUZZ_Pin GPIO_PIN_10
#define BUZZ_GPIO_Port GPIOB
#define SELECT_Pin GPIO_PIN_11
#define SELECT_GPIO_Port GPIOB
#define L7A_Pin GPIO_PIN_12
#define L7A_GPIO_Port GPIOB
#define L6B_Pin GPIO_PIN_13
#define L6B_GPIO_Port GPIOB
#define L5C_Pin GPIO_PIN_14
#define L5C_GPIO_Port GPIOB
#define L4D_Pin GPIO_PIN_15
#define L4D_GPIO_Port GPIOB
#define L3E_Pin GPIO_PIN_8
#define L3E_GPIO_Port GPIOA
#define L2G_Pin GPIO_PIN_9
#define L2G_GPIO_Port GPIOA
#define L1F_Pin GPIO_PIN_10
#define L1F_GPIO_Port GPIOA
#define EN7LED_Pin GPIO_PIN_11
#define EN7LED_GPIO_Port GPIOA
#define EN7SEG_Pin GPIO_PIN_12
#define EN7SEG_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
