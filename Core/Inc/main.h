/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GATE_Pin GPIO_PIN_0
#define GATE_GPIO_Port GPIOC
#define CS_0_Pin GPIO_PIN_9
#define CS_0_GPIO_Port GPIOE
#define CS_1_Pin GPIO_PIN_10
#define CS_1_GPIO_Port GPIOE
#define PITCH_BEND_Pin GPIO_PIN_6
#define PITCH_BEND_GPIO_Port GPIOC
#define MODE_SWITCH_Pin GPIO_PIN_7
#define MODE_SWITCH_GPIO_Port GPIOC
#define OCTAVE_DOWN_Pin GPIO_PIN_8
#define OCTAVE_DOWN_GPIO_Port GPIOC
#define OCTAVE_UP_Pin GPIO_PIN_9
#define OCTAVE_UP_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
