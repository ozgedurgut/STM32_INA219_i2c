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
#include "stm32g4xx_hal.h"

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
#define LOWSENSOR_Pin GPIO_PIN_8
#define LOWSENSOR_GPIO_Port GPIOA
#define HIGHSENSOR_Pin GPIO_PIN_9
#define HIGHSENSOR_GPIO_Port GPIOA
#define HEADSENSOR_EXTI_Pin GPIO_PIN_10
#define HEADSENSOR_EXTI_GPIO_Port GPIOA
#define HEADSENSOR_EXTI_EXTI_IRQn EXTI15_10_IRQn
#define COVERSENSOR_EXTI_Pin GPIO_PIN_11
#define COVERSENSOR_EXTI_GPIO_Port GPIOA
#define COVERSENSOR_EXTI_EXTI_IRQn EXTI15_10_IRQn
#define PHOTOSENSOR_EXTI_Pin GPIO_PIN_12
#define PHOTOSENSOR_EXTI_GPIO_Port GPIOA
#define PHOTOSENSOR_EXTI_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
