/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define AD_BUSY_Pin GPIO_PIN_13
#define AD_BUSY_GPIO_Port GPIOE
#define AD_BUSY_EXTI_IRQn EXTI15_10_IRQn
#define AD_CS_Pin GPIO_PIN_14
#define AD_CS_GPIO_Port GPIOE
#define AD_RD_Pin GPIO_PIN_15
#define AD_RD_GPIO_Port GPIOE
#define AD_RESET_Pin GPIO_PIN_10
#define AD_RESET_GPIO_Port GPIOB
#define AD_CONV_Pin GPIO_PIN_11                    
#define AD_CONV_GPIO_Port GPIOB
#define AD_RANGE_Pin GPIO_PIN_12
#define AD_RANGE_GPIO_Port GPIOB
#define AD_OS3_Pin GPIO_PIN_13
#define AD_OS3_GPIO_Port GPIOB
#define AD_OS2_Pin GPIO_PIN_14
#define AD_OS2_GPIO_Port GPIOB
#define AD_OS1_Pin GPIO_PIN_15
#define AD_OS1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
