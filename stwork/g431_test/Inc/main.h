/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
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
#define VBUS_Pin GPIO_PIN_0
#define VBUS_GPIO_Port GPIOA
#define BEMF1_Pin GPIO_PIN_4
#define BEMF1_GPIO_Port GPIOA
#define BEMF2_Pin GPIO_PIN_4
#define BEMF2_GPIO_Port GPIOC
#define BEMF3_Pin GPIO_PIN_11
#define BEMF3_GPIO_Port GPIOB
#define POT_Pin GPIO_PIN_12
#define POT_GPIO_Port GPIOB
#define Temperature_Pin GPIO_PIN_14
#define Temperature_GPIO_Port GPIOB
#define STATUS_Pin GPIO_PIN_6
#define STATUS_GPIO_Port GPIOC
#define PWMIN_Pin GPIO_PIN_15
#define PWMIN_GPIO_Port GPIOA
#define PWMIN_EXTI_IRQn EXTI15_10_IRQn
#define GPIO_BEMF_Pin GPIO_PIN_5
#define GPIO_BEMF_GPIO_Port GPIOB
#define A_Pin GPIO_PIN_6
#define A_GPIO_Port GPIOB
#define B_Pin GPIO_PIN_7
#define B_GPIO_Port GPIOB
#define Z_Pin GPIO_PIN_8
#define Z_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define UART_RX_INACTIVITY_TIMEOUT 1000
void DMA2_XferCpltCallback(DMA_HandleTypeDef *hdma);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
