/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "enc.h"
#include "motor.h"
#include "math_util.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern volatile uint16_t adc1_reg[8];
extern DMA_HandleTypeDef hdma_adc1;
unsigned char uart_buf[128] = { 0 };
extern volatile uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
extern volatile float torque_volts, bemf_volts, speed_avg, theta_offset;
extern volatile int16_t ctrl;
extern volatile uint16_t pwmin;
// extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern volatile uint8_t uart_rx_valid;
volatile uint32_t last_uart = 0;

extern volatile uint32_t enc_revs;

extern imu_t imu, imu_avg;
extern volatile uint8_t z_triggered;
int16_t gyro_log[512][2] = { 0 };
uint16_t gyro_log_idx = 0;
volatile uint8_t ctrl_on = 0;
//int16_t enc_log[256][2] = { 0 };
//uint8_t enc_log_idx = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

//  htim1.Instance->DIER |= TIM_DIER_COMIE;
  hadc1.Instance->CFGR |= ADC_CFGR_DMACFG;
  // NOTE: the DMA must always be able to service the ADC: if the ADC OVR's, it'll stop DMA-ing even if the overrun behavior is set to overwrite (which doesn't make sense to me...)
  HAL_ADC_Start_DMA(&hadc1, adc1_reg, 2);
  hdma_adc1.Instance->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TEIE | DMA_CCR_TCIE);

//  htim1.Instance->CR2 |= TIM_CR2_CCUS | TIM_CR2_CCPC;
  HAL_TIM_Base_Start(&htim1);

//  extern uint16_t state_en[6];
//  htim1.Instance->CCR1 = htim1.Instance->CCR2 = htim1.Instance->CCR3 = 600;
//  TIM1->CCER = state_en[5];
//  HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM);
//
//  HAL_Delay(600);


  if(0) {
	  htim4.Instance->DIER |= TIM_DIER_CC2IE;
	  htim4.Instance->CCER |= TIM_CCER_CC1E; // | TIM_CCER_CC2E;
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1);
	  htim4.Instance->SMCR &= ~TIM_SMCR_SMS;
	  htim4.Instance->SMCR |= TIM_SLAVEMODE_COMBINED_RESETTRIGGER; // make it reset and trigger so that those other instances hit too
  }
//  htim4.Instance->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
//  htim4.Instance->CCMR1 |= 0b0001 << TIM_CCMR1_OC2M_Pos;
  HAL_TIM_Base_Start(&htim4);
//  HAL_TIM_Base_Start_IT(&htim8);
//  htim1.Instance->DIER |= TIM_DIER_COMIE;

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start(&htim7);
  htim1.Instance->BDTR |= TIM_BDTR_MOE;

  HAL_ADC_Start(&hadc1);

  HAL_DMA_RegisterCallback(&hdma_usart2_rx, HAL_DMA_XFER_ALL_CB_ID, &DMA2_XferCpltCallback);
  HAL_UART_Receive_DMA(&huart2, uart_rx_buf, UART_RX_BUF_SIZE);

  for(uint16_t theta = 0x1FF, i = 0; !z_triggered; theta = (theta + 8) & 0x1FF, i++) { //
	  motor_to(theta, 50);
	  HAL_Delay(1);

//	  if((i & 0x7) == 0) {
//		  uint8_t idx = (enc_log_idx++) & 0xFF;
//		  enc_log[idx][0] = theta;
//		  enc_log[idx][1] = __HAL_TIM_GET_COUNTER(&htim4);
//	  }
  }

  if(0) {
	  for(uint32_t iter = 0;; iter++) {
		  int16_t enc_raw = -(((int16_t)__HAL_TIM_GET_COUNTER(&htim4)) - MECH_OFFSET);
		  uint16_t enc_theta = (modpos(enc_raw, ENC_MOD_BASE) * THETA_360DEG) / ENC_MOD_BASE;
		  motor_to((enc_theta + THETA_270DEG) & 0x1FF, 50);
	  }
  }

//  HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_TRIGGER);
//  motor_tick(1);
//  HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM);

//  __HAL_TIM_SET_COUNTER(&htim4, HALL_INIT);
//    motor_tick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t edge = 0;
//
//  extern volatile uint16_t ccr_;
//  TIM1->CCR1 = 0;
//  TIM1->CCR2 = 240;
//  TIM1->CCER = 0b000000010100;
//  for(uint8_t i = 20;; i += 4) {
//	  uint16_t ccr__ = ((uint16_t)i) * 160 / 256;
//	  TIM1->CCR1 = ccr__;
////	  HAL_Delay(200);
//  }

  ctrl_on = 1;
  for (volatile uint32_t ticks = 0;; ticks++)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t tick = HAL_GetTick();
	  if(last_uart + UART_RX_INACTIVITY_TIMEOUT < tick) {
		  uart_rx_valid = 0;
	  }

	  if(0) {
		  if(tick > 7000) {
			  ctrl = 0;
		  }
		  else {
			  if(tick > 3000) {
				  ctrl = CTRL0 / 2; // -abs(ctrl_setpoint);
			  }
//			  ctrl = ctrl_setpoint; // (ctrl * 7 + ctrl_setpoint) / 8;

			  if(1) { // HAL_GetTick() > 10000 &&
				  // "%lu,%d,%.1f\r\n", tick, ctrl, speed_avg); //
				  volatile uint16_t uart_buflen = sprintf(uart_buf, "%lu,%d,%.1f\r\n", tick, ctrl, speed_avg);
				  HAL_UART_Transmit_IT(&huart2, uart_buf, uart_buflen);
				  HAL_Delay(20);
			  }
		  }
	  }

	  if(1) { // HAL_GetTick() > 10000 &&
		  // "%lu,%d,%.1f\r\n", tick, ctrl, speed_avg); //
		  volatile uint16_t uart_buflen = sprintf(uart_buf, "%lu\t%d\t%.2f\t%.2f\t%.1f\t%d\t%d\r\n", HAL_GetTick(), ctrl, bemf_volts, torque_volts, speed_avg, imu_avg.tilt, imu.gyro); // ctrl, torque_ccr, speed_avg, enc_revs // "%d\t%.1f\r\n", HAL_GetTick(), speed_avg);
		  HAL_UART_Transmit_IT(&huart2, uart_buf, uart_buflen);
		  HAL_Delay(20);
	  }

	  if(0) {
		  if(imu_avg.tilt > 0 && gyro_log_idx == 0) {
			  gyro_log_idx = 1;
		  }
		  if(gyro_log_idx > 0 && gyro_log_idx < 512) {
			  gyro_log[gyro_log_idx & 0x1FF][0] = imu.gyro;
			  gyro_log[gyro_log_idx & 0x1FF][1] = ctrl;
			  gyro_log_idx++;
			  HAL_Delay(1);
		  }
		  else if(gyro_log_idx == 512) {
			  ctrl_on = 0;
			  ctrl = 0;
		  }
	  }


//	  if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET && !edge) {
//		  offset++;
//		  edge = 1;
//	  }
//	  if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET)
//		  edge = 0;

//	  HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, abz & 1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void DMA2_XferCpltCallback(DMA_HandleTypeDef *hdma) {
	uart_rx_valid = 1;
	last_uart = HAL_GetTick();
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	uart_rx_valid = 1;
	last_uart = HAL_GetTick();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
