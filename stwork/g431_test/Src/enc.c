/*
 * enc.c
 *
 *  Created on: May 21, 2020
 *      Author: derek-lam
 */

#include "enc.h"
#include "main.h"
#include "adc.h"

uint8_t enc() {
	uint8_t a = HAL_GPIO_ReadPin(A_GPIO_Port, A_Pin);
	uint8_t b = HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin);
	uint8_t z = HAL_GPIO_ReadPin(Z_GPIO_Port, Z_Pin);

	return (a << 2) | (b << 1) | z;
}
