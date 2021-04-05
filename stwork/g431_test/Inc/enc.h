/*
 * enc.h
 *
 *  Created on: May 21, 2020
 *      Author: derek-lam
 */

#include <stm32g4xx_hal.h>

#ifndef ENC_H_
#define ENC_H_

uint8_t enc();
#define Z_ST 0
#define Z_A_POL 1
#define ENCPOL 1
#define HALL_INIT -26
#define ADC_THRESH_RISE 3258 // (2.5*1.05)/3.3*4096
#define ADC_THRESH_FALL 2955 // (2.5/1.05)/3.3*4096

#endif /* ENC_H_ */
