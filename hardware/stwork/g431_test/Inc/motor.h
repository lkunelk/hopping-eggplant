/*
 * motor.h
 *
 *  Created on: May 22, 2020
 *      Author: derek-lam
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"

typedef enum {
	FWD_T_BKWD = 0, FWD_T_FWD, FWD_T_STILL, NUM_FWD_TS
} fwd_t;

inline uint8_t _motor_stopped(void) {
	return __HAL_TIM_GET_COUNTER(&htim4) == 0 && __HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_TRIGGER) == RESET;
}
void ctrl_tick(void);
void motor_tick(uint8_t standstill);

#define TIM1_POL 0b0
#define TIM1_POL_MSK 0b101010101010
#define NUM_CTRL_D_BUF 4

#define PWMIN2TARG_N 2
#define PWMIN2TARG_D 13

#define MIN_PWMIN_PULSE 500
#define MAX_PWMIN_PULSE 2500
#define PWMIN_LIM 1400

#define BRAKE_CCER 0x444
#define BRAKE_CCR 192

#define MAX_SOFT_START_COEFF 512

#define TARG0 375
#define ENC2TARG_N 18
#define ENC2TARG_D 11
#define CTRL2CCR_N 6
#define CTRL2CCR_D 1
#define MAX_CCR 1200
#define MAX_TICK 70
#define HALL_TICK_BLACKOUT 3 // number of TIM6 cycles to blackout on timer-based ticks
#define PCCR 220
// Note about PCC:
// high-side PWM just to recharge bootstrap cap; also sets absolute max CCR
// @2 downsampling, 55ccr = 260us off-time -> given 10-ohm ESR cap, 63us time constant, we're fine (2%)

//#define CTRL_I_N 1
//#define CTRL_I_D (TIM6_FREQ * 4)
//// fraction of max ccr for max I ctrl
//#define MAX_CTRL_I_N 5
//#define MAX_CTRL_I_D 7
//#define MAX_CTRL_I (CTRL_I_D * MAX_CCR / CTRL_I_N * CTRL2CCR_D / CTRL2CCR_N * MAX_CTRL_I_N / MAX_CTRL_I_D)
//#define CTRL_P_N 1
//#define CTRL_P_D 1

#define TICK2RADS 149599.65f // numerator converting ticks in 1/us to rad/s
#define VBUS_ADC2V 0.00893f
#define Kv 26098.1f // at 8.1V, %PWM-256 * (us / tick) (includes drag/damping friction)
#define Ki 93028.77894443E-12 // at 8.1V, tick/us^2/%PWM
#define TAU0 284.032f
#define TAU2VOLT 0.0032747f

#endif /* MOTOR_H_ */
