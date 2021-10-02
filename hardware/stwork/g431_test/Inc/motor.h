/*
 * motor.h
 *
 *  Created on: May 22, 2020
 *      Author: derek-lam
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"
#include <math.h>

typedef enum {
	FWD_T_BKWD = 0, FWD_T_FWD, FWD_T_STILL, NUM_FWD_TS
} fwd_t;

typedef struct imu_t {
	int32_t tilt;
	int32_t tilt_err;
	int32_t gyro;
} imu_t;

inline uint8_t _motor_stopped(void) {
	return __HAL_TIM_GET_COUNTER(&htim4) == 0 && __HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_TRIGGER) == RESET;
}
void ctrl_tick(void);
void motor_tick(void);
void motor_to(uint16_t theta, uint8_t r);

#define TIM1_POL 0b0
#define TIM1_POL_MSK 0b101010101010
#define NUM_CTRL_D_BUF 4

#define PWMIN2TARG_N 2
#define PWMIN2TARG_D 13

#define MIN_PWMIN_PULSE 500
#define MAX_PWMIN_PULSE 2500
#define PWMIN_LIM 1700

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

#define UART_RX_BUF_SIZE 4
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

#define Kv 94.52f // (rad/s)/V
#define Kv_INV 0.0106f // V/(rad/s)
#define TAU0 0.0f // 113.1133f // 284.032f
#define TAU2VOLT 0.003233f // V / (rad/s^2)

#define PWMIN_HOME 925
#define TILT_HOME 573
#define GYRO0 2048

#define CTRL_GAIN 1.0f
// P units: (rad/s^2) / (tilt LSB)
#define CTRL_P 10.0f // 10.99f
// D units: (rad/s^2) / (gyro LSB)
#define CTRL_D 1.5f //  // 1.466f // 0.391f
// FLYV units: (rad/s^2) / (rad/s)
#define CTRL_FLYV -1.0f
#define CTRL_I 0.01f

///////////////////////////////////////

#define MECH2ELEC 7 // ratio of elec to mech period
#define ENC_MOD_BASE 293 // modulo base of encoder -> electrical period
#define MECH_OFFSET 0 // encoder ticks at 0 theta

#define THETA_360DEG 512
#define THETA_270DEG 384
#define THETA_180DEG 256
#define THETA_120DEG 171
#define THETA_90DEG 128
#define THETA_60DEG 85
#define THETA_45DEG 64
#define THETA_30DEG 43
#define N_SIN_2 512
#define N_SIN_2_MASK 0x1FF

#define MOTOR_TICK_PERIOD (90E-6f)
#define CTRL_N_TICKS 11
#define CTRL_PERIOD (CTRL_N_TICKS * MOTOR_TICK_PERIOD)
#define CTRL_ENC2RADS (2 * M_PI / CTRL_PERIOD / 2048) // assume these are optimized by GCC: it's not the preprocessor that does it sadly
#define CTRL_RADS2ENC_MOTOR_TICK (1 / CTRL_ENC2RADS / CTRL_N_TICKS)

#define ACTIVE_BRAKE_MAX 100.0f // max rad/s to continue using active braking

#endif /* MOTOR_H_ */
