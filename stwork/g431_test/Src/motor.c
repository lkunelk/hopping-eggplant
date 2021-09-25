/*
 * motor.c
 *
 *  Created on: May 22, 2020
 *      Author: derek-lam
 */

#include "enc.h"
#include "main.h"
#include "motor.h"
#include "math_util.h"
#include "adc.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/*
Considered the inductor energy dumping to decide about deadtime + complimentary
Assuming 100uH coils and ~1ohm of series resistance, time constant around 600us
Therefore using complimentary dumps most of the current to ground
Plus the diodes can take those couple of amps, for the off-time (~10us) energy
  transfer is small enough (since the diode resistance >> motor coil resistance (although isn't that worse re: i2r?))
anyways, worst thing for power is to short to ground so don't do that
 */

#define PWMIN_HOME 925
#define TILT_HOME 115
volatile uint16_t adc1_reg[8] = { 0 };
volatile uint8_t uart_rx_buf[UART_RX_BUF_SIZE] = { 0 };
volatile uint8_t uart_rx_valid = 0;
volatile imu_t imu = { 0 };
volatile imu_t imu_avg = { 0 };
volatile uint16_t pwmin = 0;
volatile uint8_t abz;
volatile uint16_t abzs[16] = { 0 };
volatile uint32_t abzs_idx = 0;
volatile uint8_t last_abz = 0;
#define POT_HOME 2234
volatile int16_t e = 0;
volatile int32_t diff_i = 0;

volatile fwd_t fwd = 0;
volatile float speed_avg = 0, next_speed = 0;

volatile int32_t delta_avg = 0;
#define DIFF_BUF_LEN 4
#define LOG_DIFF_BUF_LEN 2
const int32_t DIFF_COEFFS[DIFF_BUF_LEN] = { -85, 384, -768, 469 };
volatile int16_t diff_buf[DIFF_BUF_LEN] = { 0 };
volatile uint8_t diff_buf_full_ = 0;
volatile uint8_t diff_buf_idx = 0;
volatile int32_t diff_d = 0;

#define NUM_VCAL 10
const float VCAL[2][2][NUM_VCAL] = {
	{ // BACKWARDS
		{ 0.9537440843592074f, 9.59268823e-01f, 1.15130663e+00f, 1.92385442f  , 2.68662071f  , 3.44659201f , 4.20293755f  , 4.9560314f   , 5.71134057f  , 105.71134056698827f},
		{ 0.0f               , 6.76634206f    , 46.78531916f   , 210.65292091f, 356.51508298f, 459.4140404f, 538.26692847f, 605.01989394f, 637.74173513f, 4103.537454325323f},
	},
	{ // FORWARDS
		{ 0.9537440843592074f, 9.59268823e-01f, 1.15130663e+00f, 1.92300616e+00f, 2.68423498e+00f, 3.44232523e+00f, 4.19633929e+00f, 4.94592952e+00f, 5.69879707e+00f, 105.69879706888129f },
		{ 0.0f               , 6.76634206f    , 46.78531916f   , 218.15946086f  , 378.14152672f  , 498.62810727f  , 587.30544773f  , 658.10862762f  , 688.8220895f   , 3952.446568428597f },
	}
};

//#define CTRL_GAIN 6.0f
//#define CTRL_P 0.6f
//#define CTRL_I 0.00002f
//#define CTRL_D 0.1f // 0.2f

#define CTRL_GAIN 1.0f
#define CTRL_P 4.43f // 2.897f // 0.737f
#define CTRL_D 22.13f // 22.13f // 0.907f // 1.466f // 0.391f
#define CTRL_FLYV -0.53f // -0.30508f
#define CTRL_I 0.0f

volatile uint8_t acceled = 0;
volatile uint8_t downsampler = 0;

volatile float ctrl;
volatile uint32_t ctrl_ticks;
volatile float torque = 0.0f;

volatile uint16_t standstill_ccr0 = 0;
//volatile uint16_t stopped_counter = 0;
volatile float stopped_ctrl[64] = { 0 };
volatile float stopped_torque[64] = { 0 };
volatile uint8_t stopped_data_idx = 0;

void ctrl_tick() {
	int8_t uart_rx_buf_[UART_RX_BUF_SIZE] = { 0 };
	memcpy((void*)uart_rx_buf_, (void*)uart_rx_buf, sizeof(uart_rx_buf[0]) * UART_RX_BUF_SIZE);
	if(uart_rx_valid) {
		for(uint8_t i = 0; i < UART_RX_BUF_SIZE; i++) {
			uint16_t v = (uart_rx_buf_[i] >> 2) & 0x3F;
			switch(uart_rx_buf_[i] & 0b11) {
				case 0: // GYRO LSB
					imu.gyro = (imu.gyro & (~0x3F)) | v;
					break;
				case 1: { // GYRO MSB
					volatile int16_t gyro = ((imu.gyro & (~(0x3F << 6))) | (v << 6)) << 4;
					imu.gyro = gyro >> 4;
					break;
				}
				case 2: // TILT LSB
					imu.tilt = (imu.tilt & (~0x3F)) | v;
					break;
				case 3: { // TILT MSB
					uint16_t tilt = (imu.tilt & (~(0x3F << 6))) | (v << 6);
					// tilt = (tilt << 4) >> 4; // propagate sign bit
					imu.tilt = tilt;
					break;
				}
			}
		}

		standstill_ccr0 = VCAL[0][0][0] / (VBUS_ADC2V * adc1_reg[1]) * (float)__HAL_TIM_GET_AUTORELOAD(&htim1);

		uint8_t _stopped = _motor_stopped();
		if(1) {
			uint16_t delta_t = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
			next_speed = delta_t > 0 ? (TICK2RADS / (float)delta_t) : 0.0f;
			switch(fwd) {
				case FWD_T_FWD:
					break;
				case FWD_T_BKWD:
					next_speed *= -1.0f;
					break;
				case FWD_T_STILL:
					next_speed = 0.0f;
					break;
			}

			speed_avg = _stopped ? 0 : (speed_avg * 15 + next_speed) / 16; // filter speed a little with moving avg

			float bemf_ccr = 0.0f;
			if(adc1_reg[1] > 0) {
				float bemf = 0.0f;
				if(fwd != FWD_T_STILL) {
					uint8_t fwd_ = fwd == FWD_T_FWD ? 1 : 0; // also stash to avoid preemption
					for(uint8_t i = 1; i < NUM_VCAL; i++) {
						if(speed_avg < VCAL[fwd_][1][i]) {
							bemf = VCAL[fwd_][0][i] + (speed_avg - VCAL[fwd_][1][i - 1]) / (VCAL[fwd_][1][i] - VCAL[fwd_][1][i - 1]) * (VCAL[fwd_][0][i] - VCAL[fwd_][0][i - 1]);
							break;
						}
					}
				}
				bemf_ccr = bemf / (VBUS_ADC2V * adc1_reg[1]) * (float)__HAL_TIM_GET_AUTORELOAD(&htim1);
			}

			// update pot PID states

			imu_avg.tilt = ((int16_t)imu_avg.tilt * 3 + (imu.tilt - TILT_HOME)) / 4;
			imu_avg.gyro = ((int16_t)imu_avg.gyro * 3 + imu.gyro) / 4;

	//		if((ctrl_ticks & 0) == 0) {
	//			diff_buf[(diff_buf_idx++) & (DIFF_BUF_LEN - 1)] = delta_avg;
	//			diff_d = 0;
	//			diff_buf_full_ |= diff_buf_idx > DIFF_BUF_LEN;
	//			if(diff_buf_full_) {
	//				for(uint8_t i = 0; i < DIFF_BUF_LEN; i++) {
	//					diff_d += diff_buf[(diff_buf_idx + i) & (DIFF_BUF_LEN - 1)] * DIFF_COEFFS[i];
	//				}
	//				diff_d >>= LOG_DIFF_BUF_LEN;
	//			}
	//		}

			torque = (imu_avg.tilt * CTRL_P + imu_avg.gyro * CTRL_D + speed_avg * CTRL_FLYV) * CTRL_GAIN; // in duty-256-units of torque // diff_i * CTRL_I +
			torque = (torque + TAU0 * (torque > 0 ? 1 : -1)) * TAU2VOLT / (adc1_reg[1] * VBUS_ADC2V) * __HAL_TIM_GET_AUTORELOAD(&htim1);
			if(_stopped) {
				torque *= 1.2f;
			}

			if(0) {
				if(imu.tilt > 0 && imu.tilt < 0xFF)
					ctrl = -(speed_avg - torque);
				else
					ctrl = 0;
			}

	//		if(abzs_idx < 256) { //  && ((downsampler & 3) == 0) // && // acceled &&
	//			abzs[abzs_idx++] = diff_d; // (__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1) << 1) | _stopped;
	//		}

			// !_stopped && //  && state_diff > 0

			downsampler++;
			ctrl_ticks++;

			if(_stopped && stopped_data_idx >= 64) {
				stopped_data_idx = 0;
			}
			else if(stopped_data_idx < 64) {
				stopped_ctrl[stopped_data_idx] = ctrl;
				stopped_torque[stopped_data_idx] = torque;
				stopped_data_idx++;
			}
			else {
				stopped_data_idx = 64;
			}
		}

		if(_stopped) {
			motor_tick(1);
			HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM);
		}
	}
	else {
		ctrl = 0;
	}
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

const uint16_t state_en[6] = {
//	0b010000010000, // -+0
//	0b000000010100, // 0+-
//	0b000100000100, // +0-
//	0b000101000000, // +-0
//	0b000001000001, // 0-+
//	0b010000000001, // -0+

	0b010000010000, // -+0
	0b000000010100, // 0+-
	0b000100000100, // +0-
	0b000101000000, // +-0
	0b000001000001, // 0-+
	0b010000000001, // -0+
};
volatile uint32_t* const state_ccr_[6][3] = {
	{&(TIM1->CCR2), &(TIM1->CCR3), &(TIM1->CCR1)}, // common CCR going bckwd to here, "" fwd to here, free
	{&(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3)},
	{&(TIM1->CCR3), &(TIM1->CCR1), &(TIM1->CCR2)},
	{&(TIM1->CCR2), &(TIM1->CCR3), &(TIM1->CCR1)},
	{&(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3)},
	{&(TIM1->CCR3), &(TIM1->CCR1), &(TIM1->CCR2)},
};
#define HALL2STATE_INVALID 0 // use legal values for invalid, juuuust in case
// 1 0 2 2 3 1
// 2 0 4 5 7 3
const uint8_t hall2state[8] = {
	4, HALL2STATE_INVALID, 3, 2, 5, 0, HALL2STATE_INVALID, 1
};
//const uint8_t hall2state[8] = {
//	1, HALL2STATE_INVALID, 2, 3, 0, 5, HALL2STATE_INVALID, 4
//};

// 5 7 3 2 0 4
// 3 2 6 4 5 1
// 5 7 3 2 0 4

//const uint8_t hall2state[8] = {
//	4, HALL2STATE_INVALID, 3, 2, 5, HALL2STATE_INVALID, 2, 1
//};

volatile uint32_t* const ccrs[3] = { &(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3) };
volatile uint32_t* const state_ccr[6][3] = {
	{ &(TIM1->CCR2), &(TIM1->CCR3), &(TIM1->CCR1) },
	{ &(TIM1->CCR2), &(TIM1->CCR1), &(TIM1->CCR3) },
	{ &(TIM1->CCR3), &(TIM1->CCR1), &(TIM1->CCR2) },
	{ &(TIM1->CCR3), &(TIM1->CCR2), &(TIM1->CCR1) },
	{ &(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3) },
	{ &(TIM1->CCR1), &(TIM1->CCR3), &(TIM1->CCR2) },
};
const uint8_t mod6[18] = { 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5 };
//volatile int32_t ctrl_i = 0, ctrl_p = 0, ctrl_d_buf[NUM_CTRL_D_BUF] = { 0 };
//volatile uint8_t ctrl_d_idx = 0;

volatile uint32_t ticks = 0;
volatile uint16_t speed_buf[64] = { 0 };
volatile uint8_t speed_buf_idx = 0;
volatile uint16_t v_buf[64] = { 0 };
volatile uint8_t v_buf_idx = 0;

volatile uint32_t moved_ = 0;
const uint8_t FWD_OFFSETS[NUM_FWD_TS] = { 1, 3, 2 };
volatile fwd_t last_fwd = FWD_T_STILL;
void motor_tick(uint8_t standstill) {
	if(!standstill) {
		__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_TRIGGER); // clear the interrupt that triggers whenever TIM1 resets (either by overflow or by hall tick)
	}

	ticks++;
	abz = enc();

//	if(abz != last_abz) {
//		abzs[(abzs_idx++) & 15] = abz;
//	}
//	last_abz = abz;

	int8_t state_diff = (int8_t)hall2state[abz] - (int8_t)hall2state[last_abz];
	if(standstill) {
		fwd = FWD_T_STILL;
	}
	else if(state_diff <= -3 || (state_diff > 0 && state_diff < 3)) {
		fwd = FWD_T_FWD;
	}
	else {
		fwd = FWD_T_BKWD;
	}

	// SIGNS:
	// +ctrl => +state
	// +ctrl => CCW disk => CW body torque => +pot
	// +ctrl => +speed_avg

	if(1) {
		ctrl = 300;
		if(HAL_GetTick() > 3000) {
			ctrl = -300;
	//		uint16_t abzs_idx_ = abzs_idx >> 4;
	//		if(abzs_idx_ < 1024 && !standstill) {
	//			abzs[abzs_idx_] = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
	//			abzs_idx++;
	//		}
	////		else if(abzs_idx_ >= 1024) {
	////			nccr = 0;
	////		}
		}
		if(HAL_GetTick() > 6000 || (HAL_GetTick() > 3000 && fabs(speed_avg) > 400)) {
			ctrl = 0;
		}
	}

//	ctrl = 300;

	uint8_t state_base = hall2state[abz] + (ctrl > 0 ? 0 : 3);
	volatile uint8_t state = (state_base + FWD_OFFSETS[fwd]) % 6;
	volatile uint8_t state_still = (state_base + FWD_OFFSETS[FWD_T_STILL]) % 6;

	if(last_fwd != fwd && last_fwd != FWD_T_STILL && fwd != FWD_T_STILL) {
		// the currently loaded energization is wrong: fix it immediately
		uint16_t ccer = (state_en[state_still] & ~TIM1_POL_MSK) | TIM1_POL; // force the correct energization for the current hall sensor
		TIM1->CCER = ccer;
		HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM); // commutate immediately
	}

	volatile uint16_t nccr = min(MAX_CCR, fabs(ctrl)); // (standstill ? standstill_ccr0 : 0)); // !acceled *

	uint16_t ccer = (state_en[state] & ~TIM1_POL_MSK) | TIM1_POL;

//	nccr = min(MAX_CCR, 1000);
//	uint16_t abzs_idx_ = (abzs_idx + 14) >> 4;
//	if(abzs_idx_ < 1024 && !standstill) {
//		abzs[abzs_idx_] = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
//		abzs_idx++;
//	}
//	else if(abzs_idx_ >= 1024) {
//		nccr = 0;
//	}

	// <timer critical region>

	*state_ccr[state_still][0] = nccr;
	*state_ccr[state_still][2] = nccr;
	*state_ccr[state_still][1] = __HAL_TIM_GET_AUTORELOAD(&htim1);
//	htim1.Instance->CCR1 = htim1.Instance->CCR2 = htim1.Instance->CCR3 = nccr;
	TIM1->CCER = ccer;

	// </timer critical region>

	last_fwd = fwd;

	if(!standstill) {
		last_abz = abz;
	}
}

/**
 * Misc useful timer critical region functions
 */

//	if(HAL_GetTick() > 7000) {
//		nccr = 0;
//		uint16_t abzs_idx_ = (abzs_idx + 3) >> 2;
//		if(abzs_idx_ < 1024 && !standstill) {
//			abzs[abzs_idx_] = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
//			abzs_idx++;
//		}
////		else if(abzs_idx_ >= 1024) {
////			nccr = 0;
////		}
//	}

// // ---

//	if(HAL_GetTick() < 20000) {
//		nccr = min(MAX_CCR, 300);
//		speed_buf[(speed_buf_idx++) & 63] = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
//		v_buf[(v_buf_idx++) & 63] = adc1_reg[1];
//	}
//	else {
//		nccr = 0;
//	}

// // ---

//	htim1.Instance->CCR1 = htim1.Instance->CCR2 = htim1.Instance->CCR3 = 0;

// // ---

//	{
		// tried to make one of the coil terminals hold its level at 100% duty
		// to improve the inductor charging, but this doesn't actually make that big of a
		// difference to the off-phase of the PWM, only improving the coil discharge rate
		// by the FET diode turn-on voltage, plus for many speeds it may discharge completely
		// anyways. This introduced timing and prediction problems trying to guess the next state
		// and made the motor less efficient, so I'm abandoning this for now.
//		uint8_t next_state = state; // standstill ? mod6[state + (ctrl > 0 ? 1 : 5)] : state;
//		uint8_t fwd_ = 1; // standstill ? ctrl > 0 : fwd;
//
//		*state_ccr_[next_state][fwd_] = nccr;
//		*state_ccr_[next_state][!fwd_] = *state_ccr_[next_state][2] = 1024;
//	}
