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

/*
Considered the inductor energy dumping to decide about deadtime + complimentary
Assuming 100uH coils and ~1ohm of series resistance, time constant around 600us
Therefore using complimentary dumps most of the current to ground
Plus the diodes can take those couple of amps, for the off-time (~10us) energy
  transfer is small enough (since the diode resistance >> motor coil resistance (although isn't that worse re: i2r?))
anyways, worst thing for power is to short to ground so don't do that
 */
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
#define HALL2STATE_INVALID 0 // use legal values for invalid, juuuust in case
// 1 0 2 2 3 1
// 2 0 4 5 7 3
//const uint8_t hall2state[8] = {
//	1, HALL2STATE_INVALID, 0, 5, 2, 3, HALL2STATE_INVALID, 4
//};

// 5 7 3 2 0 4
// 3 2 6 4 5 1
// 5 7 3 2 0 4
const uint8_t hall2state[8] = {
	4, HALL2STATE_INVALID, 3, 2, 5, HALL2STATE_INVALID, 2, 1
};

volatile uint32_t* const ccrs[3] = { &(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3) };
volatile uint32_t* const state_ccr[6][3] = {
//	{ &(TIM1->CCR2), &(TIM1->CCR3), &(TIM1->CCR1) },
//	{ &(TIM1->CCR2), &(TIM1->CCR1), &(TIM1->CCR3) },
//	{ &(TIM1->CCR3), &(TIM1->CCR1), &(TIM1->CCR2) },
//	{ &(TIM1->CCR3), &(TIM1->CCR2), &(TIM1->CCR1) },
//	{ &(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3) },
//	{ &(TIM1->CCR1), &(TIM1->CCR3), &(TIM1->CCR2) },
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

#define PWMIN_HOME 123
volatile uint16_t pwmin = 0;
extern volatile uint8_t abz;
volatile int16_t abzs[256] = { 0 };
volatile uint16_t abzs_idx = 0;
volatile uint8_t last_abz = 0xFF;
#define POT_HOME 2234
volatile int16_t e = 0;
volatile uint32_t ticks = 0;
volatile int32_t diff_i = 0;

volatile float speed_avg = 0;

volatile int32_t delta_avg = 0;
#define DIFF_BUF_LEN 4
#define LOG_DIFF_BUF_LEN 2
const int32_t DIFF_COEFFS[DIFF_BUF_LEN] = { -85, 384, -768, 469 };
volatile int16_t diff_buf[DIFF_BUF_LEN] = { 0 };
volatile uint8_t diff_buf_idx = 0;
volatile int32_t diff_d = 0;
#define CTRL_GAIN 6.0f
#define CTRL_P 0.3f
#define CTRL_I 0.00002f
#define CTRL_D 0.2f

volatile uint8_t acceled = 0;
volatile uint8_t downsampler = 0;

volatile int16_t ctrl;
volatile uint32_t ctrl_ticks;

void ctrl_tick() {
	abz = enc();

	int8_t state_diff = hall2state[abz] - hall2state[last_abz];
	int8_t fwd = state_diff <= -3 || (state_diff > 0 && state_diff < 3);
	uint8_t _stopped = _motor_stopped();

	if(_stopped || (state_diff != 0 && __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1) > 0)) {
		speed_avg = _stopped ? 0 : (speed_avg * 3 + (fwd ? Kv : -Kv) / (float)max(1, __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1))) / 4; // filter speed a little with moving avg
	}

	// update pot PID states
	int16_t delta = pwmin - PWMIN_HOME;

	if(ctrl_ticks > 0)
		delta_avg = (delta_avg * 7 + delta) / 8;
	else
		delta_avg = delta;

	if((ctrl_ticks & 7) == 0) {
		diff_buf[(diff_buf_idx++) & (DIFF_BUF_LEN - 1)] = delta_avg;
		diff_d = 0;
		for(uint8_t i = 0; i < DIFF_BUF_LEN; i++) {
			diff_d += diff_buf[(diff_buf_idx + i) & (DIFF_BUF_LEN - 1)] * DIFF_COEFFS[i];
		}
		diff_d >>= LOG_DIFF_BUF_LEN;

		if(abzs_idx < 256) { //  && ((downsampler & 3) == 0) // && // acceled &&
			abzs[abzs_idx++] = diff_d; // (__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1) << 1) | _stopped;
		}
//		else {
//			ctrl_ticks++;
//		}
	}

	diff_i += delta;
	volatile float torque = (delta * CTRL_P + diff_i * CTRL_I + diff_d * CTRL_D) * CTRL_GAIN; // in duty-256-units of torque
	if(_stopped) {
		torque *= 2.0f;
	}
	ctrl = speed_avg + torque;

	// !_stopped && //  && state_diff > 0

	downsampler++;
	ctrl_ticks++;
	last_abz = abz;

	if(_stopped) {
		motor_tick(1);
		HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM);
	}
}

volatile uint32_t moved_ = 0;
void motor_tick(uint8_t standstill) {
	ticks++;
	abz = enc();

	// SIGNS:
	// +ctrl => +state
	// +ctrl => CCW disk => CW body torque => +pot
	// +ctrl => +speed_avg

	volatile uint8_t state = mod6[
		hall2state[abz]
//		+ (ctrl > 0 ? 0 : 3) // direction reversal
		+ 3
		+ 2 // hall-motor offset
		+ (standstill
			? 0 // if at standstill, torque stationary
			: ((speed_avg > 0) ? 1 : -1) // else, assume the speed shows the next direction, plan the next pulse accordingly
		)
	];

	volatile uint16_t nccr = min(MAX_CCR, abs(ctrl) + 35); // !acceled *
	uint16_t ccer = (state_en[state] & ~TIM1_POL_MSK) | TIM1_POL;

	nccr = min(MAX_CCR, (ticks >> 3) + 200);
	if(!standstill && !moved_) {
		moved_ = nccr;
	}
	if(moved_) {
		nccr = 0;
	}

	// <timer critical region>
//	htim1.Instance->CCR1 = htim1.Instance->CCR2 = htim1.Instance->CCR3 = 0;
	htim1.Instance->CCR1 = htim1.Instance->CCR2 = htim1.Instance->CCR3 = nccr;
	TIM1->CCER = ccer;

	if(moved_) {
		nccr = 0;
	}

	// </timer critical region>

	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_TRIGGER);
}
