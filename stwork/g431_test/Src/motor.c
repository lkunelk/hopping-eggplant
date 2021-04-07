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

volatile uint16_t pwmin = 0;
extern volatile uint8_t abz;
volatile uint16_t abzs[1024] = { 0 };
volatile uint16_t abzs_idx = 0;
volatile uint8_t last_abz = -1;
#define POT_HOME 946
volatile int16_t e = 0;
volatile uint32_t ticks = 0;
volatile int32_t diff_i = 0;
#define DIFF_BUF_LEN 16
volatile int16_t diff_buf[DIFF_BUF_LEN] = { 0 };
volatile uint8_t diff_buf_idx = 0;
#define CTRL_P_N 1
#define CTRL_P_D 10
#define CTRL_I_N 1
#define CTRL_I_D 512
#define CTRL_D_N 1
#define CTRL_D_D 64

volatile uint8_t acceled = 0;
volatile uint8_t downsampler = 0;
void motor_tick(void) {
	ticks++;
	abz = enc();

	// update pot PID states
	int16_t delta = pwmin - POT_HOME;
	diff_buf[(diff_buf_idx++) & (DIFF_BUF_LEN - 1)] = delta;
	diff_i += delta;
	volatile int16_t ctrl = 110; // delta * CTRL_P_N / CTRL_P_D + diff_i * CTRL_I_N / CTRL_I_D;

	volatile uint8_t state = mod6[hall2state[abz] + (ctrl > 0 ? 0 : 3) + 4 + (last_abz == -1)]; // + (abz != last_abz && last_abz != -1) // ctrl = 0 -> stall
//	volatile uint16_t ctrl_ = abs((ctrl * CTRL2CCR_N) / CTRL2CCR_D);
//	ctrl_ = min(MAX_CCR, ctrl_); // ((ticks) & 0xFF) * MAX_CCR / 0xFF; // 80; // (pwmin - MIN_PWMIN_PULSE) * PWMIN2TARG_N / PWMIN2TARG_D; //

	uint16_t nccr = min(MAX_CCR, max(10, abs(ctrl))); // !acceled *
	uint16_t ccer = (state_en[state] & ~TIM1_POL_MSK) | TIM1_POL;

	// <timer critical region>
	htim1.Instance->CCR1 = htim1.Instance->CCR2 = htim1.Instance->CCR3 = nccr;
	TIM1->CCER = ccer;
	// </timer critical region>

//	if(!acceled && ticks > 24000) {
//		acceled = 1;
//		ticks = 0;
//	}
	if(abzs_idx < 1024 && ((downsampler & 3) == 0)) { // && // acceled &&
		abzs[abzs_idx++] = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
	}
	downsampler++;

	last_abz = abz;

//	else {
//		soft_start_coeff = 0;
//		TIM1->CCER = BRAKE_CCER;
//		TIM1->CCR1 = BRAKE_CCR;
//		TIM1->CCR2 = BRAKE_CCR;
//		TIM1->CCR3 = BRAKE_CCR;
//	}
}
