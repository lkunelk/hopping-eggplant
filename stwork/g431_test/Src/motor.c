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

volatile uint8_t acceled = 0;
volatile uint8_t downsampler = 0;

volatile int16_t ctrl;
volatile uint32_t ctrl_ticks;
volatile float torque_volts = 0.0f;
volatile float bemf_volts = 0.0f;

volatile uint16_t standstill_ccr0 = 0;
//volatile uint16_t stopped_counter = 0;
volatile float stopped_ctrl[64] = { 0 };
volatile float stopped_torque[64] = { 0 };
volatile uint8_t stopped_data_idx = 0;

volatile int16_t last_enc_pos = 0;
volatile int32_t enc_revs = 0;

extern volatile uint8_t ctrl_on;

void ctrl_tick() {
	int8_t uart_rx_buf_[UART_RX_BUF_SIZE] = { 0 };
	memcpy((void*)uart_rx_buf_, (void*)uart_rx_buf, sizeof(uart_rx_buf[0]) * UART_RX_BUF_SIZE);
	if(uart_rx_valid) {
		for(uint8_t i = 0; i < UART_RX_BUF_SIZE; i++) {
			uint32_t v = (uart_rx_buf_[i] >> 2) & 0x3F;
			switch(uart_rx_buf_[i] & 0b11) {
				case 0: // GYRO LSB
					imu.gyro = (imu.gyro & (~0x3F)) | v;
					break;
				case 1: { // GYRO MSB
					imu.gyro = ((imu.gyro & (~(0x3F << 6))) | (v << 6));
					break;
				}
				case 2: // TILT LSB
					imu.tilt = (imu.tilt & (~0x3F)) | v;
					break;
				case 3: { // TILT MSB
					// tilt = (tilt << 4) >> 4; // propagate sign bit
					imu.tilt = (imu.tilt & (~(0x3F << 6))) | (v << 6);
					break;
				}
			}
		}

		standstill_ccr0 = VCAL[0][0][0] / (VBUS_ADC2V * adc1_reg[1]) * (float)__HAL_TIM_GET_AUTORELOAD(&htim1);

		uint8_t _stopped = _motor_stopped();
		if(1) {
			if(0) {
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
			}

			{
				int16_t enc_pos = __HAL_TIM_GET_COUNTER(&htim4);
				int16_t delta = enc_pos - last_enc_pos;
				last_enc_pos = enc_pos;
				if(delta > 1024) { // wraparound
					delta = delta - 2048;
					enc_revs--;
				}
				else if(delta < -1024) {
					delta = 2048 + delta;
					enc_revs++;
				}
				speed_avg = (speed_avg * 15 + delta * (float)CTRL_ENC2RADS) / 16; // filter speed a little with moving avg
			}

			float v_bat = (adc1_reg[1] * VBUS_ADC2V);

			if(adc1_reg[1] > 0) {
				if(0) {
					if(fwd != FWD_T_STILL) {
						uint8_t fwd_ = speed_avg > 0; // fwd == FWD_T_FWD ? 1 : 0; // also stash to avoid preemption
						for(uint8_t i = 1; i < NUM_VCAL; i++) {
							if(speed_avg < VCAL[fwd_][1][i]) {
								bemf_volts = VCAL[fwd_][0][i] + (speed_avg - VCAL[fwd_][1][i - 1]) / (VCAL[fwd_][1][i] - VCAL[fwd_][1][i - 1]) * (VCAL[fwd_][0][i] - VCAL[fwd_][0][i - 1]) - VCAL[fwd_][0][0];
								break;
							}
						}
					}
				}

				bemf_volts = speed_avg * Kv_INV;
			}

			// update pot PID states

			imu_avg.tilt = ((int16_t)imu_avg.tilt * 3 + (imu.tilt - TILT_HOME)) / 4;
			imu_avg.tilt_err += imu_avg.tilt;
			int32_t max_int_error = v_bat * (MAX_CCR / (CTRL_I * TAU2VOLT) / 2048.0f);
			imu_avg.tilt_err = fmin(fmax(imu_avg.tilt_err, -max_int_error), max_int_error);
			imu_avg.gyro = ((int16_t)imu_avg.gyro * 3 + (imu.gyro - GYRO0)) / 4;

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

			// TODO: question, will TAU0 be defined in terms of MAX_CCR then or 100% duty? because for safety the `motor_to` `r` is normalized 0-255 mapping to 0-
			// this torque step
			float accel = (imu_avg.tilt * CTRL_P + imu_avg.gyro * CTRL_D + imu_avg.tilt_err * CTRL_I + speed_avg * CTRL_FLYV) * CTRL_GAIN;
			// need to update TAU0 and TAU2VOLT with torque tests
			torque_volts = accel * TAU2VOLT; // __HAL_TIM_GET_AUTORELOAD(&htim1)
			if(speed_avg < 0.5f) {
				torque_volts *= 1.2f;
			}

			if(ctrl_on) {
				if((imu_avg.tilt + TILT_HOME) > 3 && (imu_avg.tilt + TILT_HOME) < (1024-3)) {
					// convert from volts to motor R units (0->255 = 0->MAX_CCR)
					float ctrl_volts = (bemf_volts + torque_volts);
					ctrl = ctrl_volts / v_bat * (2048.0f / (float)MAX_CCR * 256.0f);

				}
				else
					ctrl = 0;
			}

			downsampler++;
			ctrl_ticks++;
		}
	}
	else {
		ctrl = 0;
	}
}

volatile uint32_t motor_ticks = 0;
void motor_tick(void) {
	++motor_ticks;
	 // (speed_avg * CTRL_RADS2ENC_MOTOR_TICK * motor_ticks)
	int16_t enc_raw = -(((int16_t)__HAL_TIM_GET_COUNTER(&htim4)) - MECH_OFFSET);
	uint16_t enc_theta = (modpos(enc_raw, ENC_MOD_BASE) * THETA_360DEG) / ENC_MOD_BASE;
	int16_t ctrl_ = ctrl; // -50;
	int16_t theta_offset = (ctrl_ > 0 ? (THETA_270DEG - 20) : (THETA_360DEG + THETA_90DEG + 20));
//	theta_offset = (theta_offset * 2047 + theta_offset_setpoint) / 2048;

	if((ctrl_ > 0) != (speed_avg > 0)) {
		ctrl_ = fmax(0, ACTIVE_BRAKE_MAX - fabs(speed_avg)) / ACTIVE_BRAKE_MAX * (float)ctrl_;
	}

	motor_to((enc_theta + theta_offset) & 0x1FF, min(abs(ctrl_), 0xFF)); // (sgn(ctrl_) != fsgn(speed_avg)) ? 0 :

	if(motor_ticks == CTRL_N_TICKS) {
		ctrl_tick();
		motor_ticks = 0;
	}
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//#define Q1_TOP 256
//#define Q2_TOP 512
//#define Q3_TOP 768
//#define Q4_TOP 1024
//#define Q5_TOP 1280

const uint16_t state_en_[6] = {
	0b000101011100, // +o-
	0b000111000101, // +-o
	0b010111000001, // o-+
	0b110001010001, // -o+
	0b110000010101, // -+o
	0b010100011100, // o+-
};

#define SIN_2_SHIFT 15
const int16_t SIN_2[N_SIN_2] = { 0, 402, 804, 1206, 1608, 2009, 2410, 2811, 3212, 3612, 4011, 4410, 4808, 5205, 5602, 5998, 6393, 6786, 7179, 7571, 7962, 8351, 8739, 9126, 9512, 9896, 10278, 10659, 11039, 11417, 11793, 12167, 12539, 12910, 13279, 13645, 14010, 14372, 14732, 15090, 15446, 15800, 16151, 16499, 16846, 17189, 17530, 17869, 18204, 18537, 18868, 19195, 19519, 19841, 20159, 20475, 20787, 21096, 21403, 21705, 22005, 22301, 22594, 22884, 23170, 23452, 23731, 24007, 24279, 24547, 24811, 25072, 25329, 25582, 25832, 26077, 26319, 26556, 26790, 27019, 27245, 27466, 27683, 27896, 28105, 28310, 28510, 28706, 28898, 29085, 29268, 29447, 29621, 29791, 29956, 30117, 30273, 30424, 30571, 30714, 30852, 30985, 31113, 31237, 31356, 31470, 31580, 31685, 31785, 31880, 31971, 32057, 32137, 32213, 32285, 32351, 32412, 32469, 32521, 32567, 32609, 32646, 32678, 32705, 32728, 32745, 32757, 32765, 32767, 32765, 32757, 32745, 32728, 32705, 32678, 32646, 32609, 32567, 32521, 32469, 32412, 32351, 32285, 32213, 32137, 32057, 31971, 31880, 31785, 31685, 31580, 31470, 31356, 31237, 31113, 30985, 30852, 30714, 30571, 30424, 30273, 30117, 29956, 29791, 29621, 29447, 29268, 29085, 28898, 28706, 28510, 28310, 28105, 27896, 27683, 27466, 27245, 27019, 26790, 26556, 26319, 26077, 25832, 25582, 25329, 25072, 24811, 24547, 24279, 24007, 23731, 23452, 23170, 22884, 22594, 22301, 22005, 21705, 21403, 21096, 20787, 20475, 20159, 19841, 19519, 19195, 18868, 18537, 18204, 17869, 17530, 17189, 16846, 16499, 16151, 15800, 15446, 15090, 14732, 14372, 14010, 13645, 13279, 12910, 12539, 12167, 11793, 11417, 11039, 10659, 10278, 9896, 9512, 9126, 8739, 8351, 7962, 7571, 7179, 6786, 6393, 5998, 5602, 5205, 4808, 4410, 4011, 3612, 3212, 2811, 2410, 2009, 1608, 1206, 804, 402, 0, -402, -804, -1206, -1608, -2009, -2410, -2811, -3212, -3612, -4011, -4410, -4808, -5205, -5602, -5998, -6393, -6786, -7179, -7571, -7962, -8351, -8739, -9126, -9512, -9896, -10278, -10659, -11039, -11417, -11793, -12167, -12539, -12910, -13279, -13645, -14010, -14372, -14732, -15090, -15446, -15800, -16151, -16499, -16846, -17189, -17530, -17869, -18204, -18537, -18868, -19195, -19519, -19841, -20159, -20475, -20787, -21096, -21403, -21705, -22005, -22301, -22594, -22884, -23170, -23452, -23731, -24007, -24279, -24547, -24811, -25072, -25329, -25582, -25832, -26077, -26319, -26556, -26790, -27019, -27245, -27466, -27683, -27896, -28105, -28310, -28510, -28706, -28898, -29085, -29268, -29447, -29621, -29791, -29956, -30117, -30273, -30424, -30571, -30714, -30852, -30985, -31113, -31237, -31356, -31470, -31580, -31685, -31785, -31880, -31971, -32057, -32137, -32213, -32285, -32351, -32412, -32469, -32521, -32567, -32609, -32646, -32678, -32705, -32728, -32745, -32757, -32765, -32767, -32765, -32757, -32745, -32728, -32705, -32678, -32646, -32609, -32567, -32521, -32469, -32412, -32351, -32285, -32213, -32137, -32057, -31971, -31880, -31785, -31685, -31580, -31470, -31356, -31237, -31113, -30985, -30852, -30714, -30571, -30424, -30273, -30117, -29956, -29791, -29621, -29447, -29268, -29085, -28898, -28706, -28510, -28310, -28105, -27896, -27683, -27466, -27245, -27019, -26790, -26556, -26319, -26077, -25832, -25582, -25329, -25072, -24811, -24547, -24279, -24007, -23731, -23452, -23170, -22884, -22594, -22301, -22005, -21705, -21403, -21096, -20787, -20475, -20159, -19841, -19519, -19195, -18868, -18537, -18204, -17869, -17530, -17189, -16846, -16499, -16151, -15800, -15446, -15090, -14732, -14372, -14010, -13645, -13279, -12910, -12539, -12167, -11793, -11417, -11039, -10659, -10278, -9896, -9512, -9126, -8739, -8351, -7962, -7571, -7179, -6786, -6393, -5998, -5602, -5205, -4808, -4410, -4011, -3612, -3212, -2811, -2410, -2009, -1608, -1206, -804, -402 };
volatile uint32_t* const ccrs[3] = { &(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3) };
#define MOTOR_R_SHIFT 8
void motor_to(uint16_t theta, uint8_t r) {
	theta &= 0x1FF;
	uint8_t phase = min(5, theta / (THETA_60DEG + 5));

	uint16_t ccr_range = ((uint32_t)r * MAX_CCR) >> MOTOR_R_SHIFT;
	for(uint8_t i = 0; i < 3; i++) {
		int32_t sin_arg = ((uint16_t)(theta + THETA_30DEG + THETA_120DEG * i)) & N_SIN_2_MASK;
		int32_t r_sin_raw = ((int32_t)SIN_2[sin_arg] * (int32_t)r) >> MOTOR_R_SHIFT;
		uint32_t ccr = ((int32_t)ccr_range + ((r_sin_raw * MAX_CCR) >> SIN_2_SHIFT)) / 2;

		*ccrs[i] = min(MAX_CCR, ccr);
	}

	TIM1->CCER = 0b010101010101; // state_en_[phase]; // 0b010101010101;
}
