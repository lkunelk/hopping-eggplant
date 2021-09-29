/*
 * math_util.c
 *
 *  Created on: May 1, 2020
 *      Author: derek-lam
 */

#include "math_util.h"
#include <stdint.h>
#include <math.h>

int8_t sgn(int32_t x) {
	return x > 0 ? 1 : -1; // (x < 0 ? -1 : 0);
}

int8_t fsgn(float x) {
	return x > 0 ? 1 : -1; // (x < 0 ? -1 : 0);
}
