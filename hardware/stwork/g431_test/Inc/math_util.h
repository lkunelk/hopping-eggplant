/*
 * math_util.h
 *
 *  Created on: May 1, 2020
 *      Author: derek-lam
 */

#ifndef MATH_UTIL_H_
#define MATH_UTIL_H_
#include <stdint.h>

// thanks https://stackoverflow.com/a/3437484/3925507
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

int8_t sgn(int32_t x);

#endif /* MATH_UTIL_H_ */
