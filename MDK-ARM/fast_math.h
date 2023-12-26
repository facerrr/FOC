#ifndef FASTMATH_H
#define FASTMATH_H
#include "main.h"
#include <math.h>

#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559

typedef uint8_t  u8;
typedef uint16_t u16;

float sin_f32(float x);
float cos_f32(float x);
float bias_abs(float x);


static inline float fmodf_pos(float x, float y)
{
    float out = fmodf(x, y);
    if (out < 0.0f){
        out += y;
	}
    return out;
}

static inline int mod(int dividend, int divisor)
{
    int r = dividend % divisor;
    return (r < 0) ? (r + divisor) : r;
}


static inline float wrap_pm(float x, float pm_range)
{
    return fmodf_pos(x + pm_range, 2.0f * pm_range) - pm_range;
}

#endif



