#ifndef __USER_LIB_H
#define __USER_LIB_H
#include "stdint.h"
#include "arm_math.h"

#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif
#ifndef PI_2
#define PI_2 1.5707963267948966192313216916398f
#endif
#ifndef PI_3
#define PI_3 1.0471975511965977461542144610932f
#endif
#ifndef PI_4
#define PI_4 0.78539816339744830961566084581988f
#endif
#ifndef PI_6
#define PI_6 0.52359877559829887307710723054658f
#endif
#ifndef PI_8
#define PI_8 0.39269908169872415480783042290994f
#endif
#ifndef PI_12
#define PI_12 0.26179938779914943653855361527329f
#endif

float Sqrt(float x);
float float_constrain(float Value, float minValue, float maxValue);
float loop_float_constrain(float Input, float minValue, float maxValue);
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
float rad_format(float Ang);
float Max(float Value1, float Value2);
float Min(float Value1, float Value2);

#define Abs(x) (((x) > 0) ? (x) : (-(x)))

#define LimitOutput(input, min, max) \
    {                                \
        if (input < min)             \
            input = min;             \
        else if (input > max)        \
            input = max;             \
    }

#endif
