#ifndef __USER_MATH__
#define __USER_MATH__

#include "math.h"

#define PI 3.1415926f
#define SQRT1_2 0.70710678118f  // 1/sqrt(2)
#define SQRT2   1.41421356237f  // sqrt(2)

#define RAD_S_TO_RPM (30.0f/PI)
#define RPM_TO_RAD_S (PI / 30.0f) //Rpm转化为角度度
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI / 180.0f) //角度转化为弧度
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / PI) //弧度转化为角度
#endif

#endif // !__USER_MATH__