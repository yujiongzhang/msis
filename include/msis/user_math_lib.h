#ifndef USER_MATH_LIB_H
#define USER_MATH_LIB_H

#include <cmath>

void Conversion_Quaternion_to_Euler(float q[4], float* yaw, float* pitch, float* roll);
void Conversion_Euler_to_Quaternion(float q[4], float yaw_d, float pitch_d, float roll_d);


class user_math_lib
{
public:
    user_math_lib();
};

#endif // USER_MATH_LIB_H