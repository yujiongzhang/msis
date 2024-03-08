#include "user_math_lib.h"


user_math_lib::user_math_lib()
{

}

void Conversion_Quaternion_to_Euler(float q[], float *yaw, float *pitch, float *roll)
{



    *yaw = atan2f(2.0f*(q[1]*q[2] + q[0]*q[3]), 2.0f*(q[0]*q[0] + q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3] - q[0]*q[2]));
    *roll = atan2f(2.0f*(q[2]*q[3] + q[0]*q[1]), 2.0f*(q[0]*q[0] + q[3]*q[3])-1.0f);
}



void Conversion_Euler_to_Quaternion(float q[], float yaw_d, float pitch_d, float roll_d)
{
    //转换弧度
    float yaw = yaw_d * M_PI / 180.0f;
    float pitch = pitch_d * M_PI / 180.0f;
    float roll = roll_d * M_PI / 180.0f;

    float cy, sy, cp, sp, cr, sr;

    // Use ARM DSP Math library functions for sine and cosine
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}


