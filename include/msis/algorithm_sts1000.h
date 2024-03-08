#ifndef ALGORITHM_STS1000_H
#define ALGORITHM_STS1000_H

#define SCANPERANGLE 30    //每一度画30根线
#define SCANLINEMAX 7200 //7200条线填充

#define IMAGEWIDTH 1000
#define IMAGEHEIGHT 1000

#define IMAGESCANRANGE 500 //画图的半径

#define ORIGINALPX 500
#define ORIGINALPY 500

#define SCAN_STEP 1.8 // 360/200

#define POSE_X_RESOLUTION 0.02 //X轴的合理误差，小于该值，认为ROV没有动
#define POSE_Y_RESOLUTION 0.02 //Y轴的合理误差，小于该值，认为ROV没有动
#define POSE_Z_RESOLUTION 0.0 //Z轴的合理误差，小于该值，认为ROV没有动
#define POSE_ROLL_RESOLUTION 0.0 //ROLL的合理误差，小于该值，认为ROV没有动
#define POSE_PITCH_RESOLUTION 0.0 //PITCH的合理误差，小于该值，认为ROV没有动
#define POSE_YAW_RESOLUTION 1.0 //YAW的合理误差，小于该值，认为ROV没有动

typedef struct // 传感器姿态变化
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
}Pose_zyj;


#endif