#ifndef _NAMELESSCOTRUN_OPTICALFLOW_
#define _NAMELESSCOTRUN_OPTICALFLOW_

void Optflow_Prase(void);
void Optflow_Statemachine(void);

typedef struct
{
int16_t x_pixel_vel;
int16_t y_pixel_vel;
int16_t x_integral; //间隔时间内，X  Axis累计像素位移
int16_t y_integral;//间隔时间内，Y  Axis累计像素位移
int16_t ground_distance;//距离
uint8_t quality;//图像质量
}OpticalFlow_Data;


typedef struct
{
 Vector2f Position;//位置估计量
 Vector2f Speed;//速度估计量
 Vector2f Acceleration;//加速度估计量
 Vector2f Last_Position;
 Vector2f Last_Speed;
 Vector2f Postion_Expect;
 Vector2f Speed_Expect;
 Vector2f Postion_Feedback;
 Vector2f Speed_Feedback;
 Vector2f Postion_Err;
 Vector2f Speed_Err;
 int16_t Distance;
}Optical_INS;



extern OpticalFlow_Data NamelessCotrun_OptFlow;

extern float OptFlow_Vel_X,OptFlow_Vel_Y;
extern float Last_OptFlow_Vel_X,Last_OptFlow_Vel_Y;
extern Optical_INS NamelessCotrunOptical;

#endif

