#ifndef _NAMELESSCOTRUN_OPTICALFLOW_
#define _NAMELESSCOTRUN_OPTICALFLOW_

void Optflow_Prase(void);
void Optflow_Statemachine(void);

typedef struct
{
int16_t x_pixel_vel;
int16_t y_pixel_vel;
int16_t x_integral; //���ʱ���ڣ�X  Axis�ۼ�����λ��
int16_t y_integral;//���ʱ���ڣ�Y  Axis�ۼ�����λ��
int16_t ground_distance;//����
uint8_t quality;//ͼ������
}OpticalFlow_Data;


typedef struct
{
 Vector2f Position;//λ�ù�����
 Vector2f Speed;//�ٶȹ�����
 Vector2f Acceleration;//���ٶȹ�����
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

