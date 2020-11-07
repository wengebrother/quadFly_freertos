#ifndef __IMU_H
#define __IMU_H
/*----------------------------------------------------------------------------------------------------------------------/
        *               ������ֻ��������ѧϰʹ�ã���Ȩ����Ȩ���������ƴ��Ŷӣ�
        *               �����ƴ��Ŷӽ��ɿس���Դ���ṩ�������ߣ�
        *               ������ҪΪ�����ƴ��Ŷ��ṩ������
        *               δ��������ɣ����ý�Դ�����ṩ������
        *               ���ý�Դ����ŵ����Ϲ�����������أ�
        *               �������Դ�����Ĳ�����緢��������Ϊ��
        *               �����ƴ��Ŷӽ���֮�Է��ɽ��������
-----------------------------------------------------------------------------------------------------------------------/
        *               ������Ϣ���ܶ���ֹ��ǰ�����������˳���������
        *               ��Դ���ף���ѧ����ϧ��ף������Ϯ�����׳ɹ�������
-----------------------------------------------------------------------------------------------------------------------/
	*		�����ƴ���Դ�ɿ� V1.1	�人�Ƽ���ѧ  By.YuYi
	*		CSDN����: http://blog.csdn.net/u011992534
	*               �ſ�ID��NamelessCotrun����С��
	*               �����ƴ���Դ�ɿ�QQȺ��540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               �ٶ�����:�����ƴ���Դ�ɿ�
        *               �޸�����:2017/10/30
        *               �汾��V1.1
        *               ��Ȩ���У�����ؾ���
        *               Copyright(C) �人�Ƽ���ѧ�����ƴ��Ŷ� 2017-2019
        *               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#define RtA         57.324841
#define AtR    	    0.0174533
#define Acc_G 	    0.0000610351
#define Gyro_G 	    0.0610351
#define Gyro_Gr	    0.0010653
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
#define GYRO_CALIBRATION_COFF 0.060976f  //2000
//#define GYRO_CALIBRATION_COFF 0.030488f  //1000
//#define GYRO_CALIBRATION_COFF 0.0152672f    //500

typedef struct
{
  float q[4];
  float angle[3];
}_Attitude_Tag;
extern _Attitude_Tag att;

extern float Yaw;
extern float Pitch;
extern float Roll;
extern float Yaw_Gyro,Pitch_Gyro,Roll_Gyro,Yaw_Gyro_Earth_Frame;
float constrain(float value, const float min_val, const float max_val);
void AHRSUpdate_GraDes_Delay_Corretion(float gx, float gy, float gz, float ax, float ay, float az);

void AHRSUpdate_GraDes_TimeSync(float gx, float gy, float gz, float ax, float ay, float az);
void Vector_From_BodyFrame2EarthFrame(Vector3f *bf,Vector3f *ef);
void Vector_From_EarthFrame2BodyFrame(Vector3f *ef,Vector3f *bf);
uint8_t Effective_Gravity_Acceleration(uint16_t num,float quality);


void NCQ_Quad_Init(void);
void DirectionConsineMatrix(Vector3f gyro,Vector3f acc,Vector2f magn);



extern  float rMat[3][3];
extern Vector3f DCM_Euler_Angle;
extern float Gyro_Length;
extern float Mag_Yaw;
extern float Gyro_Delta_Length;
extern float Test_DCM[2];
extern Vector3f Body_Motion_Acceleration;
extern Vector3f_Body Circle_Angle;
extern float gyro[3];
#endif
