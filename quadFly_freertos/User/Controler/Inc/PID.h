#ifndef _PID_H_
#define _PID_H_

#define PID_STARTADDR  0x0803A800//0x0803A800
#define PID1_Address  0
#define PID2_Address  12
#define PID3_Address  24
#define PID4_Address  36
#define PID5_Address  48
#define PID6_Address  60
#define PID7_Address  72
#define PID8_Address  84
#define PID9_Address  96
#define PID10_Address 108
#define PID11_Address 120


#include "Filter.h"
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
	*		�����ƴ���Դ�ɿ�   �人�Ƽ���ѧ  By.YuYi
	*		CSDN����: http://blog.csdn.net/u011992534
	*               �ſ�ID��NamelessCotrun����С��
	*               �����ƴ���Դ�ɿ�QQȺ��540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               �ٶ�����:�����ƴ���Դ�ɿ�
        *               �޸�����:2018/6/16
        *               �汾��V1.7.3.0
        *               ��Ȩ���У�����ؾ���
        *               Copyright(C) �人�Ƽ���ѧ�����ƴ��Ŷ� 2017-2019
        *               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
typedef struct
{
 float p;
 float i;
 float d;
}Vector3f_pid;


typedef struct
{
    uint8 Err_Limit_Flag :1;//ƫ���޷���־
    uint8 Integrate_Limit_Flag :1;//�����޷���־
    uint8 Integrate_Separation_Flag :1;//���ַ����־
    float Expect;//����
    float FeedBack;//����ֵ
    float Err;//ƫ��
    float Last_Err;//�ϴ�ƫ��
    float Err_Max;//ƫ���޷�ֵ
    float Integrate_Separation_Err;//���ַ���ƫ��ֵ
    float Integrate;//����ֵ
    float Integrate_Max;//�����޷�ֵ
    float Kp;//���Ʋ���Kp
    float Ki;//���Ʋ���Ki
    float Kd;//���Ʋ���Kd
    float Control_OutPut;//�����������
    float Last_Control_OutPut;//�ϴο����������
    float Control_OutPut_Limit;//����޷�
    /***************************************/
    float Pre_Last_Err;//���ϴ�ƫ��
    float Adaptable_Kd;//����Ӧ΢�ֲ���
    float Last_FeedBack;//�ϴη���ֵ
    float Dis_Err;//΢����
    float Dis_Error_History[5];//��ʷ΢����
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    float Last_Dis_Err_LPF;
    float Pre_Last_Dis_Err_LPF;
    Butter_BufferData Control_Device_LPF_Buffer;//��������ͨ�����������
    Testime PID_Controller_Dt;
}PID_Controler;

typedef struct
{
     PID_Controler Pitch_Angle_Control;
     PID_Controler Pitch_Gyro_Control;
     PID_Controler Roll_Angle_Control;
     PID_Controler Roll_Gyro_Control;
     PID_Controler Yaw_Angle_Control;
     PID_Controler Yaw_Gyro_Control;
     PID_Controler High_Position_Control;
     PID_Controler High_Speed_Control;
     PID_Controler Longitude_Position_Control;
     PID_Controler Longitude_Speed_Control;
     PID_Controler Latitude_Position_Control;
     PID_Controler Latitude_Speed_Control;
     /*************���ٶȿ��������¼�****************/
     PID_Controler High_Acce_Control;
     PID_Controler Longitude_Acce_Control;
     PID_Controler Latitude_Acce_Control;
}AllControler;

typedef enum
{
     Pitch_Angle_Controler=0,
     Pitch_Gyro_Controler=1,
     Roll_Angle_Controler=2,
     Roll_Gyro_Controler=3,
     Yaw_Angle_Controler=4,
     Yaw_Gyro_Controler=5,
     High_Position_Controler=6,
     High_Speed_Controler=7,
     Longitude_Position_Controler=8,
     Longitude_Speed_Controler=9,
     Latitude_Position_Controler=10,
     Latitude_Speed_Controler=11,
     High_Acce_Controler=12,
     Longitude_Acce_Controler=13,
     Latitude_Acce_Controler=14

}Controler_Label;

extern AllControler Total_Controller;


void PID_Paramter_Init_With_Flash(void);
void  Total_PID_Init(void);
void  PID_Init(PID_Controler *Controler,Controler_Label Label);
float PID_Control(PID_Controler *Controler);
float PID_Control_Yaw(PID_Controler *Controler);

float PID_Control_Div_LPF(PID_Controler *Controler);
float PID_Control_Err_LPF(PID_Controler *Controler);
float Differential_Forward_PID_Control_Div_LPF(PID_Controler *Controler);
float PID_Control_Div_LPF_For_Gyro(PID_Controler *Controler);

void  PID_Integrate_Reset(PID_Controler *Controler);
void  Take_Off_Reset(void);
void Throttle_Control_Reset(void);
void Save_Or_Reset_PID_Parameter(void);
extern uint8_t Sort_PID_Cnt,Sort_PID_Flag;
#endif


