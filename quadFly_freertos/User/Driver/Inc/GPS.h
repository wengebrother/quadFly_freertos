#ifndef __GPS_H
#define	__GPS_H/*----------------------------------------------------------------------------------------------------------------------/
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
#include<stdint.h>
#include<stdio.h>
#include<string.h>
#define EARTHR 6371004
//-----GNSS fixType-----//
typedef enum
{
  NO_FIX = 0x00,//�޶�λ
  DEAD_RECKONING_ONLY = 0x01,//���ߵ���λ
  FIX_2D = 0x02,//2ά��λ
  FIX_3D = 0x03,//3ά��λ
  GNSS_DEAD_RECKONING = 0x04,//GPS��ߵ���λ
  TIME_ONLY_FIX = 0x05//ֻ��ʱ�䶨λ
}GNSS_FixType_Def;


void GPS_PVT_Parse(void);
void Ublox_Set_Output_PVT_10hz_Baud_Set(unsigned long Baud);
void Set_GPS_USART(void);

extern double Last_Longitude,Last_Latitude;
extern int32_t Longitude_Origion,Latitude_Origion;
extern double Longitude,Latitude;

extern double Longitude_Deg,Latitude_Deg;
extern double Longitude_Min,Latitude_Min;
extern u16 TimeBeijing[3];
extern char TimeStr[8];
extern float GPS_Quality;
extern uint16 GPS_Sate_Num,Last_GPS_Sate_Num;
extern float GPS_Yaw;
extern float GPS_Ground_Speed;
extern float GPS_Speed_Resolve[2];
extern Vector3_Nav GPS_Vel;
extern Vector3_Nav GPS_Vel_Div;

extern float GPS_Pos_DOP;
extern uint8  GPS_Fix_Flag[4];

extern uint8 GPS_FixType;
extern float High_GPS;

#endif
