#include "Headfile.h"
#include "IST8310.h"
#include "HMC5883.h"
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

#define MAG_REVERSE_SIDE
void IST8310_Init(void)
{
  Single_WriteI2C_Adjust(IST8310_SLAVE_ADDRESS,0x41,0x24);//����16x�ڲ�ƽ��
  Single_WriteI2C_Adjust(IST8310_SLAVE_ADDRESS,0x42,0xC0);//Set/Reset�ڲ�ƽ��
}



IST8310 Mag_IST8310;
void Get_Mag_IST8310(void)
{
  static uint16_t IST8310_Sample_Cnt=0;
  float MagTemp[3]={0};
  IST8310_Sample_Cnt++;
  if(IST8310_Sample_Cnt==1)
  {
  Single_WriteI2C_Adjust(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x01);//Single Measurement Mode
  }
  else if(IST8310_Sample_Cnt==4)//���ټ��6ms,�˴�Ϊ8ms
  {
    Mag_IST8310.Buf[0]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x03);//OUT_X_L_A
    Mag_IST8310.Buf[1]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x04);//OUT_X_H_A
    Mag_IST8310.Buf[2]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x05);//OUT_Y_L_A
    Mag_IST8310.Buf[3]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x06);//OUT_Y_H_A
    Mag_IST8310.Buf[4]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x07);//OUT_Z_L_A
    Mag_IST8310.Buf[5]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x08);//OUT_Z_H_A
    /*****************�ϳ��������������******************/
    Mag_IST8310.Mag_Data[0]=(Mag_IST8310.Buf[1]<<8)|Mag_IST8310.Buf[0];
    Mag_IST8310.Mag_Data[1]=(Mag_IST8310.Buf[3]<<8)|Mag_IST8310.Buf[2];
    Mag_IST8310.Mag_Data[2]=(Mag_IST8310.Buf[5]<<8)|Mag_IST8310.Buf[4];
    IST8310_Sample_Cnt=0;
  }
#ifdef MAG_REVERSE_SIDE//����ӳ���������������
   Mag_IST8310.x = Mag_IST8310.Mag_Data[0];
   Mag_IST8310.y = -Mag_IST8310.Mag_Data[1];
   Mag_IST8310.z = Mag_IST8310.Mag_Data[2];
#else
   Mag_IST8310.x = Mag_IST8310.Mag_Data[0];
   Mag_IST8310.y = Mag_IST8310.Mag_Data[1];
   Mag_IST8310.z = Mag_IST8310.Mag_Data[2];
#endif
   DataMag.x=Mag_IST8310.x;
   DataMag.y=Mag_IST8310.y;
   DataMag.z=Mag_IST8310.z;
   MagTemp[0]=GildeAverageValueFilter_MAG(Mag_IST8310.x-Mag_Offset[0],Data_X_MAG);//���������˲�
   MagTemp[1]=GildeAverageValueFilter_MAG(Mag_IST8310.y-Mag_Offset[1],Data_Y_MAG);
   MagTemp[2]=GildeAverageValueFilter_MAG(Mag_IST8310.z-Mag_Offset[2],Data_Z_MAG);



   Mag_Data[0]=Mag_IST8310.Mag_Data_Correct[0]=MagTemp[0];
   Mag_Data[1]=Mag_IST8310.Mag_Data_Correct[1]=MagTemp[1];
   Mag_Data[2]=Mag_IST8310.Mag_Data_Correct[2]=MagTemp[2];
   
   /************��������ǲ���*****************/
   MagN.x=Mag_IST8310.thx = MagTemp[0] * Cos_Roll+ MagTemp[2] * Sin_Roll;
   MagN.y=Mag_IST8310.thy = MagTemp[0] * Sin_Pitch*Sin_Roll
                    +MagTemp[1] * Cos_Pitch
                    -MagTemp[2] * Cos_Roll*Sin_Pitch;
   /***********�����еõ������ƹ۲�Ƕ�*********/
   Mag_IST8310.Angle_Mag=atan2(Mag_IST8310.thx,Mag_IST8310.thy)*57.296;
}


float Earth_Magnetic_Field_Intensity=0;
void Compass_Tradeoff(void)
{
  if(Extern_Mag_Work_Flag==1)//ֻҪ��ʼ��ʱ����⵽�ⲿ������HMC5883/5983�������ⲿ�������ںϣ�ע���ⲿ����������
  {
    HMC5883L_StateMachine();
    Earth_Magnetic_Field_Intensity=sqrt(Mag_Data[0]*Mag_Data[0]
                                        +Mag_Data[1]*Mag_Data[1]
                                        +Mag_Data[2]*Mag_Data[2])/MAG_GAIN_SCALE7;
  }
  else//ʹ���ڲ�������IST8310
  { 
    Get_Mag_IST8310();
    Earth_Magnetic_Field_Intensity=sqrt(Mag_Data[0]*Mag_Data[0]
                                        +Mag_Data[1]*Mag_Data[1]
                                        +Mag_Data[2]*Mag_Data[2])/330;
  }

}


