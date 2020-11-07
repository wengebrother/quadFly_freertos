#include "Headfile.h"
#include "IST8310.h"
#include "HMC5883.h"
/*----------------------------------------------------------------------------------------------------------------------/
        *               本程序只供购买者学习使用，版权著作权属于无名科创团队，
        *               无名科创团队将飞控程序源码提供给购买者，
        *               购买者要为无名科创团队提供保护，
        *               未经作者许可，不得将源代码提供给他人
        *               不得将源代码放到网上供他人免费下载，
        *               更不能以此销售牟利，如发现上述行为，
        *               无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
        *               生命不息、奋斗不止；前人栽树，后人乘凉！！！
        *               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
-----------------------------------------------------------------------------------------------------------------------/
	*		无名科创开源飞控   武汉科技大学  By.YuYi
	*		CSDN博客: http://blog.csdn.net/u011992534
	*               优酷ID：NamelessCotrun无名小哥
	*               无名科创开源飞控QQ群：540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               百度贴吧:无名科创开源飞控
        *               修改日期:2018/6/16
        *               版本：V1.7.3.0
        *               版权所有，盗版必究。
        *               Copyright(C) 武汉科技大学无名科创团队 2017-2019
        *               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/

#define MAG_REVERSE_SIDE
void IST8310_Init(void)
{
  Single_WriteI2C_Adjust(IST8310_SLAVE_ADDRESS,0x41,0x24);//开启16x内部平均
  Single_WriteI2C_Adjust(IST8310_SLAVE_ADDRESS,0x42,0xC0);//Set/Reset内部平均
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
  else if(IST8310_Sample_Cnt==4)//至少间隔6ms,此处为8ms
  {
    Mag_IST8310.Buf[0]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x03);//OUT_X_L_A
    Mag_IST8310.Buf[1]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x04);//OUT_X_H_A
    Mag_IST8310.Buf[2]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x05);//OUT_Y_L_A
    Mag_IST8310.Buf[3]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x06);//OUT_Y_H_A
    Mag_IST8310.Buf[4]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x07);//OUT_Z_L_A
    Mag_IST8310.Buf[5]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x08);//OUT_Z_H_A
    /*****************合成三轴磁力计数据******************/
    Mag_IST8310.Mag_Data[0]=(Mag_IST8310.Buf[1]<<8)|Mag_IST8310.Buf[0];
    Mag_IST8310.Mag_Data[1]=(Mag_IST8310.Buf[3]<<8)|Mag_IST8310.Buf[2];
    Mag_IST8310.Mag_Data[2]=(Mag_IST8310.Buf[5]<<8)|Mag_IST8310.Buf[4];
    IST8310_Sample_Cnt=0;
  }
#ifdef MAG_REVERSE_SIDE//重新映射磁力计三轴数据
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
   MagTemp[0]=GildeAverageValueFilter_MAG(Mag_IST8310.x-Mag_Offset[0],Data_X_MAG);//滑动窗口滤波
   MagTemp[1]=GildeAverageValueFilter_MAG(Mag_IST8310.y-Mag_Offset[1],Data_Y_MAG);
   MagTemp[2]=GildeAverageValueFilter_MAG(Mag_IST8310.z-Mag_Offset[2],Data_Z_MAG);



   Mag_Data[0]=Mag_IST8310.Mag_Data_Correct[0]=MagTemp[0];
   Mag_Data[1]=Mag_IST8310.Mag_Data_Correct[1]=MagTemp[1];
   Mag_Data[2]=Mag_IST8310.Mag_Data_Correct[2]=MagTemp[2];
   
   /************磁力计倾角补偿*****************/
   MagN.x=Mag_IST8310.thx = MagTemp[0] * Cos_Roll+ MagTemp[2] * Sin_Roll;
   MagN.y=Mag_IST8310.thy = MagTemp[0] * Sin_Pitch*Sin_Roll
                    +MagTemp[1] * Cos_Pitch
                    -MagTemp[2] * Cos_Roll*Sin_Pitch;
   /***********反正切得到磁力计观测角度*********/
   Mag_IST8310.Angle_Mag=atan2(Mag_IST8310.thx,Mag_IST8310.thy)*57.296;
}


float Earth_Magnetic_Field_Intensity=0;
void Compass_Tradeoff(void)
{
  if(Extern_Mag_Work_Flag==1)//只要初始化时，检测到外部磁力计HMC5883/5983，就用外部磁力计融合，注意外部磁力计轴向
  {
    HMC5883L_StateMachine();
    Earth_Magnetic_Field_Intensity=sqrt(Mag_Data[0]*Mag_Data[0]
                                        +Mag_Data[1]*Mag_Data[1]
                                        +Mag_Data[2]*Mag_Data[2])/MAG_GAIN_SCALE7;
  }
  else//使用内部磁力计IST8310
  { 
    Get_Mag_IST8310();
    Earth_Magnetic_Field_Intensity=sqrt(Mag_Data[0]*Mag_Data[0]
                                        +Mag_Data[1]*Mag_Data[1]
                                        +Mag_Data[2]*Mag_Data[2])/330;
  }

}


