#include "Headfile.h"
#include "MPU6050.h"
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
float  Y_g_off = -100,X_g_off = 40,Z_g_off =0;
float  X_w_off =0,Y_w_off =0,Z_w_off =0;
float K[3]={1.0,1.0,1.0};//默认标度(量程)误差
float B[3]={0,0,0};//默认零位误差
//********************************************************
float  X_g,Y_g,Z_g;
float  X_w,Y_w,Z_w;
float  X_g_av,Y_g_av,Z_g_av;//可用的加速度计值
float  X_w_av,Y_w_av,Z_w_av;//可用的陀螺仪值
_IMU_Tag imu;
//200_30z
Butter_Parameter Accel_Parameter={
1,  -0.7477891782585,    0.272214937925,
0.1311064399166,   0.2622128798333,   0.1311064399166
};
Butter_Parameter Gyro_Parameter={
//200hz---51hz
1,  0.03680751639284,   0.1718123812701,
0.3021549744157,   0.6043099488315,   0.3021549744157
};
Butter_Parameter Butter_1HZ_Parameter_Acce={
  //200hz---1hz
  1,   -1.955578240315,   0.9565436765112,
  0.000241359049042, 0.000482718098084, 0.000241359049042
};

float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;

  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));

  return y;
}


//**************************************
//MPU6050
//**************************************
void InitMPU6050_GY86(void)
{
	Single_WriteI2C(0xD0,PWR_MGMT_1  , 0x00);//关闭所有中断,解除休眠
        Single_WriteI2C(0xD0,SMPLRT_DIV  , 0x00); // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
        //Single_WriteI2C(0xD0,MPU_CONFIG  , 0x02); //内部低通滤波频率，98hz
        Single_WriteI2C(0xD0,MPU_CONFIG  , 0x03); //内部低通滤波频率，44hz
        Single_WriteI2C(0xD0,GYRO_CONFIG , 0x10);//1000deg/s
        Single_WriteI2C(0xD0,ACCEL_CONFIG, 0x10);// Accel scale 8g (4096 LSB/g)
        //Single_WriteI2C(0xD0,ACCEL_CONFIG, 0x18);// Accel scale 16g (2048 LSB/g)
#ifdef IMU_BOARD_GY86
        Single_WriteI2C(0xD0,USER_CTRL, 0x00);//开启MPU6050主机模式、针对GY86模块
        Single_WriteI2C(0xD0,INT_PIN_CFG, 0x02);
#endif
        Set_Cutoff_Frequency(Sampling_Freq, 30,&Gyro_Parameter);//姿态角速度反馈滤波参数
        Set_Cutoff_Frequency(Sampling_Freq, 10,&Accel_Parameter);//姿态解算加计修正滤波值
        Set_Cutoff_Frequency(Sampling_Freq, 1,&Butter_1HZ_Parameter_Acce);//传感器校准加计滤波值
}


uint8_t Gyro_Range_Mode=0x00;
float Gyro_Range_Scale=0,Gyro_Range_Offset_Gain=2000;
//**************************************
int16_t GetData(uint8_t REG_Address)//获取陀螺仪原始数据
{
	uint8_t Hd,Ld;
	Hd=Single_ReadI2C(0xD0,REG_Address);
	Ld=Single_ReadI2C(0xD0,REG_Address+1);
	return (Hd<<8)+Ld;
}
//**************************************
//地推均值滤波
//**************************************
#define N 5
float Data_X_g[N];
float Data_Y_g[N];
float Data_Z_g[N];
float GildeAverageValueFilter(float NewValue,float *Data)
{
	float max,min;
	float sum;
	unsigned char i;
	Data[0]=NewValue;
	max=Data[0];
	min=Data[0];
	sum=Data[0];
	for(i=N-1;i!=0;i--)
	{
	  if(Data[i]>max) max=Data[i];
	  else if(Data[i]<min) min=Data[i];
	  sum+=Data[i];
	  Data[i]=Data[i-1];
	}
	 i=N-2;
	 sum=sum-max-min;
	 sum=sum/i;
	 return(sum);
}

/**********************************
函数名：void IMU_Calibration(void)
说明：MPU6050标定
入口：无
出口：无
备注：用来开机时设定陀螺仪的零值
**********************************/
void GET_GYRO(void)
{
	X_w  = GetData(GYRO_XOUT_H);
	Y_w  = GetData(GYRO_YOUT_H);
	Z_w  = GetData(GYRO_ZOUT_H);
}
s32 g_Gyro_xoffset = 0, g_Gyro_yoffset = 0, g_Gyro_zoffset = 0;
void IMU_Calibration(void)
{
	u8 i;
	for (i = 0; i < 100; i++)			//连续采样30次，一共耗时30*3=90ms
	{
		GET_GYRO();						//读取MPU6050的值
		g_Gyro_xoffset +=X_w;
		g_Gyro_yoffset +=Y_w;
		g_Gyro_zoffset +=Z_w;
		delay_ms(5);
	}
	X_w_off =(g_Gyro_xoffset/100);//得到标定偏移
	Y_w_off =(g_Gyro_yoffset/100);
	Z_w_off =(g_Gyro_zoffset/100);
}

void GET_ACC_DATA(void)
{
	X_g    = GetData(ACCEL_XOUT_H)-X_g_off;
	X_g_av = GildeAverageValueFilter(X_g,Data_X_g);
	Y_g    = GetData(ACCEL_YOUT_H)-Y_g_off;
	Y_g_av = GildeAverageValueFilter(Y_g,Data_Y_g);
	Z_g    = GetData(ACCEL_ZOUT_H)-Z_g_off;
	Z_g_av = GildeAverageValueFilter(Z_g,Data_Z_g);
}




Butter_BufferData Gyro_BufferData[3];
Butter_BufferData Accel_BufferData[3];
float MPU_LPF(float curr_inputer,
               Butter_BufferData *Buffer,
               Butter_Parameter *Parameter)
{
        /* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
        Buffer->Input_Butter[2]=curr_inputer;
	/* Butterworth滤波 */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) 序列保存 */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
        return (Buffer->Output_Butter[2]);
}


void GET_GYRO_DATA(void)//角速度低通滤波后用于姿态解算
{
	X_w  = GetData(GYRO_XOUT_H)-X_w_off;
	Y_w  = GetData(GYRO_YOUT_H)-Y_w_off;
	Z_w  = GetData(GYRO_ZOUT_H)-Z_w_off;
        X_w_av=MPU_LPF(X_w,
                        &Gyro_BufferData[0],
                        &Gyro_Parameter
                        );
        Y_w_av=MPU_LPF(Y_w,
                        &Gyro_BufferData[1],
                        &Gyro_Parameter
                        );
        Z_w_av=MPU_LPF(Z_w,
                        &Gyro_BufferData[2],
                        &Gyro_Parameter
                        );
}


void gyroDataFlilter(float *gyrofilter)//角速度低通滤波后用于姿态解算
{
	float X_err=0,Y_err=0,Z_err=0;
	X_err  = GetData(GYRO_XOUT_H)-X_w_off; //原始数据减去偏差
	Y_err  = GetData(GYRO_YOUT_H)-Y_w_off;
	Z_err  = GetData(GYRO_ZOUT_H)-Z_w_off;
        gyrofilter[0]=MPU_LPF(X_err,
                        &Gyro_BufferData[0],
                        &Gyro_Parameter
                        );
        gyrofilter[1]=MPU_LPF(Y_err,
                        &Gyro_BufferData[1],
                        &Gyro_Parameter
                        );
        gyrofilter[2]=MPU_LPF(Z_err,
                        &Gyro_BufferData[2],
                        &Gyro_Parameter
                        );
	
	
}


Vector3f DCM_Gyro,DCM_Acc;
void GET_MPU_DATA(void)
{
  //{Gyro_Range_Scale=0.007633f;Gyro_Range_Offset_Gain=250;}
  //{Gyro_Range_Scale=0.015267f;Gyro_Range_Offset_Gain=500;}
  {Gyro_Range_Scale=0.030487f;Gyro_Range_Offset_Gain=1000;}
  //{Gyro_Range_Scale=0.060975f;Gyro_Range_Offset_Gain=2000;}

  Accel_Filter();
  GET_GYRO_DATA();
  DCM_Acc.x=X_Origion;
  DCM_Acc.y=Y_Origion;
  DCM_Acc.z=Z_Origion;
  DCM_Gyro.x=X_w_av*GYRO_CALIBRATION_COFF*DEG2RAD;
  DCM_Gyro.y=Y_w_av*GYRO_CALIBRATION_COFF*DEG2RAD;
  DCM_Gyro.z=Z_w_av*GYRO_CALIBRATION_COFF*DEG2RAD;
}



float Gyro_X,Gyro_Y,Gyro_Z;
float Angle_X,Angle_Y,Angle_Z;
float ACCE_X,ACCE_Y,ACCE_Z;
void Angle_Calculate(void)//角度计算
{
    float ACCE_X_TEMP,ACCE_Y_TEMP,ACCE_Z_TEMP;
    ACCE_X_TEMP=ACCE_X=X_g_av;
    ACCE_Y_TEMP=ACCE_Y=Y_g_av;
    ACCE_Z_TEMP=ACCE_Z=Z_g_av;
    ACCE_Y=-57.3*atan(ACCE_X_TEMP*invSqrt(ACCE_Y_TEMP*ACCE_Y_TEMP+ACCE_Z_TEMP*ACCE_Z_TEMP));//横滚角
    ACCE_X=57.3*atan(ACCE_Y_TEMP*invSqrt(ACCE_X_TEMP*ACCE_X_TEMP+ACCE_Z_TEMP*ACCE_Z_TEMP));//俯仰角
}


int16_t Acce_Correct[3]={0};//用于矫正加速度量，截至频率很低
uint8_t Acce_Correct_Update_Flag=0;
Butter_BufferData Butter_Buffer_Correct[3];
void Acce_Correct_Filter()
{
   Acce_Correct[0]=Int_Sort(LPButterworth(imu.accelRaw[0],
                    &Butter_Buffer_Correct[0],&Butter_1HZ_Parameter_Acce));
   Acce_Correct[1]=Int_Sort(LPButterworth(imu.accelRaw[1]
                    ,&Butter_Buffer_Correct[1],&Butter_1HZ_Parameter_Acce));
   Acce_Correct[2]=Int_Sort(LPButterworth(imu.accelRaw[2]
                    ,&Butter_Buffer_Correct[2],&Butter_1HZ_Parameter_Acce));
   Acce_Correct_Update_Flag=1;
}


float X_Origion,Y_Origion,Z_Origion;
void Accel_Filter(void)
{
        imu.accelRaw[0] = GetData(ACCEL_XOUT_H);
        imu.accelRaw[1] = GetData(ACCEL_YOUT_H);
        imu.accelRaw[2] = GetData(ACCEL_ZOUT_H);
        Acce_Correct_Filter();
        X_Origion=K[0]*imu.accelRaw[0]-B[0]*One_G_TO_Accel;//经过椭球校正后的三轴加速度量
        Y_Origion=K[1]*imu.accelRaw[1]-B[1]*One_G_TO_Accel;
        Z_Origion=K[2]*imu.accelRaw[2]-B[2]*One_G_TO_Accel;
        FilterBefore_NamelessQuad.Acceleration[_YAW]=
                      -Sin_Roll* X_Origion
                        + Sin_Pitch *Cos_Roll *Y_Origion
                           + Cos_Pitch * Cos_Roll *Z_Origion;
        FilterBefore_NamelessQuad.Acceleration[_PITCH]=
                   Cos_Yaw* Cos_Roll * X_Origion
                        +(Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw) * Y_Origion
                          +(Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw) * Z_Origion;
        FilterBefore_NamelessQuad.Acceleration[_ROLL]=
                   Sin_Yaw* Cos_Roll * X_Origion
                        +(Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw) * Y_Origion
                          + (Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw) * Z_Origion;
        FilterBefore_NamelessQuad.Acceleration[_YAW]*=AcceGravity/AcceMax;
        FilterBefore_NamelessQuad.Acceleration[_YAW]-=AcceGravity;
        FilterBefore_NamelessQuad.Acceleration[_YAW]*=100;//加速度cm/s^2
        FilterBefore_NamelessQuad.Acceleration[_PITCH]*=AcceGravity/AcceMax;
        FilterBefore_NamelessQuad.Acceleration[_PITCH]*=100;//加速度cm/s^2
        FilterBefore_NamelessQuad.Acceleration[_ROLL]*=AcceGravity/AcceMax;
        FilterBefore_NamelessQuad.Acceleration[_ROLL]*=100;//加速度cm/s^2
        Acce_Control_Filter();//加速度滤波，用于惯导、加速度控制反馈量
	/* 加速度计Butterworth滤波 */
        X_g_av=MPU_LPF(X_Origion,&Accel_BufferData[0],&Accel_Parameter);
        Y_g_av=MPU_LPF(Y_Origion,&Accel_BufferData[1],&Accel_Parameter);
        Z_g_av=MPU_LPF(Z_Origion,&Accel_BufferData[2],&Accel_Parameter);
}
