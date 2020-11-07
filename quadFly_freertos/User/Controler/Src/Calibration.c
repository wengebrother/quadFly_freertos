#include "Headfile.h"
#include "Calibration.h"
#include "CalibrationRoutines.h"
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
int16_t Mag_Offset[3]={0,0,0};
float Mag_Data[3]={0};
Vector2f MagN={0,0};
float HMC5883L_Yaw=0;
/***************加速度计6面矫正，参考APM代码，配合遥控器进行现场矫正**************************/
void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
    int16_t j,k;
    for( j=0; j<6; j++ )
    {
        dS[j] = 0.0f;
        for( k=0; k<6; k++ )
        {
            JS[j][k] = 0.0f;
        }
    }
}

void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6])
{
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int16_t i,j,k;
    float mu;
    //make upper triangular
    for( i=0; i<6; i++ ) {
        //eliminate all nonzero entries below JS[i][i]
        for( j=i+1; j<6; j++ ) {
            mu = JS[i][j]/JS[i][i];
            if( mu != 0.0f ) {
                dS[j] -= mu*dS[i];
                for( k=j; k<6; k++ ) {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }
    //back-substitute
    for( i=5; i>=0; i-- ) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;

        for( j=0; j<i; j++ ) {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }
    for( i=0; i<6; i++ ) {
        delta[i] = dS[i];
    }
}

void Calibrate_Update_Matrices(float dS[6],
                               float JS[6][6],
                               float beta[6],
                               float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];
    for(j=0;j<3;j++)
    {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }

    for(j=0;j<6;j++)
    {
        dS[j]+=jacobian[j]*residual;
        for(k=0;k<6;k++)
        {
            JS[j][k]+=jacobian[j]*jacobian[k];
        }
    }
}

uint8 Calibrate_accel(Acce_Unit accel_sample[6],
                      Acce_Unit *accel_offsets,
                      Acce_Unit *accel_scale)
{
    int16_t i;
    int16_t num_iterations = 0;
    float eps = 0.000000001;
    float change = 100.0;
    float data[3]={0};
    float beta[6]={0};
    float delta[6]={0};
    float ds[6]={0};
    float JS[6][6]={0};
    bool success = TRUE;
    // reset
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;
    while( num_iterations < 20 && change > eps ) {
        num_iterations++;
        Calibrate_Reset_Matrices(ds, JS);

        for( i=0; i<6; i++ ) {
            data[0] = accel_sample[i].x;
            data[1] = accel_sample[i].y;
            data[2] = accel_sample[i].z;
            Calibrate_Update_Matrices(ds, JS, beta, data);

        }
        Calibrate_Find_Delta(ds, JS, delta);
        change =    delta[0]*delta[0] +
                    delta[0]*delta[0] +
                    delta[1]*delta[1] +
                    delta[2]*delta[2] +
                    delta[3]*delta[3] / (beta[3]*beta[3]) +
                    delta[4]*delta[4] / (beta[4]*beta[4]) +
                    delta[5]*delta[5] / (beta[5]*beta[5]);
        for( i=0; i<6; i++ ) {
            beta[i] -= delta[i];
        }
    }
    // copy results out
    accel_scale->x = beta[3] * GRAVITY_MSS;
    accel_scale->y = beta[4] * GRAVITY_MSS;
    accel_scale->z = beta[5] * GRAVITY_MSS;
    accel_offsets->x = beta[0] * accel_scale->x;
    accel_offsets->y = beta[1] * accel_scale->y;
    accel_offsets->z = beta[2] * accel_scale->z;

    // sanity check scale
    if(fabsf(accel_scale->x-1.0f) > 0.5f
         || fabsf(accel_scale->y-1.0f) > 0.5f
           || fabsf(accel_scale->z-1.0f) > 0.5f )
    {
        success = FALSE;
    }
    // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
    if(fabsf(accel_offsets->x) > 5.0f
         || fabsf(accel_offsets->y) > 5.0f
           || fabsf(accel_offsets->z) > 5.0f )
    {
        success = FALSE;
    }
    // return success or failure
    return success;
}



float Aoco[6]={1,1,1};
float Aoc[6][6]={1,1,1};
Acce_Unit new_offset={
  0,0,0,
};
Acce_Unit new_scales={
  1.0,1.0,1.0,
};

Acce_Unit Accel_Offset_Read={
  0,0,0,
};
Acce_Unit Accel_Scale_Read={
  0,0,0,
};
uint8_t Cal_Flag=0;
void Calibrationer(void)
{
 uint16 i=0;
 Acce_Unit Test_Calibration[6]=
{
20,     21,    4152,
4062,  -24,      78,
-4082, 1,        -8,
-45,   -4071,   30,
20,    4035,    -8,
30,     -60,   -3980
};
 for(i=0;i<6;i++)
 {
    Test_Calibration[i].x *=ACCEL_TO_1G;
    Test_Calibration[i].y *=ACCEL_TO_1G;
    Test_Calibration[i].z *=ACCEL_TO_1G;
 }

 Cal_Flag=Calibrate_accel(Test_Calibration,
                 &new_offset,
                 &new_scales);
}


uint8_t flight_direction=6;
uint8_t Accel_Calibration_Flag=0;//加速度计校准模式
uint8_t Accel_Calibration_Finished[6]={0,0,0,0,0,0};//对应面校准完成标志位
uint8_t Accel_Calibration_All_Finished=0;//6面校准全部校准完成标志位
uint16_t Accel_Calibration_Makesure_Cnt=0;
uint16_t Accel_flight_direction_cnt=0;
void Accel_Calibration_Check(void)
{
   uint16_t  i=0;
   if(Throttle_Control==1000&&Yaw_Control>=Yaw_Max*Scale_Pecent_Max&&Roll_Control<=-30&&Pitch_Control>=30)
   {
      Accel_Calibration_Makesure_Cnt++;
   }
   if(Throttle_Control==1000
      &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
        &&Roll_Control<=-30
          &&Pitch_Control>=30
            &&Accel_Calibration_Makesure_Cnt>=200*3//持续三秒
              &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
  {
      Bling_Mode=1;
      Accel_Calibration_Flag=1;//加速度校准模式
      Cal_Flag=0;
      Bling_Set(&Light_1,1000,100,0.5,0,GPIOC,GPIO_Pin_4,1);
      Bling_Set(&Light_2,1000,100,0.5,0,GPIOC,GPIO_Pin_5,1);
      Bling_Set(&Light_3,1000,100,0.5,0,GPIOC,GPIO_Pin_10,1);
      flight_direction=6;
      Accel_Calibration_All_Finished=0;//全部校准完成标志位清零
      Accel_Calibration_Makesure_Cnt=0;
      for(i=0;i<6;i++)
      {
        Accel_Calibration_Finished[i]=0;//对应面标志位清零
        acce_sample[i].x=0; //清空对应面的加速度计量
        acce_sample[i].y=0; //清空对应面的加速度计量
        acce_sample[i].z=0; //清空对应面的加速度计量
      }
      Page_Number=10;//OLED加速度计矫正页面
      Reset_Mag_Calibartion(1);
      Reset_RC_Calibartion(1);
      Forced_Lock_Makesure_Cnt=0;
  }

  if(Accel_Calibration_Flag==1)
  {
     if(Throttle_Control==1000&&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max&&Roll_Control==0&&Pitch_Control==0)
     {
       Accel_flight_direction_cnt++;
       if(Accel_flight_direction_cnt>=5*20)//100ms
       {
         flight_direction=0;
          Unlock_Makesure_Cnt=0;
          Lock_Makesure_Cnt=0;
       }

     }
     else if(Throttle_Control==1000&&Yaw_Control==0&&Roll_Control>=30&&Pitch_Control==0)
     {
       Accel_flight_direction_cnt++;
       if(Accel_flight_direction_cnt>=5*20)//100ms
       {
         flight_direction=1;
         Unlock_Makesure_Cnt=0;
         Lock_Makesure_Cnt=0;
       }
     }
     else if(Throttle_Control==1000&&Yaw_Control==0&&Roll_Control<=-30&&Pitch_Control==0)
     {
       Accel_flight_direction_cnt++;
       if(Accel_flight_direction_cnt>=5*20)//100ms
       {
         flight_direction=2;
         Unlock_Makesure_Cnt=0;
         Lock_Makesure_Cnt=0;
       }
     }
     else if(Throttle_Control==1000&&Yaw_Control==0&&Roll_Control==0&&Pitch_Control>=30)
     {
       Accel_flight_direction_cnt++;
       if(Accel_flight_direction_cnt>=5*20)//100ms
       {
         flight_direction=3;
         Unlock_Makesure_Cnt=0;
         Lock_Makesure_Cnt=0;
       }
     }
     else if(Throttle_Control==1000&&Yaw_Control==0&&Roll_Control==0&&Pitch_Control<=-30)
     {
       Accel_flight_direction_cnt++;
       if(Accel_flight_direction_cnt>=5*20)//100ms
       {
         flight_direction=4;
         Unlock_Makesure_Cnt=0;
         Lock_Makesure_Cnt=0;
       }
     }
     else if(Throttle_Control==1000&&Yaw_Control>Yaw_Max*Scale_Pecent_Max&&Roll_Control==0&&Pitch_Control==0)
     {
       Accel_flight_direction_cnt++;
       if(Accel_flight_direction_cnt>=5*20)//100ms
       {
         flight_direction=5;
         Unlock_Makesure_Cnt=0;
         Lock_Makesure_Cnt=0;
       }
     }
     else
     {
       Accel_flight_direction_cnt/=2;
     }

   if(Accel_flight_direction_cnt>=200)  Accel_flight_direction_cnt=0;

 }

}

Acce_Unit acce_sample[6]={0};//三行6列，保存6面待矫正数据
uint8_t Flash_Buf[12]={0};
/***************************************************
函数名: void Accel_Calibartion()
说明:	加速度基本标定、利用遥控器直接进入
入口:	无
出口:	无
备注:	在主函数While(1)里面，利用中断空隙时间一直运行
****************************************************/
uint8_t Accel_Calibartion(void)
{
  uint16 i,j=0;
  float acce_sample_sum[3]={0,0,0};//加速度和数据
/*第一面飞控平放，Z轴正向朝着正上方，Z axis is about 1g,X、Y is about 0g*/
/*第二面飞控平放，X轴正向朝着正上方，X axis is about 1g,Y、Z is about 0g*/
/*第三面飞控平放，X轴正向朝着正下方，X axis is about -1g,Y、Z is about 0g*/
/*第四面飞控平放，Y轴正向朝着正下方，Y axis is about -1g,X、Z is about 0g*/
/*第五面飞控平放，Y轴正向朝着正上方，Y axis is about 1g,X、Z is about 0g*/
/*第六面飞控平放，Z轴正向朝着正下方，Z axis is about -1g,X、Y is about 0g*/
if(flight_direction<=5)//检测到对应面数据
{
  uint16_t num_samples=0;
  while(num_samples<1000)//采样200次
  {
    if(Gyro_Length<=20.0f
       &&Acce_Correct_Update_Flag==1)//通过陀螺仪模长来确保机体静止
    {
       for(j=0;j<3;j++){
          acce_sample_sum[j]+=Acce_Correct[j]*ACCEL_TO_1G;//加速度计转化为1g量程下
       }
       //delay_ms(4);//间隔10ms，1s内数据取平均
       num_samples++;
       Acce_Correct_Update_Flag=0;
    }
    Accel_Calibration_Finished[flight_direction]=1;//对应面校准完成标志位置1
  }
  acce_sample[flight_direction].x=acce_sample_sum[0]/num_samples; //保存对应面的加速度计量
  acce_sample[flight_direction].y=acce_sample_sum[1]/num_samples; //保存对应面的加速度计量
  acce_sample[flight_direction].z=acce_sample_sum[2]/num_samples; //保存对应面的加速度计量
  flight_direction=6;//单面矫正完毕
}

  if((Accel_Calibration_Finished[0]
    &Accel_Calibration_Finished[1]
     &Accel_Calibration_Finished[2]
       &Accel_Calibration_Finished[3]
         &Accel_Calibration_Finished[4]
           &Accel_Calibration_Finished[5])
             &&Accel_Calibration_All_Finished==0)//6面全部校准完毕
  {
      Accel_Calibration_All_Finished=1;//加速度计6面校准完成标志
      Accel_Calibration_Flag=0;//加速度计校准结束，释放遥感操作
      Cal_Flag=Calibrate_accel(acce_sample,
                                &new_offset,
                                  &new_scales);//将所得6面数据
      for(i=0;i<6;i++)
      {
        Accel_Calibration_Finished[i]=0;//对应面标志位清零
      }
     if(Cal_Flag==TRUE)//加速度计校准成功
     {
       WriteFlashNineFloat(Accel_Offset_Address,
                        new_offset.x,
                        new_offset.y,
                        new_offset.z,
                        new_scales.x,
                        new_scales.y,
                        new_scales.z,
                        Mag_Offset_Read.x,
                        Mag_Offset_Read.y,
                        Mag_Offset_Read.z);//写入加速度零点偏执与磁力计中心偏执
       Parameter_Init();//读取写入参数
       Bling_Mode=0;//恢复正常指示模式
       Bling_Set(&Light_1,3000,1000,0.3,0,GPIOC,GPIO_Pin_4,0);
       Bling_Set(&Light_2,3000,1000,0.5,0,GPIOC,GPIO_Pin_5,0);
       Bling_Set(&Light_3,3000,1000,0.8,0,GPIOC,GPIO_Pin_10,0);
     }
     else//加速度计校准失败
     {
        Bling_Mode=0;//恢复正常指示模式
        Bling_Set(&Light_1,5000,200,0.3,0,GPIOC,GPIO_Pin_4,0);
        Bling_Set(&Light_2,5000,200,0.5,0,GPIOC,GPIO_Pin_5,0);
        Bling_Set(&Light_3,5000,200,0.8,0,GPIOC,GPIO_Pin_10,0);
        Page_Number=0;//OLED恢复首页
     }
     return TRUE;
  }
  return FALSE;
}



void Reset_Accel_Calibartion(uint8_t Type)
{
  uint16 i=0;
  for(i=0;i<6;i++)
  {
     Accel_Calibration_Finished[i]=0;//对应面标志位清零
     acce_sample[i].x=0; //清空对应面的加速度计量
     acce_sample[i].y=0; //清空对应面的加速度计量
     acce_sample[i].z=0; //清空对应面的加速度计量
  }
  Accel_Calibration_All_Finished=0;//全部校准完成标志位清零
  if(Type==1)  Accel_Calibration_Flag=0;
}


typedef struct
{
uint8_t accel_off;
uint8_t accel_scale;
uint8_t mag;
}Parameter_Flag;

Parameter_Flag Parameter_Read_Flag;

bool Parameter_Init(void)
{
    bool success=TRUE;
   /************加速度计零偏与标度值*******/
    Parameter_Read_Flag.accel_off=ReadFlashThreeFloat(Accel_Offset_Address,
                         &Accel_Offset_Read.x,
                         &Accel_Offset_Read.y,
                         &Accel_Offset_Read.z);

    Parameter_Read_Flag.accel_scale=ReadFlashThreeFloat(Accel_Scale_Address,
                         &Accel_Scale_Read.x,
                         &Accel_Scale_Read.y,
                         &Accel_Scale_Read.z);
    /************磁力计零偏****************/
    Parameter_Read_Flag.mag=ReadFlashThreeFloat(Mag_Offset_Address,
                         &Mag_Offset_Read.x,
                         &Mag_Offset_Read.y,
                         &Mag_Offset_Read.z);
    // sanity check scale
    if(ABS(Accel_Scale_Read.x-1.0f)>0.5
         || ABS(Accel_Scale_Read.y-1.0f)>0.5f
           || ABS(Accel_Scale_Read.z-1.0f)>0.5f)
    {
        success = FALSE;
    }
    // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
    if(ABS(Accel_Offset_Read.x) > 5.0f
         || ABS(Accel_Offset_Read.y) > 5.0f
           || ABS(Accel_Offset_Read.z) > 5.0f)
    {
        success = FALSE;
    }


   if(success==TRUE
      &&Parameter_Read_Flag.accel_off!=0x07
       &&Parameter_Read_Flag.accel_scale!=0x07)//Flash内数据正常，更新加速度校正值
   {
    B[0]=Accel_Offset_Read.x;//*One_G_TO_Accel;
    B[1]=Accel_Offset_Read.y;//*One_G_TO_Accel;
    B[2]=Accel_Offset_Read.z;//*One_G_TO_Accel;
    K[0]=Accel_Scale_Read.x;
    K[1]=Accel_Scale_Read.y;
    K[2]=Accel_Scale_Read.z;
   }
   /**********磁力计中心偏执获取************/
   if(Parameter_Read_Flag.mag!=0x07)
   {
   Mag_Offset[0]=(int16_t)(Mag_Offset_Read.x);
   Mag_Offset[1]=(int16_t)(Mag_Offset_Read.y);
   Mag_Offset[2]=(int16_t)(Mag_Offset_Read.z);
   }
   else
   {
   Mag_Offset[0]=0;
   Mag_Offset[1]=0;
   Mag_Offset[2]=0;
   }
   return success;
}
/************加速度计6面矫正结束***********************/


/***********磁力计中心矫正，取单轴最大、最小值平均******/
uint8_t Mag_Calibration_Flag=0,Mag_Calibration_All_Finished;
uint8_t Mag_Calibration_Finished[3]={0};
uint16_t Mag_Calibration_Makesure_Cnt=0;
uint8_t  Mag_Calibration_Mode=3;
uint16_t Mag_Calibration_Cnt=0;
float Yaw_Correct=0;
/*********************************************/
const int16_t Mag_360_define[36]={
0,10,20,30,40,50,60,70,80,90,
100,110,120,130,140,150,160,170,180,190,
200,210,220,230,240,250,260,270,280,290,
300,310,320,330,340,350
};//磁力计矫正遍历角度，确保数据采集充分
uint8_t Last_Mag_360_Flag[3][36]={0};
uint8_t Mag_360_Flag[3][36]={0};
uint16_t Mag_Is_Okay_Flag[3];
Calibration Mag;
Mag_Unit DataMag;
Mag_Unit Mag_Offset_Read={
  0,0,0,
};
void Mag_Calibration_Check(void)
{
   uint16_t  i=0,j=0;
   if(Throttle_Control==1000
      &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
        &&Roll_Control>=30
          &&Pitch_Control>=30)
     Mag_Calibration_Makesure_Cnt++;

   if(Throttle_Control==1000
      &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
        &&Roll_Control>=30
          &&Pitch_Control>=30
           &&Mag_Calibration_Makesure_Cnt>200*5//持续5S
            &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
           //进入磁力计校准模式
  {
      Bling_Mode=2;
      Mag_Calibration_Flag=1;//磁力计校准模式
      Mag_Calibration_Mode=3;
      Bling_Set(&Light_1,1000,500,0.2,0,GPIOC,GPIO_Pin_4,1);
      Bling_Set(&Light_2,1000,500,0.5,0,GPIOC,GPIO_Pin_5,1);
      Bling_Set(&Light_3,1000,500,0.7,0,GPIOC,GPIO_Pin_10,1);
      Mag_Calibration_Makesure_Cnt=0;
      Mag_Calibration_All_Finished=0;//全部校准完成标志位清零
      for(i=0;i<3;i++)
      {
        Mag_Calibration_Finished[i]=0;//对应面标志位清零
        for(j=0;j<36;j++) {Mag_360_Flag[i][j]=0;}
      }
      Page_Number=11;
      Reset_Accel_Calibartion(1);
      Reset_RC_Calibartion(1);
      Forced_Lock_Makesure_Cnt=0;
  }

  if(Mag_Calibration_Flag==1)
  {
     if(Throttle_Control==1000
        &&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max
          &&Roll_Control==0
            &&Pitch_Control==0) //第一面矫正
     {
         Mag_Calibration_Cnt++;
         if(Mag_Calibration_Cnt>=5*20)
         {
            Mag_Calibration_Mode=0;
            Mag_Is_Okay_Flag[0]=0;//单面数据采集完成标志位置0
            Mag_Is_Okay_Flag[1]=0;//单面数据采集完成标志位置0
            Mag_Is_Okay_Flag[2]=0;//单面数据采集完成标志位置0
            for(i=0;i<36;i++) Mag_360_Flag[0][i]=0;//清空采集角遍历数据点
            for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角遍历数据点
            for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角遍历数据点
            LS_Init(&Mag_LS);//清空拟合中间变量
            Unlock_Makesure_Cnt=0;
            Lock_Makesure_Cnt=0;
	 }

     }
  else if(Throttle_Control==1000
             &&Yaw_Control>Yaw_Max*Scale_Pecent_Max
               &&Roll_Control==0
                 &&Pitch_Control==0) //第二面矫正
     {
         Mag_Calibration_Cnt++;
         if(Mag_Calibration_Cnt>=5*20)
         {
             Mag_Calibration_Mode=1;
             Mag_Is_Okay_Flag[0]=0;//单面数据采集完成标志位置0
             Mag_Is_Okay_Flag[1]=0;//单面数据采集完成标志位置0
             Mag_Is_Okay_Flag[2]=0;//单面数据采集完成标志位置0
             for(i=0;i<36;i++) Mag_360_Flag[0][i]=0;//清空采集角遍历数据点
             for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角遍历数据点
             for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角遍历数据点
             LS_Init(&Mag_LS);//清空拟合中间变量
             Unlock_Makesure_Cnt=0;
             Lock_Makesure_Cnt=0;
         }
     }
  else
  {
    Mag_Calibration_Cnt/=2;
  }
  if(Mag_Calibration_Cnt>=200)  Mag_Calibration_Cnt=200;

  }

}

void Reset_Mag_Calibartion(uint8_t Type)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Mag_360_Flag[0][i]=0;//清空采集角点
    Mag_360_Flag[1][i]=0;//清空采集角点
    Mag_360_Flag[2][i]=0;//清空采集角点
  }
  Mag_Is_Okay_Flag[0]=0;
  Mag_Is_Okay_Flag[1]=0;
  Mag_Is_Okay_Flag[2]=0;
  Mag_Calibration_Mode=3;
  if(Type==1)  Mag_Calibration_Flag=0;
}


uint8_t Check_Plane_Sampling_Okay(uint8_t plane_number)
{
  uint8_t finished_flag=0;
  if(Mag_360_Flag[plane_number][0]&Mag_360_Flag[plane_number][1]&Mag_360_Flag[plane_number][2]
    &Mag_360_Flag[plane_number][3]&Mag_360_Flag[plane_number][4]&Mag_360_Flag[plane_number][5]
     &Mag_360_Flag[plane_number][6]&Mag_360_Flag[plane_number][7]&Mag_360_Flag[plane_number][8]
      &Mag_360_Flag[plane_number][9]&Mag_360_Flag[plane_number][10]&Mag_360_Flag[plane_number][11]
        &Mag_360_Flag[plane_number][12]&Mag_360_Flag[plane_number][13]&Mag_360_Flag[plane_number][14]
         &Mag_360_Flag[plane_number][15]&Mag_360_Flag[plane_number][16]&Mag_360_Flag[plane_number][17]
          &Mag_360_Flag[plane_number][18]&Mag_360_Flag[plane_number][19]&Mag_360_Flag[plane_number][20]
           &Mag_360_Flag[plane_number][21]&Mag_360_Flag[plane_number][22]&Mag_360_Flag[plane_number][23]
            &Mag_360_Flag[plane_number][24]&Mag_360_Flag[plane_number][25]&Mag_360_Flag[plane_number][26]
             &Mag_360_Flag[plane_number][27]&Mag_360_Flag[plane_number][28]&Mag_360_Flag[plane_number][29]
              &Mag_360_Flag[plane_number][30]&Mag_360_Flag[plane_number][31]&Mag_360_Flag[plane_number][32]
               &Mag_360_Flag[plane_number][33]&Mag_360_Flag[plane_number][34]&Mag_360_Flag[plane_number][35])
 finished_flag=1;
 return finished_flag;
}
/***************************************************
函数名: Mag_Calibartion(Mag_Unit MagData,Vector3f_Body Circle_Angle_Calibartion)
说明:	磁力计中心标定、利用遥控器直接进入
入口:	陀螺仪积分角度值、三轴磁力计原始值
出口:	无
备注:	在主函数While(1)里面，利用中断空隙时间一直运行
****************************************************/
uint8_t Mag_Calibartion(Mag_Unit MagData,Vector3f_Body Circle_Angle_Calibartion)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Last_Mag_360_Flag[0][i]=Mag_360_Flag[0][i];
    Last_Mag_360_Flag[1][i]=Mag_360_Flag[1][i];
    Last_Mag_360_Flag[2][i]=Mag_360_Flag[2][i];
  }
/********第一面Z轴正向朝着正上方，
  开始绕竖直轴旋转，Z axis is about 1g,X、Y is about 0g*/
/********第二面Y轴正向朝着正上方，
  开始绕竖直轴旋转，Y axis is about 1g,X、Z is about 0g*/
if(Mag_Calibration_Mode<3)//检测到对应面数据
{
  for(i=0;i<36;i++)
  {
    if(ABS(Circle_Angle_Calibartion.Yaw-Mag_360_define[i])<=5.0
         &&Mag_Calibration_Mode==0
          &&Acce_Correct[2]>=AcceMax_1G/2)//Z轴基本竖直
    {
      Mag_360_Flag[0][i]=1;
    }

    if(ABS(Circle_Angle_Calibartion.Rol-Mag_360_define[i])<=5.0
         &&Mag_Calibration_Mode==1
          &&Acce_Correct[1]>=AcceMax_1G/2)//Y轴基本竖直
    {
      Mag_360_Flag[1][i]=1;
    }

    if(ABS(Circle_Angle_Calibartion.Pit-Mag_360_define[i])<=5.0
         &&Mag_Calibration_Mode==2
          &&Acce_Correct[0]>=AcceMax_1G/2)//X轴基本竖直
    {
      Mag_360_Flag[2][i]=1;
    }
  }
  if(MagData.x >= Mag.x_max)   Mag.x_max = (int16_t)(MagData.x);
  if(MagData.x <  Mag.x_min)   Mag.x_min = (int16_t)(MagData.x);
  if(MagData.y >= Mag.y_max)   Mag.y_max = (int16_t)(MagData.y);
  if(MagData.y <  Mag.y_min)   Mag.y_min = (int16_t)(MagData.y);
  if(MagData.z >= Mag.z_max)   Mag.z_max = (int16_t)(MagData.z);
  if(MagData.z <  Mag.z_min)   Mag.z_min = (int16_t)(MagData.z);
}
if(Check_Plane_Sampling_Okay(0))
{
  Mag_Is_Okay_Flag[0]=1;//单面数据采集完成标志位置1
  for(i=0;i<36;i++)  Mag_360_Flag[0][i]=0;//清空采集角点
  if(Mag_Is_Okay_Flag[1]==0)//另外一面数据未完成
    Mag_Calibration_Mode=1;//自动进入下一面数据采集模式
  else Mag_Calibration_Mode=3;//
}

if(Check_Plane_Sampling_Okay(1))
{
  Mag_Is_Okay_Flag[1]=1;//单面数据采集完成标志位置1
  for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角点
  if(Mag_Is_Okay_Flag[2]==0)//另外一面数据未完成
    Mag_Calibration_Mode=2;//自动进入下一面数据采集模式
  else Mag_Calibration_Mode=3;
}

if(Check_Plane_Sampling_Okay(2))
{
  Mag_Is_Okay_Flag[2]=1;//单面数据采集完成标志位置1
  for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角点
  if(Mag_Is_Okay_Flag[0]==0)//另外一面数据未完成
    Mag_Calibration_Mode=0;//自动进入下一面数据采集模式
  else Mag_Calibration_Mode=3;
}



  if(Mag_Calibration_Mode==0)  Yaw_Correct=Circle_Angle_Calibartion.Yaw;
  else if(Mag_Calibration_Mode==1)  Yaw_Correct=Circle_Angle_Calibartion.Rol;
  else if(Mag_Calibration_Mode==2)  Yaw_Correct=Circle_Angle_Calibartion.Pit;
  else Yaw_Correct=0;



  if(Mag_Is_Okay_Flag[0]==1
     &&Mag_Is_Okay_Flag[1]==1
       &&Mag_Is_Okay_Flag[2]==1)//三面数据全部采集完毕，计算磁力计零点
{
      Mag.x_offset=(Mag.x_min+Mag.x_max)/2.0;
      Mag.y_offset=(Mag.y_min+Mag.y_max)/2.0;
      Mag.z_offset=(Mag.z_min+Mag.z_max)/2.0;
      Mag_Offset_Read.x=Mag.x_offset;
      Mag_Offset_Read.y=Mag.y_offset;
      Mag_Offset_Read.z=Mag.z_offset;
      Mag_Is_Okay_Flag[0]=0;
      Mag_Is_Okay_Flag[1]=0;
      Mag_Is_Okay_Flag[2]=0;
      Mag_Calibration_Flag=0;//磁力计校准结束，释放遥感操作
      Bling_Mode=0;//恢复正常指示模式

      Mag_Offset[0]=(int16_t)(Mag_Offset_Read.x);
      Mag_Offset[1]=(int16_t)(Mag_Offset_Read.y);
      Mag_Offset[2]=(int16_t)(Mag_Offset_Read.z);

      Bling_Set(&Light_1,5000,1000,0.3,0,GPIOC,GPIO_Pin_4,0);
      Bling_Set(&Light_2,5000,1000,0.5,0,GPIOC,GPIO_Pin_5,0);
      Bling_Set(&Light_3,5000,1000,0.8,0,GPIOC,GPIO_Pin_10,0);
      WriteFlashNineFloat(Accel_Offset_Address,
                        Accel_Offset_Read.x,
                        Accel_Offset_Read.y,
                        Accel_Offset_Read.z,
                        Accel_Scale_Read.x,
                        Accel_Scale_Read.y,
                        Accel_Scale_Read.z,
                        Mag.x_offset,
                        Mag.y_offset,
                        Mag.z_offset);//写入加速度零点偏执与磁力计中心偏执
      return TRUE;
  }
return FALSE;
}


void Mag_LS_Init()
{
    LS_Init(&Mag_LS);
}
float mag_a,mag_b,mag_c,mag_r;
/***************************************************
函数名: Mag_Calibartion(Mag_Unit MagData,Vector3f_Body Circle_Angle_Calibartion)
说明:	磁力计最小二乘法拟合球面、利用遥控器直接进入
入口:	陀螺仪积分角度值、三轴磁力计原始值
出口:	无
备注:	在主函数While(1)里面，利用中断空隙时间一直运行
****************************************************/
uint8_t Mag_Calibartion_LS(Mag_Unit MagData,Vector3f_Body Circle_Angle_Calibartion)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Last_Mag_360_Flag[0][i]=Mag_360_Flag[0][i];
    Last_Mag_360_Flag[1][i]=Mag_360_Flag[1][i];
    Last_Mag_360_Flag[2][i]=Mag_360_Flag[2][i];
  }

/********第一面Z轴正向朝着正上方，
  开始绕竖直轴旋转，Z axis is about 1g,X、Y is about 0g*/
/********第二面Y轴正向朝着正上方，
  开始绕竖直轴旋转，Y axis is about 1g,X、Z is about 0g*/
if(Mag_Calibration_Mode<3)//检测到对应面数据
{
  for(i=0;i<36;i++)
  {
        if(ABS(Circle_Angle_Calibartion.Yaw-Mag_360_define[i])<=5.0
         &&Mag_Calibration_Mode==0
          &&Acce_Correct[2]>=AcceMax_1G/2)//Z轴基本竖直
    {
      Mag_360_Flag[0][i]=1;
    }

    if(ABS(Circle_Angle_Calibartion.Rol-Mag_360_define[i])<=5.0
         &&Mag_Calibration_Mode==1
          &&Acce_Correct[1]>=AcceMax_1G/2)//Y轴基本竖直
    {
      Mag_360_Flag[1][i]=1;
    }

    if(ABS(Circle_Angle_Calibartion.Pit-Mag_360_define[i])<=5.0
         &&Mag_Calibration_Mode==2
          &&Acce_Correct[0]>=AcceMax_1G/2)//X轴基本竖直
    {
      Mag_360_Flag[2][i]=1;
    }
  }

    for(i=0;i<36;i++)
  {
    if((Last_Mag_360_Flag[0][i]==0&&Mag_360_Flag[0][i]==1)
      ||(Last_Mag_360_Flag[1][i]==0&&Mag_360_Flag[1][i]==1)
      ||(Last_Mag_360_Flag[2][i]==0&&Mag_360_Flag[2][i]==1))
    {
       LS_Accumulate(&Mag_LS, MagData.x,MagData.y,MagData.z);
       LS_Calculate(&Mag_LS,36*3,0.0f,&mag_a, &mag_b, &mag_c,&mag_r);
    }
  }
}

if(Check_Plane_Sampling_Okay(0))
{
  Mag_Is_Okay_Flag[0]=1;//单面数据采集完成标志位置1
  for(i=0;i<36;i++)  Mag_360_Flag[0][i]=0;//清空采集角点
  if(Mag_Is_Okay_Flag[1]==0)//另外一面数据未完成
    Mag_Calibration_Mode=1;//自动进入下一面数据采集模式
  else Mag_Calibration_Mode=3;//
}

if(Check_Plane_Sampling_Okay(1))
{
  Mag_Is_Okay_Flag[1]=1;//单面数据采集完成标志位置1
  for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角点
  if(Mag_Is_Okay_Flag[2]==0)//另外一面数据未完成
    Mag_Calibration_Mode=2;//自动进入下一面数据采集模式
  else Mag_Calibration_Mode=3;
}

if(Check_Plane_Sampling_Okay(2))
{
  Mag_Is_Okay_Flag[2]=1;//单面数据采集完成标志位置1
  for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角点
  if(Mag_Is_Okay_Flag[0]==0)//另外一面数据未完成
    Mag_Calibration_Mode=0;//自动进入下一面数据采集模式
  else Mag_Calibration_Mode=3;
}

  if(Mag_Calibration_Mode==0)  Yaw_Correct=Circle_Angle_Calibartion.Yaw;
  else if(Mag_Calibration_Mode==1)  Yaw_Correct=Circle_Angle_Calibartion.Rol;
  else if(Mag_Calibration_Mode==2)  Yaw_Correct=Circle_Angle_Calibartion.Pit;
  else Yaw_Correct=0;

  if(Mag_Is_Okay_Flag[0]==1
     &&Mag_Is_Okay_Flag[1]==1
       &&Mag_Is_Okay_Flag[2]==1)//三面数据全部采集完毕，计算磁力计零点
  {
      Mag_Offset_Read.x=mag_a;
      Mag_Offset_Read.y=mag_b;
      Mag_Offset_Read.z=mag_c;
      Mag_Is_Okay_Flag[0]=0;
      Mag_Is_Okay_Flag[1]=0;
      Mag_Is_Okay_Flag[2]=0;
      Mag_Calibration_Flag=0;//磁力计校准结束，释放遥感操作
      Bling_Mode=0;//恢复正常指示模式

      Mag_Offset[0]=(int16_t)(Mag_Offset_Read.x);
      Mag_Offset[1]=(int16_t)(Mag_Offset_Read.y);
      Mag_Offset[2]=(int16_t)(Mag_Offset_Read.z);

      Bling_Set(&Light_1,5000,1000,0.3,0,GPIOC,GPIO_Pin_4,0);
      Bling_Set(&Light_2,5000,1000,0.5,0,GPIOC,GPIO_Pin_5,0);
      Bling_Set(&Light_3,5000,1000,0.8,0,GPIOC,GPIO_Pin_10,0);
      WriteFlashNineFloat(Accel_Offset_Address,
                        Accel_Offset_Read.x,
                        Accel_Offset_Read.y,
                        Accel_Offset_Read.z,
                        Accel_Scale_Read.x,
                        Accel_Scale_Read.y,
                        Accel_Scale_Read.z,
                        Mag_Offset_Read.x,
                        Mag_Offset_Read.y,
                        Mag_Offset_Read.z);//写入加速度零点偏执与磁力计中心偏执
      return TRUE;
  }
return FALSE;
}



#define  RC_TOP_DEFAULT       2000
#define  RC_BUTTOM_DEFAULT    1000
#define  RC_MIDDLE_DEFAULT    1500
#define  RC_DEADBAND_DEFAULT  100
#define  RC_DEADBAND_PERCENT  0.1
#define  RC_RESET_DEFAULT  1500

Vector_RC  RC_Calibration[8]={0};
uint8_t RC_Read_Flag[8];
void RC_Calibration_Init()
{
  uint16_t i=0;
  uint32_t max_read[8]={0},min_read[8]={0};
  for(i=0;i<8;i++)
  {
    RC_Read_Flag[i]=ReadFlash_RC(8*i,&min_read[i],&max_read[i]);
  }

  if(RC_Read_Flag[0]!=0x03
     &&RC_Read_Flag[1]!=0x03
       &&RC_Read_Flag[2]!=0x03
         &&RC_Read_Flag[3]!=0x03
           &&RC_Read_Flag[4]!=0x03
             &&RC_Read_Flag[5]!=0x03
               &&RC_Read_Flag[6]!=0x03
                 &&RC_Read_Flag[7]!=0x03)//flash中存在数据
  {
      for(i=0;i<8;i++)
      {
          RC_Calibration[i].max=max_read[i];
          RC_Calibration[i].min=min_read[i];
          RC_Calibration[i].middle=(max_read[i]+min_read[i])/2;
          RC_Calibration[i].deadband=(uint16_t)((max_read[i]-min_read[i])*RC_DEADBAND_PERCENT);
      }
  }
  else//flash中不存在数据
  {
      for(i=0;i<8;i++)
      {
      RC_Calibration[i].max=RC_TOP_DEFAULT;
      RC_Calibration[i].min=RC_BUTTOM_DEFAULT;
      RC_Calibration[i].middle=RC_MIDDLE_DEFAULT;
      RC_Calibration[i].deadband=RC_DEADBAND_DEFAULT;
      }
  }
}

void RC_Calibration_RESET()
{
  uint16_t i=0;
  for(i=0;i<8;i++)
  {
    RC_Calibration[i].max=RC_RESET_DEFAULT;
    RC_Calibration[i].min=RC_RESET_DEFAULT;
  }
}


uint8_t RC_Calibration_Trigger_Flag=0;
void RC_Calibration_Trigger(void)
{
  if(QuadKey2==0)
  {
    delay_ms(500);
    if(QuadKey2==0)
    {
      while(QuadKey2==0);
      RC_Calibration_RESET();//复位遥控器行程值，等待校准完毕
      RC_Calibration_Trigger_Flag=1;
      Page_Number=13;
      Key_Right_Release=1;
    }
  }
  else
  {
    RC_Calibration_Init();//直接从flash里面（或者DEFAULT值）获取遥控器行程输出
    RC_Calibration_Trigger_Flag=0;
  }
}

bool RC_Calibration_Check(uint16 *rc_date)
{
  uint16_t i=0;
  bool success_flag=FALSE;
  if(RC_Calibration_Trigger_Flag==0) return success_flag;
  for(i=0;i<8;i++)
  {
    if(rc_date[i] >= RC_Calibration[i].max)   RC_Calibration[i].max = rc_date[i];//最大行程值
    if(rc_date[i] <  RC_Calibration[i].min)   RC_Calibration[i].min = rc_date[i];//最小行程值
    RC_Calibration[i].middle=(RC_Calibration[i].max+RC_Calibration[i].min)/2;//行程中位
    RC_Calibration[i].deadband=(uint16_t)((RC_Calibration[i].max-RC_Calibration[i].min)*RC_DEADBAND_PERCENT);//设置满量程的百分之RC_DEADBAND_PERCENT为中位死区
  }

  if(QuadKey2==0)//遥控器标定完成后，通过按键来结束标定过程
  {
    delay_ms(2000);
    if(QuadKey2==0)
    {
      while(QuadKey2==0);
      RC_Calibration_Trigger_Flag=0;
      Key_Right_Release=0;
      success_flag=TRUE;
      WriteFlash_RC(0,&RC_Calibration[0],
                      &RC_Calibration[1],
                      &RC_Calibration[2],
                      &RC_Calibration[3],
                      &RC_Calibration[4],
                      &RC_Calibration[5],
                      &RC_Calibration[6],
                      &RC_Calibration[7]);
    }
  }
  return success_flag;
}

void Reset_RC_Calibartion(uint8_t Type)
{
  if(Type==1)  
  {
    RC_Calibration_Trigger_Flag=0;
    Key_Right_Release=0;
  }
}
uint16_t ESC_Calibration_Makesure_Cnt=0;
uint32_t ESC_Calibration_Flag=0;
void ESC_Calibration_Check(void)
{
   if(Throttle_Control==1000
      &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
        &&Roll_Control>=30
          &&Pitch_Control<=-30)
     ESC_Calibration_Makesure_Cnt++;
   //else ESC_Calibration_Makesure_Cnt/=2;

   if(Throttle_Control==1000
      &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
        &&Roll_Control>=30
          &&Pitch_Control<=-30
           &&ESC_Calibration_Makesure_Cnt>200*5//持续5S
            &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
    //进入ESC校准模式
  {
      ESC_Calibration_Flag=1;
      ESC_Calibration_Makesure_Cnt=0;
      Forced_Lock_Makesure_Cnt=0;
      WriteFlash_ESC(0,ESC_Calibration_Flag);
      
      Bling_Set(&Light_1,5000,500,0.2,0,GPIOC,GPIO_Pin_4,1);
      Bling_Set(&Light_2,5000,500,0.2,0,GPIOC,GPIO_Pin_5,1);
      Bling_Set(&Light_3,5000,500,0.2,0,GPIOC,GPIO_Pin_10,1);
      Page_Number=14;
  }
}

#define Thr_Chl_Num  2
void ESC_Calibration()
{
   PWM_Set(PPM_Databuf[Thr_Chl_Num],PPM_Databuf[Thr_Chl_Num],PPM_Databuf[Thr_Chl_Num],PPM_Databuf[Thr_Chl_Num],
           PPM_Databuf[Thr_Chl_Num],PPM_Databuf[Thr_Chl_Num]);
   
}


void ESC_HardWave_Init()//只初始化校准电调的必要资源
{
  NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC初始化结构体
  SystemInit();
  delay_init(72);
  OLED_Init_Fast();
  TIM2_Configuration_Cnt();//TIM2程序计时定时器
  PPM_Init();//PPM遥控器接收初始化
  SBUS_USART5_Init();//串口5、SBUS解析
  PWM_Init();//PWM初始化—TIM4
   
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//优先级组别2，具体参见misc.h line80
  //飞控系统定时器
  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn ;//计数定时器
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  //PPM接收机
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
  
  //SBUS解析串口 
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn; //中断号；
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级；
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //响应优先级；
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  while(1)
  {
   SBUS_Linear_Calibration();
   ESC_Calibration();
   
   LCD_clear_L(0,0);LCD_clear_L(0,1);LCD_P8x16Str(0,0,"Please Move Thr");
   LCD_clear_L(0,2);LCD_clear_L(0,3);LCD_P8x16Str(0,2,"Down When ESC");
   LCD_clear_L(0,4);LCD_clear_L(0,5);LCD_P8x16Str(0,4,"Beep Beep");
   LCD_P6x8Str(80,4,"Thr:");
   write_6_8_number(80,5,PPM_Databuf[2]);
   LCD_clear_L(0,6);LCD_P6x8Str(0,6,"Repower When Set Up");
   
   LCD_clear_L(0,7);
   LCD_P6x8Str(0,7,"SysT:");
   write_6_8_number(30,7,Time_Sys[3]);
   LCD_P6x8Str(45,7,":");
   write_6_8_number(55,7,Time_Sys[2]);
   LCD_P6x8Str(70,7,":");
   write_6_8_number(80,7,Time_Sys[1]);
   LCD_P6x8Str(95,7,":");
   write_6_8_number(105,7,Time_Sys[0]);
  }
}


uint8_t Check_Calibration_Flag(void)
{
  uint8_t cal_flag=0x00; 
  if(Key_Right_Release==1)      cal_flag|=0x01;//遥控器校准
  if(Accel_Calibration_Flag==1) cal_flag|=0x02;//加速度计校准
  if(Mag_Calibration_Flag==1)   cal_flag|=0x04;//磁力计校准
  return cal_flag;
}

