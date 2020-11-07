#include "Headfile.h"
#include "NamelessCorun_Ctrl.h"
#include "control_config.h"
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
/*****************遥控器行程设置**********************/
uint16 Motor_PWM_1,Motor_PWM_2,Motor_PWM_3,Motor_PWM_4,Motor_PWM_5,Motor_PWM_6;//六个电机输出PWM
uint16 Last_Motor_PWM_1,Last_Motor_PWM_2,Last_Motor_PWM_3,Last_Motor_PWM_4,Last_Motor_PWM_5,Last_Motor_PWM_6;//上次六个电机输出PWM
uint8 MotorTest=0xff;//电机序号测试

float Yaw_Feedforward=0.3;//偏航角前馈控制;
uint8_t Controler_High_Mode=1,Last_Controler_High_Mode=1;
uint8_t Controler_Horizontal_Mode=1,Last_Controler_Horizontal_Mode=1;
uint8_t Controler_Land_Mode=1,Last_Controler_Land_Mode=1;
uint8_t Control_Mode_Change=0;
uint16_t High_Hold_Throttle=0;
uint8_t  Pos_Hold_SetFlag=0;
/***************************************************
函数名: void Controler_Mode_Select(void)
说明:	控制器模式选择函数
入口:	无
出口:	无
备注:	中断任务调度持续运行
****************************************************/
void Controler_Mode_Select()
{
   Last_Controler_High_Mode=Controler_High_Mode;//上次高度控制模式
   Last_Controler_Horizontal_Mode=Controler_Horizontal_Mode;//上次位置控制模式
   Last_Controler_Land_Mode=Controler_Land_Mode;//上次返航模式

   if(PPM_Databuf[4]>=(RC_Calibration[4].max-RC_Calibration[4].deadband))       Controler_High_Mode=2;//气压计、超神波定高
   else if(PPM_Databuf[4]<=(RC_Calibration[4].min+RC_Calibration[4].deadband))  Controler_High_Mode=1;//纯姿态自稳

   if(PPM_Databuf[5]>=(RC_Calibration[5].max-RC_Calibration[5].deadband))         Controler_Horizontal_Mode=2;//水平位置控制
   else if(PPM_Databuf[5]<=(RC_Calibration[5].min+RC_Calibration[5].deadband))   Controler_Horizontal_Mode=1;//姿态自稳控制

   if(PPM_Databuf[6]>=(RC_Calibration[6].max-RC_Calibration[6].deadband))        Controler_Land_Mode=2;//返航模式
   else if(PPM_Databuf[6]<=(RC_Calibration[6].min+RC_Calibration[6].deadband))   Controler_Land_Mode=1;//非返航模式

   if(Controler_Land_Mode!=Last_Controler_Land_Mode)
   {
      if(Controler_Land_Mode==1)  
        land_reset();//返航模式切回正常模式
        Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];//将开关拨动瞬间的惯导高度设置为期望高度
   }

   if(Controler_High_Mode!=Last_Controler_High_Mode)
   {
     if(Controler_High_Mode==2)  {Control_Mode_Change=1;}//自稳切定高，设置基准油门值，悬停高度
     if(Controler_High_Mode==1)  {Control_Mode_Change=1;}//定高切自稳
   }
   
   if(Controler_Horizontal_Mode!=Last_Controler_Horizontal_Mode)//位置通道有切换
   {
     if(Controler_Horizontal_Mode==2)  {Control_Mode_Change=2;Pos_Hold_SetFlag=0;}//自稳切定点，设置悬停点
     if(Controler_Horizontal_Mode==1)  {Control_Mode_Change=2;Pos_Hold_SetFlag=1;}//定点时自稳
   }
   
   
   if(Control_Mode_Change==1)//存在定高模式切换，高度只设置一次
   {
       if(Controler_High_Mode==High_Hold_Mode)//本次为定高模式，即自稳切定高
       {
          High_Hold_Throttle=Throttle_Control;//保存当前油门值，只存一次
        /*******************将当前惯导竖直位置估计作为目标高度***************************/
          Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];//将开关拨动瞬间的惯导高度设置为期望高度
       }
       else//本次为自稳模式，即定高切自稳
       {
          PID_Integrate_Reset(&Total_Controller.High_Acce_Control);//清空竖直加速度控制器积分项
          PID_Integrate_Reset(&Total_Controller.High_Speed_Control);//清空竖直加速度控制器积分项
          PID_Integrate_Reset(&Total_Controller.High_Position_Control);//清空竖直加速度控制器积分项
       }
       Control_Mode_Change=0;//将模式切换位置0,有且仅处理一次
   }
   else if(Control_Mode_Change==2)//存在定点模式切换，悬停位置只设置一次
   {
      if(Controler_Horizontal_Mode==Pos_Hold_Mode)//本次为定点模式
      {
         if(Pos_Hold_SetFlag==0&&(GPS_ok()==TRUE))//满足设置悬停点条件
         {
         /*******************将当前惯导水平位置估计作为目标悬停点************************/
          Total_Controller.Latitude_Position_Control.Expect=NamelessQuad.Position[_ROLL];
          Total_Controller.Longitude_Position_Control.Expect=NamelessQuad.Position[_PITCH];
          Pos_Hold_SetFlag=1;
         }
      }
      else//定点切自稳
      {
          PID_Integrate_Reset(&Total_Controller.Latitude_Speed_Control);//清空水平速度控制器积分项
          PID_Integrate_Reset(&Total_Controller.Latitude_Position_Control);//清空水平位置控制器积分项
          PID_Integrate_Reset(&Total_Controller.Longitude_Speed_Control);//清空水平速度控制器积分项
          PID_Integrate_Reset(&Total_Controller.Longitude_Position_Control);//清空水平位置控制器积分项
      }
      Control_Mode_Change=0;//已响应本次定点档位切换
   }
   
/******当前档位为定点模式，但显示悬停点未设置，说明之前未满足设置定点条件有三种情况********
   1、初始通过开关切定点模式时，GPS状态未满足悬停条件；
   2、初始通过开关切定点模式时，GPS状态未满足悬停条件，之后持续检测仍然未满足GPS定点条件；
   3、之前GPS状态满足悬停条件，但由于GPS信号质量变差，自动切换至不满足GPS定点条件；
*******重新判断当下是否满足定点条件，如满足条件更新悬停点，允许进入定点模式******/
   if(Controler_Horizontal_Mode==2)
   {
      if(GPS_ok()==TRUE)//首次切定点不满足定点条件，之后又满足定点条件
      {
        if(Pos_Hold_SetFlag==0)//满足定点条件后，有且仅设置一次
        {
      /*******************将当前惯导水平位置估计作为目标悬停点************************/
        Total_Controller.Latitude_Position_Control.Expect=NamelessQuad.Position[_ROLL];
        Total_Controller.Longitude_Position_Control.Expect=NamelessQuad.Position[_PITCH];
        PID_Integrate_Reset(&Total_Controller.Latitude_Speed_Control);//清空水平速度控制器积分项
        PID_Integrate_Reset(&Total_Controller.Latitude_Position_Control);//清空水平位置控制器积分项
        PID_Integrate_Reset(&Total_Controller.Longitude_Speed_Control);//清空水平速度控制器积分项
        PID_Integrate_Reset(&Total_Controller.Longitude_Position_Control);//清空水平位置控制器积分项
        Pos_Hold_SetFlag=1;
        }
      }
      else//定点档位处于定点模式，但未满足定点条件，将Pos_Hold_SetFlag置0，等待满足时再设置悬停点
      {
        Pos_Hold_SetFlag=0;//不满足定点条件时，复位位置锁定标志位，等待满足定点条件时，再次锁定
        PID_Integrate_Reset(&Total_Controller.Latitude_Speed_Control);//清空水平速度控制器积分项
        PID_Integrate_Reset(&Total_Controller.Latitude_Position_Control);//清空水平位置控制器积分项
        PID_Integrate_Reset(&Total_Controller.Longitude_Speed_Control);//清空水平速度控制器积分项
        PID_Integrate_Reset(&Total_Controller.Longitude_Position_Control);//清空水平位置控制器积分项
      }
   }
/******若满足GPS定点模式，对Pos_Hold_SetFlag置1，允许进入定点模式*****************/
}



uint16 Value_Limit(uint16 Min,uint16 Max,uint16 Data)
{
   if(Data>=Max) Data=Max;
   else if(Data<=Min) Data=Min;
   return Data;
}


float Yaw_Base=0;
uint16 Yaw_Cnt=0;
void Angle_Control()//角度环节
{
      //角度反馈
     Total_Controller.Pitch_Angle_Control.FeedBack=Pitch;
     PID_Control(&Total_Controller.Pitch_Angle_Control);
     Total_Controller.Roll_Angle_Control.FeedBack =Roll;
     PID_Control(&Total_Controller.Roll_Angle_Control);

     if(Yaw_Control==0)//偏航杆置于中位
     {
          if(Yaw_Cnt<=500)//无头模式、飞机上电后一段时间锁定偏航角，磁力计、陀螺仪融合需要一段时间，这里取500
          {
            Yaw_Cnt++;
          }
          if(Total_Controller.Yaw_Angle_Control.Expect==0//回中时赋角度期望值
             ||Yaw_Cnt<=500
               ||Landon_Earth_Flag==1)//或者满足触地条件、复位偏航角期望
          {
            Total_Controller.Yaw_Angle_Control.Expect=Yaw;
          }
          Total_Controller.Yaw_Angle_Control.FeedBack=Yaw;//偏航角反馈

          PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control);//偏航角度控制
          Total_Controller.Yaw_Gyro_Control.Expect=Total_Controller.Yaw_Angle_Control.Control_OutPut;//偏航角速度环期望，来源于偏航角度控制器输出
     }
     else//波动偏航方向杆后，只进行内环角速度控制
     {
        Total_Controller.Yaw_Angle_Control.Expect=0;//偏航角期望给0,不进行角度控制
        Total_Controller.Yaw_Gyro_Control.Expect=Yaw_Control;//偏航角速度环期望，直接来源于遥控器打杆量
     }
}



uint16_t Yaw_Control_Fault_Cnt=0;
void Gyro_Control()//角速度环
{
if(GYRO_CONTROL_MODE==PID_MODE)//俯仰、横滚方向姿态内环角速度控制器采用PID控制器
{
     /***************内环角速度期望****************/
     Total_Controller.Pitch_Gyro_Control.Expect=Total_Controller.Pitch_Angle_Control.Control_OutPut;
     Total_Controller.Roll_Gyro_Control.Expect=Total_Controller.Roll_Angle_Control.Control_OutPut;

     /***************内环角速度反馈****************/
     Total_Controller.Pitch_Gyro_Control.FeedBack=Pitch_Gyro;
     Total_Controller.Roll_Gyro_Control.FeedBack=Roll_Gyro;

     /***************内环角速度控制****************/
     PID_Control_Div_LPF(&Total_Controller.Pitch_Gyro_Control);
     PID_Control_Div_LPF(&Total_Controller.Roll_Gyro_Control);
     
     /***************内环角速度控制：微分参数动态调整****************/
     //PID_Control_Div_LPF_For_Gyro(&Total_Controller.Pitch_Gyro_Control);
     //PID_Control_Div_LPF_For_Gyro(&Total_Controller.Roll_Gyro_Control);
}
else if(GYRO_CONTROL_MODE==ADRC_MODE)//俯仰、横滚方向姿态内环角速度控制器采用ADRC自抗扰控制器
{
    ADRC_Control(&ADRC_Pitch_Controller,
                 Total_Controller.Pitch_Angle_Control.Control_OutPut,//角速度期望
                 Pitch_Gyro);//角速度状态反馈
    ADRC_Control(&ADRC_Roll_Controller,
                 Total_Controller.Roll_Angle_Control.Control_OutPut,//角速度期望
                 Roll_Gyro);//角速度状态反馈
}
else//测试用、正常只选择一种模式
{
    /***************内环角速度期望****************/
    Total_Controller.Pitch_Gyro_Control.Expect=Total_Controller.Pitch_Angle_Control.Control_OutPut;
    Total_Controller.Roll_Gyro_Control.Expect=Total_Controller.Roll_Angle_Control.Control_OutPut;
    /***************内环角速度反馈****************/
    Total_Controller.Pitch_Gyro_Control.FeedBack=Pitch_Gyro;
    Total_Controller.Roll_Gyro_Control.FeedBack=Roll_Gyro;

    /***************内环角速度控制****************/
    PID_Control_Div_LPF(&Total_Controller.Pitch_Gyro_Control);
    PID_Control_Div_LPF(&Total_Controller.Roll_Gyro_Control);

    ADRC_Control(&ADRC_Pitch_Controller,
                 Total_Controller.Pitch_Angle_Control.Control_OutPut,//角速度期望
                 Pitch_Gyro);//角速度状态反馈
    ADRC_Control(&ADRC_Roll_Controller,
                Total_Controller.Roll_Angle_Control.Control_OutPut,//角速度期望
                Roll_Gyro);//角速度状态反馈
}

//偏航角前馈控制
 //Total_Controller.Yaw_Gyro_Control.FeedBack=Yaw_Gyro;
 Total_Controller.Yaw_Gyro_Control.FeedBack=Yaw_Gyro_Earth_Frame;//Yaw_Gyro;
 PID_Control_Div_LPF(&Total_Controller.Yaw_Gyro_Control);
 //**************************偏航角前馈控制**********************************
 Total_Controller.Yaw_Gyro_Control.Control_OutPut+=Yaw_Feedforward*Total_Controller.Yaw_Gyro_Control.Expect;//偏航角前馈控制
 Total_Controller.Yaw_Gyro_Control.Control_OutPut=constrain_float(Total_Controller.Yaw_Gyro_Control.Control_OutPut,
                                                                  -Total_Controller.Yaw_Angle_Control.Control_OutPut_Limit,
                                                                    Total_Controller.Yaw_Angle_Control.Control_OutPut_Limit);
 /*******偏航控制异常情况判断，即偏航控制量很大时，偏航角速度很小，如此时为强外力干扰、已着地等******************************/
 if(ABS(Total_Controller.Yaw_Gyro_Control.Control_OutPut)>Total_Controller.Yaw_Gyro_Control.Control_OutPut_Limit/2//偏航控制输出相对较大
    &&ABS(Yaw_Gyro)<=15.0)//偏航角速度相对很小
  {
    Yaw_Control_Fault_Cnt++;
    if(Yaw_Control_Fault_Cnt>=500) Yaw_Control_Fault_Cnt=500;
  }
 else Yaw_Control_Fault_Cnt/=2;//不满足，快速削减至0

 if(Yaw_Control_Fault_Cnt==400)//持续5ms*400=2S,特殊处理
 {
   PID_Integrate_Reset(&Total_Controller.Yaw_Gyro_Control);//清空偏航角速度控制的积分
   PID_Integrate_Reset(&Total_Controller.Yaw_Angle_Control);//清空偏航角控制的积分
   Total_Controller.Yaw_Angle_Control.Expect=Yaw;//将当前偏航角，作为期望偏航角
   Yaw_Control_Fault_Cnt=0;
 }
/*******偏航控制异常处理结束******************************/
}

uint16 Throttle=0,Last_Throttle=0,Throttle_Hover=1450;
void Main_Leading_Control(void)
{
/*********************根据遥控器切换档位，飞控进入不同模式****************************/
  if(Controler_Land_Mode==1)//非返航着陆模式
  {
        if(Controler_High_Mode==1//姿态自稳定模式
           &&Controler_Horizontal_Mode==1)//GPS定点档位未设置
      {
         Total_Controller.Pitch_Angle_Control.Expect=Target_Angle[0];
         Total_Controller.Roll_Angle_Control.Expect=Target_Angle[1];

         if(Throttle_Control<=1000)   Throttle=1000;
         else Throttle=Throttle_Control;//油门直接来源于遥控器油门给定
         Last_Throttle=Throttle;
      }
      else if(Controler_High_Mode==2//定高模式
              &&Controler_Horizontal_Mode==1)//GPS定点档位未设置
      {
      /**************************定高模式，水平姿态期望角来源于遥控器******************************************/

      #if  (Optical_Enable==0)
         Total_Controller.Pitch_Angle_Control.Expect=Target_Angle[0];
         Total_Controller.Roll_Angle_Control.Expect=Target_Angle[1];
      #else   //光流辅助悬停
        if(Roll_Control==0
            &&Pitch_Control==0)//无水平遥控量给定
        {
            //位置期望
            if(NamelessCotrunOptical.Postion_Expect.x==0&&NamelessCotrunOptical.Postion_Expect.y==0)//方向杆回中后，只设置一次
            {
              NamelessCotrunOptical.Postion_Expect.x=NamelessCotrunOptical.Position.x;
              NamelessCotrunOptical.Postion_Expect.y=NamelessCotrunOptical.Position.y;
            }
            NamelessCotrunOptical.Postion_Feedback.x=NamelessCotrunOptical.Position.x;
            NamelessCotrunOptical.Postion_Feedback.y=NamelessCotrunOptical.Position.y;//位置反馈

            NamelessCotrunOptical.Postion_Err.x=NamelessCotrunOptical.Postion_Expect.x-NamelessCotrunOptical.Postion_Feedback.x;
            NamelessCotrunOptical.Postion_Err.y=NamelessCotrunOptical.Postion_Expect.y-NamelessCotrunOptical.Postion_Feedback.y;//位置偏差

            NamelessCotrunOptical.Speed_Expect.x=constrain_float(0.1*NamelessCotrunOptical.Postion_Err.x,-100,100);//速度期望
            NamelessCotrunOptical.Speed_Expect.y=constrain_float(0.1*NamelessCotrunOptical.Postion_Err.y,-100,100);

            NamelessCotrunOptical.Speed_Feedback.x=NamelessCotrunOptical.Speed.x;
            NamelessCotrunOptical.Speed_Feedback.y=NamelessCotrunOptical.Speed.y;//速度反馈

            NamelessCotrunOptical.Speed_Err.x=NamelessCotrunOptical.Speed_Expect.x-NamelessCotrunOptical.Speed_Feedback.x;
            NamelessCotrunOptical.Speed_Err.y=NamelessCotrunOptical.Speed_Expect.y-NamelessCotrunOptical.Speed_Feedback.y;//速度偏差

            Total_Controller.Pitch_Angle_Control.Expect=constrain_float(-0.15*NamelessCotrunOptical.Speed_Err.y,-15,15);//角度期望
            Total_Controller.Roll_Angle_Control.Expect=constrain_float(0.15*NamelessCotrunOptical.Speed_Err.x,-15,15);//角度期望
        }
        else
        {
            Total_Controller.Pitch_Angle_Control.Expect=Target_Angle[0];
            Total_Controller.Roll_Angle_Control.Expect=Target_Angle[1];
            NamelessCotrunOptical.Postion_Expect.x=0;
            NamelessCotrunOptical.Postion_Expect.y=0;

        }
      #endif
             ncq_control_althold();//高度控制
      }
      else if(Controler_High_Mode==2//定高模式
              &&Controler_Horizontal_Mode==2)//GPS定点档位已设置)
      {
             ncq_control_althold();//高度控制
             ncq_control_poshold();//位置控制
      }
      else//其它
      {
             Total_Controller.Pitch_Angle_Control.Expect=Target_Angle[0];
             Total_Controller.Roll_Angle_Control.Expect=Target_Angle[1];
             if(Throttle_Control<=1000)   Throttle=1000;
             else Throttle=Throttle_Control;//油门直接来源于遥控器油门给定
             Last_Throttle=Throttle;
      }
      land_state_check();
  }
  else//返航着陆模式
  {
    land_run();
  }
}
/************姿态环控制器：角度+角速度****************/
void Attitude_Control(void)
{
  Angle_Control();//角度控制
  Gyro_Control();//角速度控制
}


float Active_Para1,Active_Para2;
/***************************************************
函数名: void Total_Control(void)
说明:	总控制器运行，大体分三步：
        1、根据遥控器输入、当前状态，给定运行模式（自稳+油门手动、定高+自稳、定高+定点（控速）等）
        2、主导上层控制器给定姿态期望，高度控制等
        3、自稳（姿态）控制
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Total_Control(void)
{
  static uint16_t Mode_Check_Cnt=0;
  /***********当PPM_ch6、ch7为模拟通道时，可以作为实时在线调参使用******/
  //Active_Para1=PPM_Databuf[6]-1000;
  //Active_Para2=PPM_Databuf[7]-1000;
  //Active_Para1=0.1*constrain_float(Active_Para1-500,-500,500);
  //Active_Para2=0.0001*constrain_float(Active_Para2,0,1000);
  //Total_Controler.High_Speed_Control.Kp=Active_Para1;
  //Total_Controler.High_Speed_Control.Ki=Active_Para2;
  /*************控制器模式选着******************/
  Mode_Check_Cnt++;
  if(Mode_Check_Cnt>=4)//每20ms检测一次，PPM信号刷新周期为20ms
  {
    Controler_Mode_Select();
    Mode_Check_Cnt=0;
  }
  /*************主导控制器******************/
  Main_Leading_Control();
  /*************姿态环控制器*****************/
  Attitude_Control();
}

uint16_t Throttle_Output=0;
void Throttle_Angle_Compensate()//油门倾角补偿
{
  float CosPitch_CosRoll=ABS(Cos_Pitch*Cos_Roll);
  float Throttle_Makeup=0;
  float Temp=0;
  if(CosPitch_CosRoll>=0.999999)  CosPitch_CosRoll=0.999999;
  if(CosPitch_CosRoll<=0.000001)  CosPitch_CosRoll=0.000001;
  if(CosPitch_CosRoll<=0.50)  CosPitch_CosRoll=0.50;//Pitch,Roll约等于30度
  if(Throttle>=Thr_Start)//大于起转油门量
  {
      Temp=(uint16_t)(MAX(ABS(100*Pitch),ABS(100*Roll)));
      Temp=constrain_float(9000-Temp,0,3000)/(3000*CosPitch_CosRoll);
      Throttle_Makeup=(Throttle-Thr_Start)*Temp;//油门倾角补偿
      Throttle_Output=(uint16_t)(Thr_Start+Throttle_Makeup);
      Throttle_Output=(uint16_t)(constrain_float(Throttle_Output,Thr_Start,2000));
  }
  else Throttle_Output=Throttle;
}


/**************************************************************
***************************************************************
              X型安装方式，电机序号与姿态角关系
                             -
                           Pitch
                   3#                1#
                  ！！！            ！！！
                   ！！！          ！！！
                    ！！！        ！！！
                     ！！！      ！！！
                      ！！！    ！！！
       -   Roll            ！！！            Roll   +
                           ！！！
                      ！！！    ！！！
                     ！！！      ！！！
                    ！！！        ！！！
                   ！！！          ！！！
                  ！！！            ！！！
                   2#                4#
                           Pitch
                             +
加速度传感器轴向与载体X、Y、Z同轴，沿轴向原点看，逆时针旋转角度为+
                            Y Aixs
                            *
                            *
                            *
                            *
                            *
                            *
                            * * * * * * * *   X Axis
                          (0)
*******************************************************************
******************************************************************/
uint16_t Idel_Cnt=0;
#define Idel_Transition_Gap 2//怠速递增间隔时间 10*5=50ms
#define Idel_Transition_Period 100//怠速启动最大计数器  50ms*100=5s
uint16_t Thr_Idle_Transition_Cnt=0;
void Control_Output()
{
    Throttle_Angle_Compensate();//油门倾角补偿
    landon_earth_check();//着陆条件自检
    if(Controler_State==Unlock_Controler)//解锁
    {
      if(Landon_Earth_Flag==1)//检测到着陆条件
      {
        if(Last_Motor_PWM_1<=Thr_Min
           &&Last_Motor_PWM_2<=Thr_Min
             &&Last_Motor_PWM_3<=Thr_Min
               &&Last_Motor_PWM_4<=Thr_Min
                 //&&Last_Motor_PWM_5<=Thr_Min
                   //&&Last_Motor_PWM_6<=Thr_Min
                     )//只有上锁后再解锁时才会满足
        {
           //如果上次油门输出值为最低位，进入怠速时，安排过渡过程
           Thr_Idle_Transition_Cnt=Idel_Transition_Period;
        }
        else//其他时刻进入着陆条件
        {
          if(Last_Landon_Earth_Flag==0)//上次为起飞状态，本次为着陆状态，上锁电机
          {
              Controler_State=Lock_Controler;
              Bling_Set(&Light_1,3000,200,0.5,0,GPIOC,GPIO_Pin_4,0);
              Bling_Set(&Light_2,3000,200,0.5,0,GPIOC,GPIO_Pin_5,0);
              Bling_Set(&Light_3,3000,200,0.5,0,GPIOC,GPIO_Pin_10,0);
          }
        }

        Idel_Cnt++;
        if(Idel_Cnt>=Idel_Transition_Gap)
        {
          if(Thr_Idle_Transition_Cnt>=1)
            Thr_Idle_Transition_Cnt--;
          Idel_Cnt=0;
        }
        Motor_PWM_1=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;//油门怠速
        Motor_PWM_2=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
        Motor_PWM_3=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
        Motor_PWM_4=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
        Motor_PWM_5=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
        Motor_PWM_6=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
        Take_Off_Reset();//清积分
        ADRC_Integrate_Reset(&ADRC_Pitch_Controller);
        ADRC_Integrate_Reset(&ADRC_Roll_Controller);
      }
      else  //解锁后不满足着陆条件，默认起飞
      {
         if(Controler_High_Mode==1)//姿态档位
         {
             if(Throttle>=Thr_Fly_Start)//大于起飞油门
             {
                if(GYRO_CONTROL_MODE==PID_MODE)//水平姿态环角速度读控制器来源于PID
                {
                      Motor_PWM_1=Int_Sort(
                                     Moter1_Thr_Scale*Throttle_Output
                                    +Moter1_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter1_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_2=Int_Sort(
                                     Moter2_Thr_Scale*Throttle_Output
                                    +Moter2_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter2_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_3=Int_Sort(
                                     Moter3_Thr_Scale*Throttle_Output
                                    +Moter3_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter3_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_4=Int_Sort(
                                     Moter4_Thr_Scale*Throttle_Output
                                    +Moter4_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter4_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_5=Int_Sort(
                                     Moter5_Thr_Scale*Throttle_Output
                                    +Moter5_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter5_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter5_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_6=Int_Sort(
                                     Moter6_Thr_Scale*Throttle_Output
                                    +Moter6_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter6_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter6_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);

                }
                else//水平姿态环角速度读控制器来源于ADRC
                {
                      Motor_PWM_1=Int_Sort(
                                     Moter1_Thr_Scale*Throttle_Output
                                    +Moter1_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter1_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter1_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_2=Int_Sort(
                                     Moter2_Thr_Scale*Throttle_Output
                                    +Moter2_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter2_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter2_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_3=Int_Sort(
                                     Moter3_Thr_Scale*Throttle_Output
                                    +Moter3_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter3_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter3_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_4=Int_Sort(
                                     Moter4_Thr_Scale*Throttle_Output
                                    +Moter4_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter4_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter4_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_5=Int_Sort(
                                     Moter5_Thr_Scale*Throttle_Output
                                    +Moter5_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter5_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter5_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_6=Int_Sort(
                                     Moter6_Thr_Scale*Throttle_Output
                                    +Moter6_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter6_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter6_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                }
             }
             else//小于起飞油门
             {
                   Motor_PWM_1=Int_Sort(Throttle_Output);
                   Motor_PWM_2=Int_Sort(Throttle_Output);
                   Motor_PWM_3=Int_Sort(Throttle_Output);
                   Motor_PWM_4=Int_Sort(Throttle_Output);
                   Motor_PWM_5=Int_Sort(Throttle_Output);
                   Motor_PWM_6=Int_Sort(Throttle_Output);
                   Take_Off_Reset();//清积分
                   ADRC_Integrate_Reset(&ADRC_Pitch_Controller);
                   ADRC_Integrate_Reset(&ADRC_Roll_Controller);
             }
                Motor_PWM_1=Value_Limit(Thr_Idle,2000,Motor_PWM_1);//总输出限幅
                Motor_PWM_2=Value_Limit(Thr_Idle,2000,Motor_PWM_2);
                Motor_PWM_3=Value_Limit(Thr_Idle,2000,Motor_PWM_3);
                Motor_PWM_4=Value_Limit(Thr_Idle,2000,Motor_PWM_4);
                Motor_PWM_5=Value_Limit(Thr_Idle,2000,Motor_PWM_5);
                Motor_PWM_6=Value_Limit(Thr_Idle,2000,Motor_PWM_6);
         }
         else if(Controler_High_Mode==2)//油门托管、定高档位
         {
                if(GYRO_CONTROL_MODE==PID_MODE)//水平姿态环角速度读控制器来源于PID
                {
                      Motor_PWM_1=Int_Sort(
                                     Moter1_Thr_Scale*Throttle_Output
                                    +Moter1_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter1_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_2=Int_Sort(
                                     Moter2_Thr_Scale*Throttle_Output
                                    +Moter2_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter2_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_3=Int_Sort(
                                     Moter3_Thr_Scale*Throttle_Output
                                    +Moter3_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter3_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_4=Int_Sort(
                                     Moter4_Thr_Scale*Throttle_Output
                                    +Moter4_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter4_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_5=Int_Sort(
                                     Moter5_Thr_Scale*Throttle_Output
                                    +Moter5_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter5_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter5_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_6=Int_Sort(
                                     Moter6_Thr_Scale*Throttle_Output
                                    +Moter6_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                    +Moter6_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                    +Moter6_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                }
                else//水平姿态环角速度读控制器来源于ADRC
                {
                      Motor_PWM_1=Int_Sort(
                                     Moter1_Thr_Scale*Throttle_Output
                                    +Moter1_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter1_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter1_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_2=Int_Sort(
                                     Moter2_Thr_Scale*Throttle_Output
                                    +Moter2_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter2_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter2_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_3=Int_Sort(
                                     Moter3_Thr_Scale*Throttle_Output
                                    +Moter3_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter3_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter3_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_4=Int_Sort(
                                     Moter4_Thr_Scale*Throttle_Output
                                    +Moter4_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter4_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter4_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_5=Int_Sort(
                                     Moter5_Thr_Scale*Throttle_Output
                                    +Moter5_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter5_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter5_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
                      Motor_PWM_6=Int_Sort(
                                     Moter6_Thr_Scale*Throttle_Output
                                    +Moter6_Roll_Scale*ADRC_Roll_Controller.u
                                    +Moter6_Pitch_Scale*ADRC_Pitch_Controller.u
                                    +Moter6_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);

                }
                Motor_PWM_1=Value_Limit(Thr_Idle,2000,Motor_PWM_1);//总输出限幅
                Motor_PWM_2=Value_Limit(Thr_Idle,2000,Motor_PWM_2);
                Motor_PWM_3=Value_Limit(Thr_Idle,2000,Motor_PWM_3);
                Motor_PWM_4=Value_Limit(Thr_Idle,2000,Motor_PWM_4);
                Motor_PWM_5=Value_Limit(Thr_Idle,2000,Motor_PWM_5);
                Motor_PWM_6=Value_Limit(Thr_Idle,2000,Motor_PWM_6);
         }
      }
    }
    else//未解锁，油门置于最低位，停转
    {
      Motor_PWM_1=Thr_Min;
      Motor_PWM_2=Thr_Min;
      Motor_PWM_3=Thr_Min;
      Motor_PWM_4=Thr_Min;
      Motor_PWM_5=Thr_Min;
      Motor_PWM_6=Thr_Min;
      Take_Off_Reset();//清积分
      Throttle_Control_Reset();
      ADRC_Integrate_Reset(&ADRC_Pitch_Controller);
      ADRC_Integrate_Reset(&ADRC_Roll_Controller);
    }
      Last_Motor_PWM_1=Motor_PWM_1;
      Last_Motor_PWM_2=Motor_PWM_2;
      Last_Motor_PWM_3=Motor_PWM_3;
      Last_Motor_PWM_4=Motor_PWM_4;
      Last_Motor_PWM_5=Motor_PWM_5;
      Last_Motor_PWM_6=Motor_PWM_6;


      Motor_PWM_1=Value_Limit(0,2000,Motor_PWM_1);//总输出限幅
      Motor_PWM_2=Value_Limit(0,2000,Motor_PWM_2);
      Motor_PWM_3=Value_Limit(0,2000,Motor_PWM_3);
      Motor_PWM_4=Value_Limit(0,2000,Motor_PWM_4);
      Motor_PWM_5=Value_Limit(0,2000,Motor_PWM_5);
      Motor_PWM_6=Value_Limit(0,2000,Motor_PWM_6);
      if(MotorTest==0x00)
      {
        PWM_Set(Thr_Min,Thr_Min,Thr_Min,Thr_Min,Thr_Min,Thr_Min);
      }
      else
      {
        PWM_Set((0x01&MotorTest) ? Motor_PWM_1:0,
                (0x02&MotorTest) ? Motor_PWM_2:0,
                (0x04&MotorTest) ? Motor_PWM_3:0,
                (0x08&MotorTest) ? Motor_PWM_4:0,
                (0x10&MotorTest) ? Motor_PWM_5:0,
                (0x20&MotorTest) ? Motor_PWM_6:0);
      }
}




