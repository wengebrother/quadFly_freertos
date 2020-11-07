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
#include "Headfile.h"
#include "PID.h"
/*
1偏差限幅标志；  2积分限幅标志；3积分分离标志；   4期望；
5反馈            6偏差；        7上次偏差；       8偏差限幅值；
9积分分离偏差值；10积分值       11积分限幅值；    12控制参数Kp；
13控制参数Ki；   14控制参数Kd； 15控制器总输出；  16上次控制器总输出
17总输出限幅度
*/
const float Control_Unit[15][17]=
{
/*                                         Kp        Ki        Kd            */
 /*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17*/
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,30  ,0  ,0 , 20,   4.50   ,0.0000  ,0.00  ,0  ,0 , 300},//Pitch_Angle;偏航角度
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 , 200,  0.95   ,3.5000  ,10.0  ,0  ,0 , 500},//Pitch_Gyro;偏航角速度
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,30  ,0  ,0 , 20,   4.50   ,0.0000  ,0.00  ,0  ,0 , 300},//Roll_Angle;横滚角
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 , 200,  0.95   ,3.5000  ,10.0  ,0  ,0 , 500},//Roll_Gyro;横滚角速度
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,45  ,0  ,0 , 150 , 5.00   ,0.0000  ,0.00  ,0  ,0 , 300},//Yaw_Angle;偏航角
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,250 ,0  ,0 , 100,  1.50   ,1.0000  ,4.50  ,0  ,0 , 300},//Yaw_Gyro;偏航角速度

 //定高参数
 //高度单项比例控制，有偏差限幅、总输出即为最大攀升、下降速度400cm/s
 //Z轴速度比例+积分控制，无偏差限幅
#if (YAW_Pos_Control_Accel_Disable==1)
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 ,50 ,   1.0     ,0.000   ,0    ,0   ,0 ,400},//High_Position;海拔高度位置
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,400 ,0  ,0 ,400 ,  2.0     ,10.00   ,0.15  ,0  ,0 ,600},//High_Speed;海拔攀升速度
#else
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 ,100 ,  1.0     ,0.000   ,0    ,0  ,0 ,400},//High_Position;海拔高度位置
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,400 ,0  ,0 ,500 ,  3.0     ,0.200   ,0.1  ,0  ,0 ,500},//High_Speed;海拔攀升速度
#endif
 /*
1偏差限幅标志；  2积分限幅标志；3积分分离标志；   4期望；
5反馈            6偏差；        7上次偏差；       8偏差限幅值；
9积分分离偏差值；10积分值       11积分限幅值；    12控制参数Kp；
13控制参数Ki；   14控制参数Kd； 15控制器总输出；  16上次控制器总输出
17总输出限幅度
*/
/*                                       Kp        Ki        Kd            */
 /*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17*/
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,180  ,0 ,0 ,8,   0.150    ,0.000    ,0    ,0    ,0 ,150},//Longitude_Position;水平经度位置
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,300 ,0  ,0 ,15,  0.080    ,0.030    ,0    ,0    ,0 ,25},//Longitude_Speed;水平经度速度
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,180 ,0  ,0 ,8,   0.150    ,0.000    ,0    ,0    ,0 ,150},//Latitude_Position;水平纬度位置
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,300 ,0  ,0 ,15,  0.080    ,0.030    ,0    ,0    ,0 ,25},//Latitude_Speed;水平纬度速度
  /*************加速度控制器****************/
 //期望最大加速度500cm/s^2
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,600  ,0  ,0 ,500,0.12    ,0.4000    ,0.0  ,0   ,0 ,600},//垂直加速度控制器
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0  ,0 ,3,  0.32    ,0.0000    ,0    ,0   ,0 ,150},//水平经度方向加速度控制器
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0  ,0 ,15, 0.45    ,0.0000    ,0.0  ,0   ,0 ,25},//水平维度方向加速度控制器
};

/*
Butter_Parameter Control_Device_Div_LPF_Parameter={
 //200---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};

Butter_Parameter Control_Device_Err_LPF_Parameter={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};
*/
Butter_Parameter Control_Device_Div_LPF_Parameter;
Butter_Parameter Control_Device_Err_LPF_Parameter;

AllControler Total_Controller;//系统总控制器
void PID_Init(PID_Controler *Controler,Controler_Label Label)
{
  Controler->Err_Limit_Flag=(uint8)(Control_Unit[Label][0]);//1偏差限幅标志
  Controler->Integrate_Limit_Flag=(uint8)(Control_Unit[Label][1]);//2积分限幅标志
  Controler->Integrate_Separation_Flag=(uint8)(Control_Unit[Label][2]);//3积分分离标志
  Controler->Expect=Control_Unit[Label][3];//4期望
  Controler->FeedBack=Control_Unit[Label][4];//5反馈值
  Controler->Err=Control_Unit[Label][5];//6偏差
  Controler->Last_Err=Control_Unit[Label][6];//7上次偏差
  Controler->Err_Max=Control_Unit[Label][7];//8偏差限幅值
  Controler->Integrate_Separation_Err=Control_Unit[Label][8];//9积分分离偏差值
  Controler->Integrate=Control_Unit[Label][9];//10积分值
  Controler->Integrate_Max=Control_Unit[Label][10];//11积分限幅值
  Controler->Kp=Control_Unit[Label][11];//12控制参数Kp
  Controler->Ki=Control_Unit[Label][12];//13控制参数Ki
  Controler->Kd=Control_Unit[Label][13];//14控制参数Ki
  Controler->Control_OutPut=Control_Unit[Label][14];//15控制器总输出
  Controler->Last_Control_OutPut=Control_Unit[Label][15];//16上次控制器总输出
  Controler->Control_OutPut_Limit=Control_Unit[Label][16];//16上次控制器总输出
}

void Total_PID_Init(void)
{
 PID_Init(&Total_Controller.Pitch_Angle_Control,Pitch_Angle_Controler);
 PID_Init(&Total_Controller.Pitch_Gyro_Control,Pitch_Gyro_Controler);
 PID_Init(&Total_Controller.Roll_Angle_Control,Roll_Angle_Controler);
 PID_Init(&Total_Controller.Roll_Gyro_Control,Roll_Gyro_Controler);
 PID_Init(&Total_Controller.Yaw_Angle_Control,Yaw_Angle_Controler);
 PID_Init(&Total_Controller.Yaw_Gyro_Control,Yaw_Gyro_Controler);
 PID_Init(&Total_Controller.High_Position_Control,High_Position_Controler);
 PID_Init(&Total_Controller.High_Speed_Control,High_Speed_Controler);
 PID_Init(&Total_Controller.Longitude_Position_Control,Longitude_Position_Controler);
 PID_Init(&Total_Controller.Longitude_Speed_Control,Longitude_Speed_Controler);
 PID_Init(&Total_Controller.Latitude_Position_Control,Latitude_Position_Controler);
 PID_Init(&Total_Controller.Latitude_Speed_Control,Latitude_Speed_Controler);

 PID_Init(&Total_Controller.High_Acce_Control,High_Acce_Controler);
 PID_Init(&Total_Controller.Longitude_Acce_Control,Longitude_Acce_Controler);
 PID_Init(&Total_Controller.Latitude_Acce_Control,Latitude_Acce_Controler);

 Set_Cutoff_Frequency(Sampling_Freq, 5 ,&Control_Device_Err_LPF_Parameter);
 Set_Cutoff_Frequency(Sampling_Freq, 20,&Control_Device_Div_LPF_Parameter);
}

float PID_Control(PID_Controler *Controler)
{
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
                         +Controler->Integrate//积分
                         +Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Yaw(PID_Controler *Controler)
{
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
/***********************偏航角偏差超过+-180处理*****************************/
  if(Controler->Err<-180)  Controler->Err=Controler->Err+360;
  if(Controler->Err>180)  Controler->Err=Controler->Err-360;

  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
                         +Controler->Integrate//积分
                         +Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}



float Control_Device_LPF(float curr_inputer,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
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

float PID_Control_Div_LPF(PID_Controler *Controler)
{
  int16  i=0;
  float tempa,tempb,tempc,max,min;//用于防跳变滤波
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//原始微分

  /******************************************/
  //均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常
  tempa=Controler->Pre_Last_Dis_Err_LPF;
  tempb=Controler->Last_Dis_Err_LPF;
  tempc=Controler->Dis_Err;
  max = tempa > tempb ? tempa:tempb;
  max = max > tempc ? max:tempc;
  min = tempa < tempb ? tempa:tempb;
  min = min < tempc ? min:tempc;
  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
  /*****************************************/

  for(i=4;i>0;i--)//数字低通后微分项保存
  {
  Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Div_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz

  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
                         +Controler->Integrate//积分
                         //+Controler->Kd*Controler->Dis_Err;//微分
                         +Controler->Kd*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}


float PID_Control_Err_LPF(PID_Controler *Controler)
{
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//原始微分

  Controler->Last_Err_LPF=Controler->Err_LPF;
  Controler->Err_LPF=Control_Device_LPF(Controler->Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Err_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz

  Controler->Dis_Err_LPF=Controler->Err_LPF-Controler->Last_Err_LPF;//偏差经过低通后的微分量

  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err_LPF>=Controler->Err_Max)   Controler->Err_LPF= Controler->Err_Max;
  if(Controler->Err_LPF<=-Controler->Err_Max)  Controler->Err_LPF=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err_LPF)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err_LPF//比例
                         +Controler->Integrate//积分
                          +Controler->Kd*Controler->Dis_Err_LPF;//已对偏差低通，此处不再对微分项单独低通
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}



float Differential_Forward_PID_Control_Div_LPF(PID_Controler *Controler)//微分先行PID控制器
{
  int16  i=0;
  float tempa,tempb,tempc,max,min;//用于防跳变滤波
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  Controler->Dis_Err=-(Controler->FeedBack-Controler->Last_FeedBack);//只对反馈信号微分
  Controler->Last_FeedBack=Controler->FeedBack;//记录上次反馈值
  /******************************************/
  //均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常
  tempa=Controler->Pre_Last_Dis_Err_LPF;
  tempb=Controler->Last_Dis_Err_LPF;
  tempc=Controler->Dis_Err;
  max = tempa > tempb ? tempa:tempb;
  max = max > tempc ? max:tempc;
  min = tempa < tempb ? tempa:tempb;
  min = min < tempc ? min:tempc;
  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
  /*****************************************/

  for(i=4;i>0;i--)//数字低通后微分项保存
  {
  Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Div_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz

  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
                         +Controler->Integrate//积分
                         //+Controler->Kd*Controler->Dis_Err;//微分
                         +Controler->Kd*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Div_LPF_For_Gyro(PID_Controler *Controler)
{
  int16  i=0;
  float tempa,tempb,tempc,max,min;//用于防跳变滤波
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******偏差计算*********************/
  Controler->Pre_Last_Err=Controler->Last_Err;//上上次偏差
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  Controler->Dis_Err=(Controler->Err-Controler->Pre_Last_Err);//间隔了一次采样的微分
  /******************************************/
  //均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常
  tempa=Controler->Pre_Last_Dis_Err_LPF;
  tempb=Controler->Last_Dis_Err_LPF;
  tempc=Controler->Dis_Err;
  max = tempa > tempb ? tempa:tempb;
  max = max > tempc ? max:tempc;
  min = tempa < tempb ? tempa:tempb;
  min = min < tempc ? min:tempc;
  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
  /*****************************************/
  for(i=4;i>0;i--)//数字低通后微分项保存
  {
  Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Div_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz
  Controler->Dis_Error_History[0]=constrain_float(Controler->Dis_Error_History[0],-200,200);//微分项限幅
  Controler->Adaptable_Kd=Controler->Kd*(1+Controler->Dis_Error_History[0]/200);//自适应微分参数
    
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
                         +Controler->Integrate//积分
                         //+Controler->Kd*Controler->Dis_Err;//微分
                         //+Controler->Kd*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
                         +Controler->Adaptable_Kd*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}


void  PID_Integrate_Reset(PID_Controler *Controler)  {Controler->Integrate=0.0f;}

void Take_Off_Reset(void)
{
  PID_Integrate_Reset(&Total_Controller.Roll_Gyro_Control);//起飞前屏蔽积分
  PID_Integrate_Reset(&Total_Controller.Pitch_Gyro_Control);
  PID_Integrate_Reset(&Total_Controller.Yaw_Gyro_Control);
  PID_Integrate_Reset(&Total_Controller.Pitch_Angle_Control);
  PID_Integrate_Reset(&Total_Controller.Roll_Angle_Control);
  PID_Integrate_Reset(&Total_Controller.Yaw_Angle_Control);

  PID_Integrate_Reset(&Total_Controller.Longitude_Speed_Control);//位置控制速度环积分项
  PID_Integrate_Reset(&Total_Controller.Latitude_Speed_Control);
}

void Throttle_Control_Reset(void)
{
  PID_Integrate_Reset(&Total_Controller.High_Acce_Control);
  PID_Integrate_Reset(&Total_Controller.High_Speed_Control);
  PID_Integrate_Reset(&Total_Controller.High_Position_Control);
}

volatile FLASH_Status FLASHStatus1 = FLASH_COMPLETE;      //Flash操作状态变量
Vector3f_pid PID_Parameter[11]={0};
void WriteFlash_PID_Float(uint32_t WriteAddress,
                         Vector3f_pid pid1,
                         Vector3f_pid pid2,
                         Vector3f_pid pid3,
                         Vector3f_pid pid4,
                         Vector3f_pid pid5,
                         Vector3f_pid pid6,
                         Vector3f_pid pid7,
                         Vector3f_pid pid8,
                         Vector3f_pid pid9,
                         Vector3f_pid pid10,
                         Vector3f_pid pid11)
{
        uint32_t Buf[33]={0};
        uint16_t i=0;
        Buf[0]=*(uint32_t *)(&pid1.p);//把内存里面这四个字节写入到Flash
        Buf[1]=*(uint32_t *)(&pid1.i);
        Buf[2]=*(uint32_t *)(&pid1.d);
        Buf[3]=*(uint32_t *)(&pid2.p);
        Buf[4]=*(uint32_t *)(&pid2.i);
        Buf[5]=*(uint32_t *)(&pid2.d);
        Buf[6]=*(uint32_t *)(&pid3.p);
        Buf[7]=*(uint32_t *)(&pid3.i);
        Buf[8]=*(uint32_t *)(&pid3.d);
        Buf[9]=*(uint32_t *)(&pid4.p);
        Buf[10]=*(uint32_t *)(&pid4.i);
        Buf[11]=*(uint32_t *)(&pid4.d);
        Buf[12]=*(uint32_t *)(&pid5.p);
        Buf[13]=*(uint32_t *)(&pid5.i);
        Buf[14]=*(uint32_t *)(&pid5.d);
        Buf[15]=*(uint32_t *)(&pid6.p);
        Buf[16]=*(uint32_t *)(&pid6.i);
        Buf[17]=*(uint32_t *)(&pid6.d);
        Buf[18]=*(uint32_t *)(&pid7.p);
        Buf[19]=*(uint32_t *)(&pid7.i);
        Buf[20]=*(uint32_t *)(&pid7.d);
        Buf[21]=*(uint32_t *)(&pid8.p);
        Buf[22]=*(uint32_t *)(&pid8.i);
        Buf[23]=*(uint32_t *)(&pid8.d);
        Buf[24]=*(uint32_t *)(&pid9.p);
        Buf[25]=*(uint32_t *)(&pid9.i);
        Buf[26]=*(uint32_t *)(&pid9.d);
        Buf[27]=*(uint32_t *)(&pid10.p);
        Buf[28]=*(uint32_t *)(&pid10.i);
        Buf[29]=*(uint32_t *)(&pid10.d);
        Buf[30]=*(uint32_t *)(&pid11.p);
        Buf[31]=*(uint32_t *)(&pid11.i);
        Buf[32]=*(uint32_t *)(&pid11.d);
        FLASH_UnlockBank1();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASHStatus1 = FLASH_ErasePage(PID_STARTADDR);
	if(FLASHStatus1 == FLASH_COMPLETE)
	{
          for(i=0;i<33;i++)
          {
             FLASHStatus1 = FLASH_ProgramWord(PID_STARTADDR + WriteAddress+4*i,Buf[i]);
          }
	}
	FLASH_LockBank1();
}


uint8_t ReadFlash_PID_Float(uint32_t ReadAddress,
                         float *WriteData1,
                         float *WriteData2,
                         float *WriteData3)
{
    uint8_t buf[12];
    uint16_t i=0;
    uint8_t flag=0x00;
    ReadAddress = (uint32_t)PID_STARTADDR + ReadAddress;
    *WriteData1=*(float *)(ReadAddress);
    *WriteData2=*(float *)(ReadAddress+4);
    *WriteData3=*(float *)(ReadAddress+8);
    FLASH_LockBank1();

    for(i=0;i<12;i++)//单字节数据
    {
        *(buf+i)=*(__IO uint8_t*) ReadAddress++;
    }
    if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff))
       flag=flag|0x01;
    if((buf[4]==0xff&&buf[5]==0xff&&buf[6]==0xff&&buf[7]==0xff))
       flag=flag|0x02;
    if((buf[8]==0xff&&buf[9]==0xff&&buf[10]==0xff&&buf[11]==0xff))
       flag=flag|0x04;
    return flag;
}

uint8_t Sort_PID_Cnt=0;
uint8_t Sort_PID_Flag=0;
void Save_Or_Reset_PID_Parameter()
{
    if(Sort_PID_Flag==1)//将地面站设置PID参数写入Flash
    {
        PID_Parameter[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
        PID_Parameter[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
        PID_Parameter[0].d=Total_Controller.Pitch_Gyro_Control.Kd;

        PID_Parameter[1].p=Total_Controller.Roll_Gyro_Control.Kp;
        PID_Parameter[1].i=Total_Controller.Roll_Gyro_Control.Ki;
        PID_Parameter[1].d=Total_Controller.Roll_Gyro_Control.Kd;

        PID_Parameter[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
        PID_Parameter[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
        PID_Parameter[2].d=Total_Controller.Yaw_Gyro_Control.Kd;

        PID_Parameter[3].p=Total_Controller.Pitch_Angle_Control.Kp;
        PID_Parameter[3].i=Total_Controller.Pitch_Angle_Control.Ki;
        PID_Parameter[3].d=Total_Controller.Pitch_Angle_Control.Kd;

        PID_Parameter[4].p=Total_Controller.Roll_Angle_Control.Kp;
        PID_Parameter[4].i=Total_Controller.Roll_Angle_Control.Ki;
        PID_Parameter[4].d=Total_Controller.Roll_Angle_Control.Kd;

        PID_Parameter[5].p=Total_Controller.Yaw_Angle_Control.Kp;
        PID_Parameter[5].i=Total_Controller.Yaw_Angle_Control.Ki;
        PID_Parameter[5].d=Total_Controller.Yaw_Angle_Control.Kd;

        PID_Parameter[6].p=Total_Controller.High_Speed_Control.Kp;
        PID_Parameter[6].i=Total_Controller.High_Speed_Control.Ki;
        PID_Parameter[6].d=Total_Controller.High_Speed_Control.Kd;

        PID_Parameter[7].p=Total_Controller.High_Position_Control.Kp;
        PID_Parameter[7].i=Total_Controller.High_Position_Control.Ki;
        PID_Parameter[7].d=Total_Controller.High_Position_Control.Kd;

        PID_Parameter[8].p=Total_Controller.Latitude_Speed_Control.Kp;
        PID_Parameter[8].i=Total_Controller.Latitude_Speed_Control.Ki;
        PID_Parameter[8].d=Total_Controller.Latitude_Speed_Control.Kd;

        PID_Parameter[9].p=Total_Controller.Latitude_Position_Control.Kp;
        PID_Parameter[9].i=Total_Controller.Latitude_Position_Control.Ki;
        PID_Parameter[9].d=Total_Controller.Latitude_Position_Control.Kd;

        PID_Parameter[10].p=Total_Controller.High_Acce_Control.Kp;
        PID_Parameter[10].i=Total_Controller.High_Acce_Control.Ki;
        PID_Parameter[10].d=Total_Controller.High_Acce_Control.Kd;

        WriteFlash_PID_Float(PID1_Address,
                             PID_Parameter[0],
                             PID_Parameter[1],
                             PID_Parameter[2],
                             PID_Parameter[3],
                             PID_Parameter[4],
                             PID_Parameter[5],
                             PID_Parameter[6],
                             PID_Parameter[7],
                             PID_Parameter[8],
                             PID_Parameter[9],
                             PID_Parameter[10]);

        Sort_PID_Flag=0;
    }
    else if(Sort_PID_Flag==2)//将复位PID参数，并写入Flash
    {
        Total_PID_Init();//将PID参数重置为参数Control_Unit表里面参数
        PID_Parameter[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
        PID_Parameter[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
        PID_Parameter[0].d=Total_Controller.Pitch_Gyro_Control.Kd;

        PID_Parameter[1].p=Total_Controller.Roll_Gyro_Control.Kp;
        PID_Parameter[1].i=Total_Controller.Roll_Gyro_Control.Ki;
        PID_Parameter[1].d=Total_Controller.Roll_Gyro_Control.Kd;

        PID_Parameter[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
        PID_Parameter[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
        PID_Parameter[2].d=Total_Controller.Yaw_Gyro_Control.Kd;

        PID_Parameter[3].p=Total_Controller.Pitch_Angle_Control.Kp;
        PID_Parameter[3].i=Total_Controller.Pitch_Angle_Control.Ki;
        PID_Parameter[3].d=Total_Controller.Pitch_Angle_Control.Kd;

        PID_Parameter[4].p=Total_Controller.Roll_Angle_Control.Kp;
        PID_Parameter[4].i=Total_Controller.Roll_Angle_Control.Ki;
        PID_Parameter[4].d=Total_Controller.Roll_Angle_Control.Kd;

        PID_Parameter[5].p=Total_Controller.Yaw_Angle_Control.Kp;
        PID_Parameter[5].i=Total_Controller.Yaw_Angle_Control.Ki;
        PID_Parameter[5].d=Total_Controller.Yaw_Angle_Control.Kd;

        PID_Parameter[6].p=Total_Controller.High_Speed_Control.Kp;
        PID_Parameter[6].i=Total_Controller.High_Speed_Control.Ki;
        PID_Parameter[6].d=Total_Controller.High_Speed_Control.Kd;

        PID_Parameter[7].p=Total_Controller.High_Position_Control.Kp;
        PID_Parameter[7].i=Total_Controller.High_Position_Control.Ki;
        PID_Parameter[7].d=Total_Controller.High_Position_Control.Kd;

        PID_Parameter[8].p=Total_Controller.Latitude_Speed_Control.Kp;
        PID_Parameter[8].i=Total_Controller.Latitude_Speed_Control.Ki;
        PID_Parameter[8].d=Total_Controller.Latitude_Speed_Control.Kd;

        PID_Parameter[9].p=Total_Controller.Latitude_Position_Control.Kp;
        PID_Parameter[9].i=Total_Controller.Latitude_Position_Control.Ki;
        PID_Parameter[9].d=Total_Controller.Latitude_Position_Control.Kd;

        PID_Parameter[10].p=Total_Controller.High_Acce_Control.Kp;
        PID_Parameter[10].i=Total_Controller.High_Acce_Control.Ki;
        PID_Parameter[10].d=Total_Controller.High_Acce_Control.Kd;

        WriteFlash_PID_Float(PID1_Address,
                             PID_Parameter[0],
                             PID_Parameter[1],
                             PID_Parameter[2],
                             PID_Parameter[3],
                             PID_Parameter[4],
                             PID_Parameter[5],
                             PID_Parameter[6],
                             PID_Parameter[7],
                             PID_Parameter[8],
                             PID_Parameter[9],
                             PID_Parameter[10]);
        Sort_PID_Flag=0;
        ANO_Send_PID_Flag[0]=1;//回复默认参数后，将更新的数据发送置地面站
        ANO_Send_PID_Flag[1]=1;
        ANO_Send_PID_Flag[2]=1;
        ANO_Send_PID_Flag[3]=1;
        ANO_Send_PID_Flag[4]=1;
        ANO_Send_PID_Flag[5]=1;
    }
    else if(Sort_PID_Flag==3)//将复位PID参数，并写入Flash
    {
        Total_PID_Init();//将PID参数重置为参数Control_Unit表里面参数
        PID_Parameter[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
        PID_Parameter[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
        PID_Parameter[0].d=Total_Controller.Pitch_Gyro_Control.Kd;

        PID_Parameter[1].p=Total_Controller.Roll_Gyro_Control.Kp;
        PID_Parameter[1].i=Total_Controller.Roll_Gyro_Control.Ki;
        PID_Parameter[1].d=Total_Controller.Roll_Gyro_Control.Kd;

        PID_Parameter[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
        PID_Parameter[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
        PID_Parameter[2].d=Total_Controller.Yaw_Gyro_Control.Kd;

        PID_Parameter[3].p=Total_Controller.Pitch_Angle_Control.Kp;
        PID_Parameter[3].i=Total_Controller.Pitch_Angle_Control.Ki;
        PID_Parameter[3].d=Total_Controller.Pitch_Angle_Control.Kd;

        PID_Parameter[4].p=Total_Controller.Roll_Angle_Control.Kp;
        PID_Parameter[4].i=Total_Controller.Roll_Angle_Control.Ki;
        PID_Parameter[4].d=Total_Controller.Roll_Angle_Control.Kd;

        PID_Parameter[5].p=Total_Controller.Yaw_Angle_Control.Kp;
        PID_Parameter[5].i=Total_Controller.Yaw_Angle_Control.Ki;
        PID_Parameter[5].d=Total_Controller.Yaw_Angle_Control.Kd;

        PID_Parameter[6].p=Total_Controller.High_Speed_Control.Kp;
        PID_Parameter[6].i=Total_Controller.High_Speed_Control.Ki;
        PID_Parameter[6].d=Total_Controller.High_Speed_Control.Kd;

        PID_Parameter[7].p=Total_Controller.High_Position_Control.Kp;
        PID_Parameter[7].i=Total_Controller.High_Position_Control.Ki;
        PID_Parameter[7].d=Total_Controller.High_Position_Control.Kd;

        PID_Parameter[8].p=Total_Controller.Latitude_Speed_Control.Kp;
        PID_Parameter[8].i=Total_Controller.Latitude_Speed_Control.Ki;
        PID_Parameter[8].d=Total_Controller.Latitude_Speed_Control.Kd;

        PID_Parameter[9].p=Total_Controller.Latitude_Position_Control.Kp;
        PID_Parameter[9].i=Total_Controller.Latitude_Position_Control.Ki;
        PID_Parameter[9].d=Total_Controller.Latitude_Position_Control.Kd;

        PID_Parameter[10].p=Total_Controller.High_Acce_Control.Kp;
        PID_Parameter[10].i=Total_Controller.High_Acce_Control.Ki;
        PID_Parameter[10].d=Total_Controller.High_Acce_Control.Kd;

        WriteFlash_PID_Float(PID1_Address,
                             PID_Parameter[0],
                             PID_Parameter[1],
                             PID_Parameter[2],
                             PID_Parameter[3],
                             PID_Parameter[4],
                             PID_Parameter[5],
                             PID_Parameter[6],
                             PID_Parameter[7],
                             PID_Parameter[8],
                             PID_Parameter[9],
                             PID_Parameter[10]);
        Sort_PID_Flag=0;
        ANO_Send_PID_Flag_USB[0]=1;//回复默认参数后，将更新的数据发送置地面站
        ANO_Send_PID_Flag_USB[1]=1;
        ANO_Send_PID_Flag_USB[2]=1;
        ANO_Send_PID_Flag_USB[3]=1;
        ANO_Send_PID_Flag_USB[4]=1;
        ANO_Send_PID_Flag_USB[5]=1;
    }
}


typedef struct
{
uint8_t No_0xFF[11];
}PID_Parameter_Flag;
PID_Parameter_Flag PID_Parameter_Read_Flag;
void PID_Paramter_Init_With_Flash()
{
  uint16_t i=0;
  Total_PID_Init();
  for(i=0;i<11;i++)
  {
    PID_Parameter_Read_Flag.No_0xFF[0]=ReadFlash_PID_Float(PID1_Address+12*i,
                        &PID_Parameter[i].p,
                        &PID_Parameter[i].i,
                        &PID_Parameter[i].d);
  }
  if(PID_Parameter_Read_Flag.No_0xFF[0]!=0x07
     &&PID_Parameter_Read_Flag.No_0xFF[1]!=0x07
       &&PID_Parameter_Read_Flag.No_0xFF[2]!=0x07
         &&PID_Parameter_Read_Flag.No_0xFF[3]!=0x07
           &&PID_Parameter_Read_Flag.No_0xFF[4]!=0x07
             &&PID_Parameter_Read_Flag.No_0xFF[5]!=0x07
               &&PID_Parameter_Read_Flag.No_0xFF[6]!=0x07
                 &&PID_Parameter_Read_Flag.No_0xFF[7]!=0x07
                   &&PID_Parameter_Read_Flag.No_0xFF[8]!=0x07
                     &&PID_Parameter_Read_Flag.No_0xFF[9]!=0x07
                       &&PID_Parameter_Read_Flag.No_0xFF[10]!=0x07)//Flash内数据正常，更新PID参数值
  {
        Total_Controller.Pitch_Gyro_Control.Kp=PID_Parameter[0].p;
        Total_Controller.Pitch_Gyro_Control.Ki=PID_Parameter[0].i;
        Total_Controller.Pitch_Gyro_Control.Kd=PID_Parameter[0].d;

        Total_Controller.Roll_Gyro_Control.Kp=PID_Parameter[1].p;
        Total_Controller.Roll_Gyro_Control.Ki=PID_Parameter[1].i;
        Total_Controller.Roll_Gyro_Control.Kd=PID_Parameter[1].d;

        Total_Controller.Yaw_Gyro_Control.Kp=PID_Parameter[2].p;
        Total_Controller.Yaw_Gyro_Control.Ki=PID_Parameter[2].i;
        Total_Controller.Yaw_Gyro_Control.Kd=PID_Parameter[2].d;

        Total_Controller.Pitch_Angle_Control.Kp=PID_Parameter[3].p;
        Total_Controller.Pitch_Angle_Control.Ki=PID_Parameter[3].i;
        Total_Controller.Pitch_Angle_Control.Kd=PID_Parameter[3].d;

        Total_Controller.Roll_Angle_Control.Kp=PID_Parameter[4].p;
        Total_Controller.Roll_Angle_Control.Ki=PID_Parameter[4].i;
        Total_Controller.Roll_Angle_Control.Kd=PID_Parameter[4].d;

        Total_Controller.Yaw_Angle_Control.Kp=PID_Parameter[5].p;
        Total_Controller.Yaw_Angle_Control.Ki=PID_Parameter[5].i;
        Total_Controller.Yaw_Angle_Control.Kd=PID_Parameter[5].d;

        Total_Controller.High_Speed_Control.Kp=PID_Parameter[6].p;
        Total_Controller.High_Speed_Control.Ki=PID_Parameter[6].i;
        Total_Controller.High_Speed_Control.Kd=PID_Parameter[6].d;

        Total_Controller.High_Position_Control.Kp=PID_Parameter[7].p;
        Total_Controller.High_Position_Control.Ki=PID_Parameter[7].i;
        Total_Controller.High_Position_Control.Kd=PID_Parameter[7].d;

        Total_Controller.Latitude_Speed_Control.Kp=PID_Parameter[8].p;
        Total_Controller.Latitude_Speed_Control.Ki=PID_Parameter[8].i;
        Total_Controller.Latitude_Speed_Control.Kd=PID_Parameter[8].d;

        Total_Controller.Latitude_Position_Control.Kp=PID_Parameter[9].p;
        Total_Controller.Latitude_Position_Control.Ki=PID_Parameter[9].i;
        Total_Controller.Latitude_Position_Control.Kd=PID_Parameter[9].d;
/***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
        Total_Controller.Longitude_Speed_Control.Kp=PID_Parameter[8].p;
        Total_Controller.Longitude_Speed_Control.Ki=PID_Parameter[8].i;
        Total_Controller.Longitude_Speed_Control.Kd=PID_Parameter[8].d;

        Total_Controller.Longitude_Position_Control.Kp=PID_Parameter[9].p;
        Total_Controller.Longitude_Position_Control.Ki=PID_Parameter[9].i;
        Total_Controller.Longitude_Position_Control.Kd=PID_Parameter[9].d;

        Total_Controller.High_Acce_Control.Kp=PID_Parameter[10].p;
        Total_Controller.High_Acce_Control.Ki=PID_Parameter[10].i;
        Total_Controller.High_Acce_Control.Kd=PID_Parameter[10].d;
  }
}

