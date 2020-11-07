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
#include "Time.h"
void Timer1_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_DeInit(TIM1);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  TIM_TimeBaseStructure.TIM_Period=5000;
  TIM_TimeBaseStructure.TIM_Prescaler= (72-1);
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
  TIM_Cmd(TIM1, ENABLE);
}


Sensor_Health Sensor_Flag;
_Baro Baro_Show;

Testime Time1_Delta;
uint16_t High_Okay_flag=0;

void TIM1_UP_IRQHandler(void)//5ms刷新一次
{
 if( TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET )
 {
      Test_Period(&Time1_Delta);//系统调度时间测试器
      NRF24L01_RC();//遥控器查询接收，非中端方式
/*************加速度计、陀螺仪数字量采集***********/
      GET_MPU_DATA();//1.4ms
/*************磁力计+气压计状态机更新***********/
      Compass_Tradeoff();//决策使用外置磁力计，默认内置，使用外置时注意轴向、旋转关系
#ifdef IMU_BOARD_GY86
      MS5611_StateMachine_20ms();
#endif
#ifdef IMU_BOARD_NC686
      SPL06_001_StateMachine();
#endif
#ifdef IMU_BOARD_NC683
      FBM320_StateMachine();
#endif
 /*************姿态解算***********/
      AHRSUpdate_GraDes_TimeSync(X_w_av,Y_w_av,Z_w_av,X_g_av,Y_g_av,Z_g_av);
     //DirectionConsineMatrix(DCM_Gyro,DCM_Acc,MagN);//DCM姿态解算，参考APM
     HC_SR04_Statemachine();//超声波数据更新状态机
#if  (Optical_Enable==1)
    Optflow_Statemachine();
#endif
    
      SINS_Prepare();//得到载体相对导航系的三轴运动加速度
      if(High_Okay_flag==1)//高度惯导融合
      {
        /*
          若存在超声波时，用第二种融合方式，
          因为当观测传感器（气压计、超声波）切换时，三阶互补滤波的中间参数
          需要融合一段时间，才能收敛，而卡尔曼滤波可以直接切换
        */  
        Strapdown_INS_High();//1、三阶互补滤波惯导融合
        //Strapdown_INS_High_Kalman();//2、卡尔曼滤波惯导融合
      }
      
      if(GPS_ISR_CNT>=300&&GPS_Update_finished==1)//GPS_PVT语句更新完毕后，开始解析
      {
        GPS_PVT_Parse();//GPS串口数据帧解析
        GPS_Update_finished=0;
        Set_GPS_Home();//设置Home点
      }
      
      if(GPS_Home_Set==1)//Home点已设置
      {
        Filter_Horizontal();//水平方向惯导融合
      }
      Total_Control();//总控制器：水平位置+水平速度+姿态（角度+角速度）控制器，高度位置+高度速度+高度加速度控制器
      Control_Output();//控制量总输出 
      
      if(PPM_Isr_Cnt==100
         ||Sbus_Receive_Flag==1)//PPM接收正常才进行传感器标定检测
      {
      Accel_Calibration_Check();//加速度标定检测
      Mag_Calibration_Check();//磁力计标定检测
      ESC_Calibration_Check();//电调校准检测，进入后需要拔掉电池后进入
      }
      
      Angle_Calculate();//加速度计欧拉角计算，当没有转台平台时，可以作为姿态解算的观测角度
      Bling_Working(Bling_Mode);//状态指示灯
			
      Usb_Hid_Receive();//USB虚拟串口查询解析
      ANO_SEND_StateMachine_USE_USB();//USB虚拟串口传输，连接ANO上位机
      
      //DMA_Send_StateMachine();//DMA传输，只使用山外上位机
      //ANO_SEND_StateMachine();//DMA传输，只使用ANO地面站
      DMA_SEND_Tradeoff();//DMA传输，两路地面站机同时工作
      SBUS_Linear_Calibration();//遥控器SBUS信号查询形式解析
      TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
 }
}
