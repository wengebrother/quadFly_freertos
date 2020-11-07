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

void TIM1_UP_IRQHandler(void)//5msˢ��һ��
{
 if( TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET )
 {
      Test_Period(&Time1_Delta);//ϵͳ����ʱ�������
      NRF24L01_RC();//ң������ѯ���գ����ж˷�ʽ
/*************���ٶȼơ��������������ɼ�***********/
      GET_MPU_DATA();//1.4ms
/*************������+��ѹ��״̬������***********/
      Compass_Tradeoff();//����ʹ�����ô����ƣ�Ĭ�����ã�ʹ������ʱע��������ת��ϵ
#ifdef IMU_BOARD_GY86
      MS5611_StateMachine_20ms();
#endif
#ifdef IMU_BOARD_NC686
      SPL06_001_StateMachine();
#endif
#ifdef IMU_BOARD_NC683
      FBM320_StateMachine();
#endif
 /*************��̬����***********/
      AHRSUpdate_GraDes_TimeSync(X_w_av,Y_w_av,Z_w_av,X_g_av,Y_g_av,Z_g_av);
     //DirectionConsineMatrix(DCM_Gyro,DCM_Acc,MagN);//DCM��̬���㣬�ο�APM
     HC_SR04_Statemachine();//���������ݸ���״̬��
#if  (Optical_Enable==1)
    Optflow_Statemachine();
#endif
    
      SINS_Prepare();//�õ�������Ե���ϵ�������˶����ٶ�
      if(High_Okay_flag==1)//�߶ȹߵ��ں�
      {
        /*
          �����ڳ�����ʱ���õڶ����ںϷ�ʽ��
          ��Ϊ���۲⴫��������ѹ�ơ����������л�ʱ�����׻����˲����м����
          ��Ҫ�ں�һ��ʱ�䣬�������������������˲�����ֱ���л�
        */  
        Strapdown_INS_High();//1�����׻����˲��ߵ��ں�
        //Strapdown_INS_High_Kalman();//2���������˲��ߵ��ں�
      }
      
      if(GPS_ISR_CNT>=300&&GPS_Update_finished==1)//GPS_PVT��������Ϻ󣬿�ʼ����
      {
        GPS_PVT_Parse();//GPS��������֡����
        GPS_Update_finished=0;
        Set_GPS_Home();//����Home��
      }
      
      if(GPS_Home_Set==1)//Home��������
      {
        Filter_Horizontal();//ˮƽ����ߵ��ں�
      }
      Total_Control();//�ܿ�������ˮƽλ��+ˮƽ�ٶ�+��̬���Ƕ�+���ٶȣ����������߶�λ��+�߶��ٶ�+�߶ȼ��ٶȿ�����
      Control_Output();//����������� 
      
      if(PPM_Isr_Cnt==100
         ||Sbus_Receive_Flag==1)//PPM���������Ž��д������궨���
      {
      Accel_Calibration_Check();//���ٶȱ궨���
      Mag_Calibration_Check();//�����Ʊ궨���
      ESC_Calibration_Check();//���У׼��⣬�������Ҫ�ε���غ����
      }
      
      Angle_Calculate();//���ٶȼ�ŷ���Ǽ��㣬��û��ת̨ƽ̨ʱ��������Ϊ��̬����Ĺ۲�Ƕ�
      Bling_Working(Bling_Mode);//״ָ̬ʾ��
			
      Usb_Hid_Receive();//USB���⴮�ڲ�ѯ����
      ANO_SEND_StateMachine_USE_USB();//USB���⴮�ڴ��䣬����ANO��λ��
      
      //DMA_Send_StateMachine();//DMA���䣬ֻʹ��ɽ����λ��
      //ANO_SEND_StateMachine();//DMA���䣬ֻʹ��ANO����վ
      DMA_SEND_Tradeoff();//DMA���䣬��·����վ��ͬʱ����
      SBUS_Linear_Calibration();//ң����SBUS�źŲ�ѯ��ʽ����
      TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
 }
}
