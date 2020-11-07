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
Sensor_Okay_Flag Sensor_Init_Flag;
/***************************************************
������: void HardWave_Init(void)
˵��:	����Ӳ����Դ��ʼ��
���:	��
����:	��
��ע:	�ϵ�����е���
ע���ߣ�����С��
****************************************************/
void HardWave_Init(void)
{
   SystemInit();        //ϵͳʱ�ӳ�ʼ��
   delay_init(72);      //�δ���ʱ��ʼ��
   USART1_Init(115200); //�����ڳ�ʼ��
   USB_Config();        //USB���⴮�ڳ�ʼ��
   OLED_Init();         //��ʾ����ʼ��
   Bling_Init();        //ָʾ�ơ�����IO��ʼ��
   Key_Init();          //������ʼ��
   PPM_Init();          //PPMң�������ճ�ʼ��
   HC_SR04_Init();      //��������ʼ��
   PWM_Init();          //PWM��ʼ����TIM4
   /***********�˲���Ϊ����ң����ʱʹ�ã�Ⱥ�ļ���ң����DIY����***************
   SPI2_Configuration();
   NRF24L01_Init();//
   while(NRF24L01_Check())
  {
    printf("24L01 Check Failed!\r\n");
    printf("Please Check!\r\n");
    delay_ms(100);
  }
   NRF24L01_RX_Mode();
   ***********************************************************************/
   Sensor_Init_Flag.NRF24L01_Okay=1;
   QuadInit();
/*******************IMU��ʼ����ʼ*************************/
/***********MPU6050��ʼ��***************/
   IIC_GPIO_Config();           //���ģ��I2C��ʼ��
   InitMPU6050_GY86();          //MPU6050��ʼ�������ѡ����ò���Ƶ�ʡ����̵�
   delay_ms(500);
   IMU_Calibration();           //��������ƫ�궨
   Sensor_Init_Flag.MPU6050_Okay=1;
/***********HMC5883��ʼ��***************/
   delay_ms(100);
   QuadInit();
/***********������+��ѹ�Ƴ�ʼ��***************/
   delay_ms(500);

#ifdef IMU_BOARD_GY86           //GY86ģ�������ΪHMC5883L
   HMC5883L_Initial();
   Sensor_Init_Flag.Mag_Okay=1;
   QuadInit();
   Baro_init();
   Read_MS5611_Offset();
   Sensor_Init_Flag.Baro_Okay=1;
#endif
#ifdef IMU_BOARD_NC686          //NC686ģ�������ΪIST8310����ѹ��ΪSPL01_001
   IST8310_Init();
   Sensor_Init_Flag.Mag_Okay=1;
   QuadInit();
   spl0601_init();
   Sensor_Init_Flag.Baro_Okay=1;
   QuadInit();
#endif
#ifdef IMU_BOARD_NC683          //NC686ģ�������ΪIST8310����ѹ��ΪFBM320
   IST8310_Init();
   Sensor_Init_Flag.Mag_Okay=1;
   QuadInit();
   FBM320_Init();
   Sensor_Init_Flag.Baro_Okay=1;
   QuadInit();
#endif
   NCQ_Quad_Init();             //��ʼ��Ԫ����ʼ��
/*******************IMU��ʼ������*************************/
   Quad_Start_Bling();          //LED����Ԥ��ʾ
   delay_ms(500);
   Mag_LS_Init();		 //��������С���˷�������ϳ�ʼ��
   Parameter_Init();            //������������ʼ��
   Butterworth_Parameter_Init();//�˲���������ʼ��
   RC_Calibration_Trigger();
   LCD_CLS();                   //����
   Set_GPS_USART();             //�ϵ���ѯ�Զ�����GPS
   USART3_Init(921600);         //����3������ �ӹ���ʱ���ó�460800
   //USART4_Init(115200);         //����4
   SBUS_USART5_Init();         //����5��SBUS����
   PID_Paramter_Init_With_Flash();//PID��������ʼ��������ͨ������վ�޸Ĳ���//Total_PID_Init();PID��������ʼ����ֻ��ͨ�������޸Ĳ���
   ADRC_Init(&ADRC_Pitch_Controller,&ADRC_Roll_Controller);//�Կ��ſ�������ʼ��
   TIM2_Configuration_Cnt();    //TIM2�����ʱ��ʱ��
   Timer1_Configuration();      //TIM4������ȶ�ʱ��
   NVIC_Configuration();        //�ж����ȼ�����
}


/***************************************************
������: void NVIC_Configuration(void)
˵��:	�ж����ȼ�����
���:	��
����:	��
��ע:	�ϵ�����е���
ע���ߣ�����С��
****************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;//����NVIC��ʼ���ṹ��
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//���ȼ����2������μ�misc.h line80
  
//GPS���ݽ����ж�
//  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//�����ж�2
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);   
  
  //������
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);  
   
//�ɿ�ϵͳ��ʱ��
  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn ;//������ʱ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //������
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
  
  //PPM���ջ�
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
//SBUS�������� 
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn; //�жϺţ�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ���
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //��Ӧ���ȼ���
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
//�����ж�1����Ӧɽ����λ����������
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //-----NRF24L01�����ж�-----//
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//IRQ�ж�ͨ��-->NRF24L01,PB12
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//����ʽ���ȼ���
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//�����ȼ���
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ��ͨ��
//  NVIC_Init(&NVIC_InitStructure);//��ʼ��NVIC

//���ô��� 
//  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; //�жϺţ�
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ���
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //��Ӧ���ȼ���
//  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
//  NVIC_Init(&NVIC_InitStructure);
  
//�ɿ�������ȶ�ʱ��
//  NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
//  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
}


void NCQ_Init(void)
{
  ReadFlash_ESC(0,&ESC_Calibration_Flag);
  if(ESC_Calibration_Flag==1)
  {
    WriteFlash_ESC(0,0);//д������´��ϵ��ٴν���
    ESC_HardWave_Init();//ֻ��ʼ��У׼����ı�Ҫ��Դ
  }
  else
  {
    HardWave_Init();//�ɿذ��ڲ���Դ����������ʼ��
  }
}




//ϵͳ��Ӳ����ʼ��
void BSP_Init(void)
{
    
   SystemInit();        //ϵͳʱ�ӳ�ʼ��
	 delay_init(72);
	
	
	/*���ȼ����飬ֻ����һ�η��飬
	���������õ��ж϶�ͳһ��������飬
	ǧ��Ҫ�ٷ��飬��ֲ��ʱ��ע���飬��ֲ�ĺ�����ô�д��з��麯��*/
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//
   USART1_Init(115200); //�����ڳ�ʼ��
   Bling_Init();        //ָʾ�ơ�����IO��ʼ��
	 
	 OLED_Init();         //��ʾ����ʼ��
	 Key_Init();          //������ʼ��
	 QuadInit();
	/*******************IMU��ʼ����ʼ*************************/
/***********MPU6050��ʼ��***************/
   IIC_GPIO_Config();           //���ģ��I2C��ʼ��
   InitMPU6050_GY86();          //MPU6050��ʼ�������ѡ����ò���Ƶ�ʡ����̵�
   delay_ms(500);
   IMU_Calibration();           //��������ƫ�궨
   Sensor_Init_Flag.MPU6050_Okay=1;
   delay_ms(100);
   QuadInit();
/***********������+��ѹ�Ƴ�ʼ��***************/
   delay_ms(500);

	 	 
	 IST8310_Init();//������
   Sensor_Init_Flag.Mag_Okay=1;
   QuadInit();
   spl0601_init();//��ѹ��
   Sensor_Init_Flag.Baro_Okay=1;
   QuadInit();
  	
	 NCQ_Quad_Init();             //��ʼ��Ԫ����ʼ��
/*******************IMU��ʼ������*************************/
   delay_ms(500);
   Mag_LS_Init();		 //��������С���˷�������ϳ�ʼ��
   Parameter_Init();            //������������ʼ��
   Butterworth_Parameter_Init();//�˲���������ʼ��
   //RC_Calibration_Trigger();//ң����У׼
   LCD_CLS();                   //����
	 NVIC_Configuration();        //�ж����ȼ�����
	
}
