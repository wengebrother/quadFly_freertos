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
#include "US_100.h"
float US_Distance=0;
float US_100_Distance(uint8 MSB,uint8 LSB)
{
  return (256*(MSB)+(LSB))/1.0;
}

float US_100_Temperature(uint8 data)
{
  return (data-45)/1.0;
}


u32 Test_Cnt1[2]={0},Test_Cnt2[2]={0},Test_Delta=0;
uint8 HC_SR04_StartFlag=0;
float HC_SR04_Distance=0;
uint16 HC_SR04_RSSI=1;
uint16 Sample_Cnt=0;
void HC_SR04_Start(void)
{
   HC_SR04_RSSI--;
   if(HC_SR04_RSSI<=254&&HC_SR04_StartFlag==1)//ͨѶ�쳣���ж���ռ��ʱ����·�Ӵ�������
   {
     HC_SR04_StartFlag=0;
     Sample_Cnt=0;
     HC_SR04_OUT_LOW;//��ǿ������һ��ʱ��
     //EXTI->IMR &=~EXTI_Line1;//�ر��ⲿ�ж�
     EXTI_ClearITPendingBit(EXTI_Line1);
     Delay_Us(10);
   }
   else//��������
   {   
      if(Test_Delta<=12000)  //�޷�Լ200cm
      {
        HC_SR04_Distance=Test_Delta*(340)/20000.0;
      }
      if(HC_SR04_Distance<=150&&HC_SR04_Distance>0)  Sensor_Flag.Hcsr04_Health=1;
      else  Sensor_Flag.Hcsr04_Health=0; 
   }
   
   if(HC_SR04_StartFlag==0)
  {
    HC_SR04_OUT_HIGH;
    Delay_Us(20);//������10us,�����ʱʱ��Լ����15us
    HC_SR04_OUT_LOW;
    HC_SR04_StartFlag=1;
    Sample_Cnt=0;
    HC_SR04_UP();
    HC_SR04_RSSI=255;
  }
}

void HC_SR04_Init(void)
{
      GPIO_InitTypeDef  GPIO_InitStructure;
      //EXTI_InitTypeDef EXTI_InitStructure;
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
      GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�������
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
     
      GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//��������
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      //GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
      //EXTI_InitStructure.EXTI_Line = EXTI_Line1;
      //EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
      //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
      //EXTI_InitStructure.EXTI_LineCmd = ENABLE;
      //EXTI_Init(&EXTI_InitStructure);
      //EXTI->IMR &=~EXTI_Line1;//�ر��ⲿ�ж�
      
      GPIO_SetBits(GPIOB,GPIO_Pin_9);//��ʼ����
      delay_ms(10);//��ʼ��������һ��ʱ��
      HC_SR04_StartFlag=0;
      Sample_Cnt=0;     
}

void HC_SR04_UP()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//��������
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void HC_SR04_DN()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}



void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {   
    if(Sample_Cnt==0)//��������
    {
      //Test_Cnt1=10000*TIME_ISR_CNT+TIM2->CNT/2;
      Test_Cnt1[0]=TIME_ISR_CNT;//10ms
      Test_Cnt1[1]=TIM2->CNT;//us
      HC_SR04_DN();
      Sample_Cnt++;
    }
    else if(Sample_Cnt==1)//���½���
    {
      //Test_Cnt2=10000*TIME_ISR_CNT+TIM2->CNT/2;
      Test_Cnt2[0]=TIME_ISR_CNT;//10ms
      Test_Cnt2[1]=TIM2->CNT;//us
      HC_SR04_StartFlag=0;
      Test_Delta=10000*(Test_Cnt2[0]-Test_Cnt1[0])+(Test_Cnt2[1]-Test_Cnt1[1]);//us
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}


uint8 HC_SR04_Cnt=0;
void HC_SR04_Statemachine(void)
{
  HC_SR04_Cnt++;
  if(HC_SR04_Cnt>=20)//100ms
  {
  HC_SR04_Start();//�������������������ⲿ�ж����洦��
  HC_SR04_Cnt=0;
  }
}

