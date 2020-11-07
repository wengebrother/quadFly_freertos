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
#include "usart.h"
#include <stdio.h>
DMA_InitTypeDef DMA_InitStructure;
u16 DMA1_MEM_LEN;//保存DMA每次数据传送的长度
/*
DMA1的各通道配置这里的传输形式是固定的,这点要根据不同的情况来修改
从存储器->外设模式/8位数据宽度/存储器增量模式
DMA_CHx:DMA通道CHx      cpar:外设地址
cmar:存储器地址         cndtr:数据传输量
*/
void Quad_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
        DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
	DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr =cmar;//DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //外设作为数据传输的目的地
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器

}
void Quad_DMA1_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
        DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
	DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr =cmar;//DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //外设作为数据传输的目的地
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //DMA通道 x拥有中优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
}
void Quad_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx)//开启一次DMA传输
{
	DMA_Cmd(DMA_CHx, DISABLE );
        //关闭USART1 TX DMA1 所指示的通道
        DMA_InitStructure.DMA_BufferSize =DMA1_MEM_LEN;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
 	DMA_Cmd(DMA_CHx, ENABLE);  //使能USART1 TX DMA1 所指示的通道
}
void    Quad_DMA1_USART1_SEND(u32 SendBuff,u16 len)//DMA---USART1传输
{
	Quad_DMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,len);//DMA1通道4,外设为串口1,存储器为SendBuff,长度len.
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	Quad_DMA_Enable(DMA1_Channel4);
	//while(DMA_GetFlagStatus(DMA1_FLAG_TC4) != SET);
	//DMA_ClearFlag(DMA1_FLAG_TC4);//清除发送完成标志
}
void    Quad_DMA1_USART3_SEND(u32 SendBuff,u16 len)//DMA---USART1传输
{
	Quad_DMA1_Config(DMA1_Channel2,(u32)&USART3->DR,(u32)SendBuff,len);//DMA1通道4,外设为串口1,存储器为SendBuff,长度len.
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	Quad_DMA_Enable(DMA1_Channel2);
	//while(DMA_GetFlagStatus(DMA1_FLAG_TC4) != SET);
	//DMA_ClearFlag(DMA1_FLAG_TC4);//清除发送完成标志
}
/***************************************************
函数名: void USART1_Init(unsigned long bound)
说明:	串口1初始化
入口:	波特率
出口:	无
备注:	上电初始化，运行一次
****************************************************/



 
 
 
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE
{
	int handle;
 
};
FILE __stdout;
 
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x)
{
	x = x;
}





void USART1_Init(unsigned long bound)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = bound;//
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//stop bit is 1
	USART_InitStructure.USART_Parity = USART_Parity_No;//no parity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//no Hardware Flow Control
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//enable tx and rx
	USART_Init(USART1, &USART_InitStructure);//

	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//rx interrupt is enable
	USART_Cmd(USART1, ENABLE);

}


void UART1_Send(unsigned char tx_buf)
{
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);//这里跟分析fputc时是一样的
  USART_SendData(USART1 , tx_buf);//发送字符数组里的单个字符
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
}




void USART1_Send(unsigned char *tx_buf, int len)
{
        USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ClearITPendingBit(USART1, USART_FLAG_TXE);
	while(len--)
	{
	 USART_SendData(USART1, *tx_buf);
	 while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != 1);
	 USART_ClearFlag(USART1, USART_FLAG_TC);
	 USART_ClearITPendingBit(USART1, USART_FLAG_TXE);
	 tx_buf++;
	}

}

void USART1_Receive(unsigned char *rx_buf, int len)
{
	//rx_count = 0;
	//rx_length = len;
	//rx_address = rx_buf;
}

int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (unsigned char) ch);
	while (!(USART1->SR & USART_FLAG_TXE));
	return (ch);
}

typedef struct
{
  uint32_t Head;
  float DataBuf[8];
  uint32_t End;
}DMA_Vcan_Buff;

DMA_Vcan_Buff  Vcan_Buff;
#define DMA_SEND_PERIOD 1//4*5=20ms,周期太小不易于观察波形
uint16_t USART_Send_Cnt=19;
void DMA_Send_StateMachine(void)
{
  static uint16_t DMA_Send_Cnt=0;
  DMA_Send_Cnt++;
  if(DMA_Send_Cnt>=DMA_SEND_PERIOD)
  {
   Vcan_Buff.Head=0xfc030000;

   //1

  Vcan_Buff.DataBuf[0]=NamelessQuad.Position[_YAW];//惯导高度
  Vcan_Buff.DataBuf[1]=NamelessQuad.Speed[_YAW];//惯导速度
  Vcan_Buff.DataBuf[2]=NamelessQuad.Acceleration[_YAW];;//惯导加速度
  Vcan_Buff.DataBuf[3]=Altitude_Estimate;//气压计高度
  Vcan_Buff.DataBuf[4]=Origion_NamelessQuad.Acceleration[_YAW];
  Vcan_Buff.DataBuf[5]=SPL06_001_Filter_high;
  Vcan_Buff.DataBuf[6]=NamelessCotrun_OptFlow.x_pixel_vel;
  Vcan_Buff.DataBuf[7]=HC_SR04_Distance;

/*
  Vcan_Buff.DataBuf[0]=GPS_Vel_Div.E;//惯导高度
  Vcan_Buff.DataBuf[1]=GPS_Vel_Div.N;//惯导速度
  Vcan_Buff.DataBuf[2]=NamelessQuad.Acceleration[_PITCH];;//惯导加速度
  Vcan_Buff.DataBuf[3]=NamelessQuad.Acceleration[_ROLL];;//惯导加速度
  Vcan_Buff.DataBuf[4]=Acce_History[_PITCH][40];//惯导高度;
  Vcan_Buff.DataBuf[5]=Acce_History[_ROLL][40];
  Vcan_Buff.DataBuf[6]=0;
  Vcan_Buff.DataBuf[7]=0;

*/   
   
   //2

  //Vcan_Buff.DataBuf[0]=10*Pitch;//Pitch X
  //Vcan_Buff.DataBuf[1]=10*Roll;//Roll R
  //Vcan_Buff.DataBuf[2]=10*ACCE_X;
  //Vcan_Buff.DataBuf[3]=10*ACCE_Y;
  //Vcan_Buff.DataBuf[4]=10000*Gyro_Range_Scale;
  //Vcan_Buff.DataBuf[5]=Gyro_Range_Offset_Gain;
  //Vcan_Buff.DataBuf[6]=100*Gyro_Range_Mode;
  //Vcan_Buff.DataBuf[7]=Gyro_Length;

/*
  Vcan_Buff.DataBuf[0]=PPM_Databuf[0];
  Vcan_Buff.DataBuf[1]=PPM_Databuf[1];
  Vcan_Buff.DataBuf[2]=PPM_Databuf[2];
  Vcan_Buff.DataBuf[3]=PPM_Databuf[3];

  Vcan_Buff.DataBuf[4]=PPM_LPF_Databuf[0];
  Vcan_Buff.DataBuf[5]=PPM_LPF_Databuf[1];
  Vcan_Buff.DataBuf[6]=Total_Controller.Pitch_Gyro_Control.Dis_Err;
  Vcan_Buff.DataBuf[7]=Total_Controller.Pitch_Gyro_Control.Dis_Error_History[0];
*/
   //3
 /*
  Vcan_Buff.DataBuf[0]=NamelessQuad.Position[_PITCH];
  Vcan_Buff.DataBuf[1]=NamelessQuad.Speed[_PITCH];
  Vcan_Buff.DataBuf[2]=GPS_Vel.E;
  Vcan_Buff.DataBuf[3]=Earth_Frame_To_XYZ.E;

  Vcan_Buff.DataBuf[4]=NamelessQuad.Position[_ROLL];
  Vcan_Buff.DataBuf[5]=NamelessQuad.Speed[_ROLL];
  Vcan_Buff.DataBuf[6]=GPS_Vel.N;
  Vcan_Buff.DataBuf[7]=Earth_Frame_To_XYZ.N;

  Vcan_Buff.DataBuf[0]=NamelessQuad.Position[_PITCH];
  Vcan_Buff.DataBuf[1]=Earth_Frame_To_XYZ.E;
  Vcan_Buff.DataBuf[2]=NamelessQuad.Position[_ROLL];
  Vcan_Buff.DataBuf[3]=Earth_Frame_To_XYZ.N;
  Vcan_Buff.DataBuf[4]=GPS_Ground_Speed;
  Vcan_Buff.DataBuf[5]=sqrt(NamelessQuad.Speed[_PITCH]*NamelessQuad.Speed[_PITCH]
                            +NamelessQuad.Speed[_ROLL]*NamelessQuad.Speed[_ROLL]);
  Vcan_Buff.DataBuf[6]=Altitude_Estimate;
  Vcan_Buff.DataBuf[7]=NamelessQuad.Position[_YAW];//惯导高度
*/
  //Vcan_Buff.DataBuf[6]=GPS_Vel.N;
  //Vcan_Buff.DataBuf[7]=Earth_Frame_To_XYZ.N;
   /*
  Vcan_Buff.DataBuf[0]=GPS_Vel_Div.E;
  Vcan_Buff.DataBuf[1]=Origion_NamelessQuad.Acceleration[_PITCH];
  Vcan_Buff.DataBuf[2]=GPS_Vel.E;
  Vcan_Buff.DataBuf[3]=Acce_History[_PITCH][USART_Send_Cnt];

  Vcan_Buff.DataBuf[4]=GPS_Vel_Div.N;
  Vcan_Buff.DataBuf[5]=Origion_NamelessQuad.Acceleration[_ROLL];
  Vcan_Buff.DataBuf[6]=GPS_Vel.N;
  Vcan_Buff.DataBuf[7]=Acce_History[_ROLL][USART_Send_Cnt];
*/
/*
  Vcan_Buff.DataBuf[0]=100*Pitch;
  Vcan_Buff.DataBuf[1]=100*Roll;
  Vcan_Buff.DataBuf[2]=100*ACCE_X;
  Vcan_Buff.DataBuf[3]=100*ACCE_Y;
  Vcan_Buff.DataBuf[4]=Origion_NamelessQuad.Acceleration[_YAW];
  Vcan_Buff.DataBuf[5]=Filter_Feedback_NamelessQuad.Acceleration[_YAW];
  //Vcan_Buff.DataBuf[6]=Total_Controler.High_Position_Control.Expect;
  //Vcan_Buff.DataBuf[7]=FilterBefore_NamelessQuad.Acceleration[_YAW];
  Vcan_Buff.DataBuf[7]=Gyro_Delta_Length;
  Vcan_Buff.DataBuf[6]=Acceleration_Length;
*/
/*
  Vcan_Buff.DataBuf[0]=NamelessCotrunOptical.Position.x;
  Vcan_Buff.DataBuf[1]=NamelessCotrunOptical.Speed.x;
  Vcan_Buff.DataBuf[2]=SINS_Accel_Body.x;
  Vcan_Buff.DataBuf[3]=SINS_Accel_Body.y;
  Vcan_Buff.DataBuf[4]=OptFlow_Vel_X;
  Vcan_Buff.DataBuf[5]=OptFlow_Vel_Y;
  Vcan_Buff.DataBuf[6]=NamelessCotrun_OptFlow.x_integral;
  Vcan_Buff.DataBuf[7]=NamelessCotrun_OptFlow.y_integral;
*/

/*
  //Vcan_Buff.DataBuf[0]=Acce_Correct[0];
  //Vcan_Buff.DataBuf[1]=Acce_Correct[1];
  //Vcan_Buff.DataBuf[2]=Acce_Correct[2];
  //Vcan_Buff.DataBuf[3]=imu.accelRaw[0];
  //Vcan_Buff.DataBuf[4]=imu.accelRaw[1];
 // Vcan_Buff.DataBuf[6]=imu.accelRaw[2];
*/
/*
  Vcan_Buff.DataBuf[0]=ADRC_Roll_Controller.z2;//ADRC_Roll_Controller.x1;
  Vcan_Buff.DataBuf[1]=ADRC_Roll_Controller.x2;
  Vcan_Buff.DataBuf[2]=Total_Controler.Roll_Angle_Control.Control_OutPut;

  Vcan_Buff.DataBuf[3]=Roll_Gyro;
  Vcan_Buff.DataBuf[4]=ADRC_Roll_Controller.z1;
  Vcan_Buff.DataBuf[5]=ADRC_Roll_Controller.z2;
  Vcan_Buff.DataBuf[6]=ADRC_Roll_Controller.z3;
  Vcan_Buff.DataBuf[7]=ADRC_Roll_Controller.ESO_Input_Div;
*/
  //Vcan_Buff.DataBuf[7]=FilterBefore_NamelessQuad.Acceleration[_YAW];
  //Vcan_Buff.DataBuf[7]=Gyro_Delta_Length;
  //Vcan_Buff.DataBuf[6]=Acceleration_Length;

  Vcan_Buff.End=0x000003fc;
  Quad_DMA1_USART1_SEND((u32)(&Vcan_Buff),sizeof(Vcan_Buff));
  DMA_Send_Cnt=0;
  }
}


uint8_t Rec_Cnt=0;
uint8_t Rec_Start=0;
uint8_t RecBag[3]={0};
uint8_t Rec_Ready=0;
uint16_t Rec_Ust=0;
void USART1_IRQHandler(void)
{
	unsigned char Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Rec_Ust++;
	  Res = USART_ReceiveData(USART1);
                if(Res==0xff)
		{
                  Rec_Ready=1;
                  Rec_Cnt=0;
		}
		if(Rec_Ready==1)
		{
		  Rec_Cnt++;
		}
                if(Rec_Cnt>=3)
		{
		RecBag[Rec_Cnt-3]=Res;
		if(Rec_Cnt==5)
		{
                  Rec_Cnt=0;
                  Rec_Ready=0;
	  //UART1_Send(RecBag[0]);
	  //UART1_Send(RecBag[1]);
	  //UART1_Send(RecBag[2]);
		}
		}
          USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}



void wust_sendccd(unsigned char *ccdaddr, int16_t ccdsize)
{
    #define CMD_CCD   2
    uint8 cmdf[2] = {CMD_CCD, ~CMD_CCD};
    uint8 cmdr[2] = {~CMD_CCD, CMD_CCD};
    USART1_Send(cmdf, sizeof(cmdf));
    USART1_Send(ccdaddr, ccdsize);
    USART1_Send(cmdr, sizeof(cmdr));
}
void wust_sendware(unsigned char *wareaddr, int16_t waresize)
{
    #define CMD_WARE     3
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};
    USART1_Send(cmdf, sizeof(cmdf));
    USART1_Send(wareaddr, waresize);
    USART1_Send(cmdr, sizeof(cmdr));
}



void UART2_Send(unsigned char tx_buf)
{
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);//这里跟分析fputc时是一样的
  USART_SendData(USART2 , tx_buf);//发送字符数组里的单个字符
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
}

void USART2_Send(unsigned char *tx_buf, int len)
{
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_ClearITPendingBit(USART2, USART_FLAG_TXE);
	while(len--)
      {
	USART_SendData(USART2, *tx_buf);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != 1);
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_ClearITPendingBit(USART2, USART_FLAG_TXE);
	tx_buf++;
      }
}


unsigned char Buffer[2]={9,8};
void USART2_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO
                               |RCC_APB2Periph_GPIOA , ENABLE);//串口2

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//串口2 低速

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = bound;//
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//stop bit is 1
	USART_InitStructure.USART_Parity = USART_Parity_No;//no parity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//no Hardware Flow Control
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//enable tx and rx
	USART_Init(USART2, &USART_InitStructure);//

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//rx interrupt is enable
	USART_Cmd(USART2, ENABLE);


        //USART2_Send((unsigned char *)Buffer,2);
       //UART2_Send(0xAA);
}




void wust_sendimage(unsigned char *wareaddr, int16_t waresize)
{
    #define CMD_Image    1
    uint8 cmdf[2] = {CMD_Image, ~CMD_Image};
    uint8 cmdr[2] = {~CMD_Image, CMD_Image};
    USART1_Send(cmdf, sizeof(cmdf));
    USART1_Send(wareaddr, waresize);
    USART1_Send(cmdr, sizeof(cmdr));
}


void USART3_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;//
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//stop bit is 1
	USART_InitStructure.USART_Parity = USART_Parity_No;//no parity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//no Hardware Flow Control
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//enable tx and rx
	USART_Init(USART3, &USART_InitStructure);//
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//rx interrupt is enable
	USART_Cmd(USART3, ENABLE);
}

void USART3_Send(unsigned char tx_buf)
{
  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) != 1);
  USART_SendData(USART3, tx_buf);
  USART_ClearFlag(USART3, USART_FLAG_TC);
  USART_ClearITPendingBit(USART3, USART_FLAG_TXE);
}
void UART3_Send(unsigned char *tx_buf, int len)
{
		USART_ClearFlag(USART3, USART_FLAG_TC);
		USART_ClearITPendingBit(USART3, USART_FLAG_TXE);
	while(len--)
	{
		USART_SendData(USART3, *tx_buf);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) != 1);
		USART_ClearFlag(USART3, USART_FLAG_TC);
		USART_ClearITPendingBit(USART3, USART_FLAG_TXE);
		tx_buf++;
	}
}

void USART4_Init(unsigned long bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //UART4 TX；
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出；
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure); //端口C；

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //UART4 RX；
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入；
    GPIO_Init(GPIOC, &GPIO_InitStructure); //端口C；

    USART_InitStructure.USART_BaudRate = bound; //波特率；
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据位8位；
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //停止位1位；
    USART_InitStructure.USART_Parity = USART_Parity_No ; //无校验位；
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //无硬件流控；
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //收发模式；
    USART_Init(UART4, &USART_InitStructure);//配置串口参数；

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE); //使能串口；
}

void USART4_Send(u8 Data) //发送一个字节；
{
    USART_SendData(UART4,Data);
    while( USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET );
}


void UART4_Send(u8 *Data) //发送字符串；
{
    while(*Data)
    USART4_Send(*Data++);
}


void UART4_IRQHandler(void) //中断处理函数；
{
    u8 res;
    if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET) //判断是否发生中断；
    {
    USART_ClearFlag(UART4, USART_IT_RXNE); //清除标志位；
    res=USART_ReceiveData(UART4); //接收数据；
    USART4_Send(res); //用户自定义；
    }
}


RingBuff_t SBUS_Ringbuf;
void SBUS_USART5_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //UART5 RX；
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入；
    GPIO_Init(GPIOD, &GPIO_InitStructure); //端口D；

    USART_InitStructure.USART_BaudRate = 100000; //波特率；
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据位8位；
    USART_InitStructure.USART_StopBits = USART_StopBits_2; //停止位2位；
    USART_InitStructure.USART_Parity = USART_Parity_Even ; //偶校验位；
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控；
    USART_InitStructure.USART_Mode = USART_Mode_Rx;//发模式；
    USART_Init(UART5, &USART_InitStructure);//配置串口参数；
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART5, ENABLE); //使能串口；
    
    RingBuff_Init(&SBUS_Ringbuf);
}





void USART5_Send(u8 Data) //发送一个字节；
{
    USART_SendData(UART5,Data);
    while( USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET );
}

void UART5_Send(u8 *Data) //发送字符串；
{
    while(*Data)
    USART5_Send(*Data++);
}




void UART5_IRQHandler(void) //中断处理函数；
{
    if(USART_GetITStatus(UART5, USART_IT_RXNE) == SET) //判断是否发生中断；
    {
      RingBuf_Write(USART_ReceiveData(UART5),&SBUS_Ringbuf);//往环形队列里面写数据
      USART_ClearFlag(UART5, USART_IT_RXNE); //清除标志位；
    }
}


Testime Time_UASRT3;
uint8_t Frame_Header_IS_Okay=0;
uint8_t OpticalFlow_Data_Buffer[15];
uint8_t OpticalFlow_Data_Cnt=0;
uint8_t OpticalFlow_Data_IS_Okay=0;
void USART3_IRQHandler(void)
{
      if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
      {
        ANO_DT_Data_Receive_Prepare(USART_ReceiveData(USART3));
/*
        if(Frame_Header_IS_Okay==1)
        {
          OpticalFlow_Data_Cnt++;
          if(OpticalFlow_Data_Cnt>=1)
          {
            OpticalFlow_Data_Buffer[OpticalFlow_Data_Cnt-1]=ch3;
          }
          if(OpticalFlow_Data_Cnt>=15)
          {
            Frame_Header_IS_Okay=0;
            OpticalFlow_Data_IS_Okay=1;
            Test_Period(&Time_UASRT3);//20ms
          }
        }
        if(ch3==0xAA&&Frame_Header_IS_Okay==0)
        {
          Frame_Header_IS_Okay=1;
          OpticalFlow_Data_Cnt=0;
        }
        */
      }
      USART_ClearITPendingBit(USART3, USART_IT_RXNE);
}

/*
Testime Time_UASRT3;
uint8_t Frame_Header_IS_Okay=0;
uint8_t OpticalFlow_Data_Buffer[12];
uint8_t OpticalFlow_Data_Cnt=0;
uint8_t OpticalFlow_Data_IS_Okay=0;
void USART3_IRQHandler(void)
{
      if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
      {
        uint8_t ch3=USART_ReceiveData(USART3);
        if(Frame_Header_IS_Okay==1)
        {
          OpticalFlow_Data_Cnt++;
          if(OpticalFlow_Data_Cnt>=2)
          {
            OpticalFlow_Data_Buffer[OpticalFlow_Data_Cnt-2]=ch3;
          }
          if(OpticalFlow_Data_Cnt>=13)
          {
            Frame_Header_IS_Okay=0;
            OpticalFlow_Data_IS_Okay=1;
          }
        }
        if(ch3==0xfe&&Frame_Header_IS_Okay==0)
        {
          Frame_Header_IS_Okay=1;
          OpticalFlow_Data_Cnt=0;
        }

      }
      USART_ClearITPendingBit(USART3, USART_IT_RXNE);
}
*/


/*
float pre_LastHigh=0,LastHigh=0;
uint8 US_100_Cnt=0,US_Data[2]={0};
float US_100_Max=0,US_100_Min=0;
void USART3_IRQHandler(void)
{
  unsigned char ch;
  float tempa,tempb,tempc,max,min;             //用于均值滤波
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
  if(US_100_Start==1)
  {
    US_Data[US_100_Cnt]=USART_ReceiveData(USART3);
    US_100_Cnt++;

    if(US_100_Cnt>=2)
    {
    US_100_Cnt=0;
    US_100_Start=0;
    US_100_Finished=1;
    US_Distance=US_100_Distance(US_Data[0],US_Data[1]);//mm

    tempa=pre_LastHigh;//均值滤波，保证得到值不跳变
    tempb=LastHigh;
    tempc=US_Distance;
    max = tempa > tempb ? tempa:tempb;
    max = max > tempc ? max:tempc;
    min = tempa < tempb ? tempa:tempb;
    min = min < tempc ? min:tempc;
    if(tempa > min && tempa < max)    US_Distance = tempa;
    if(tempb > min  && tempb < max )  US_Distance = tempb;
    if(tempc > min  &&  tempc < max)  US_Distance = tempc;
    pre_LastHigh = LastHigh;//角速度递推赋值
    LastHigh = US_Distance;

    if(US_Distance>=2200)  US_Distance=2200;
    if(US_Distance<=40)  US_Distance=40;
    if(US_Distance>US_100_Max)  US_100_Max=US_Distance;

    }
  }

  USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
}
*/


/*
u8 GPS_Buf[2][100]={0};
unsigned char GPS_GPGGA_Buf[6];
unsigned int GPS_Data_Cnt=0;
unsigned GPSx_Cnt=0,GPSx_Finish_Flag=1,GPSx_Data_Cnt=0;
u16 GPS_ISR_CNT=0;
uint8_t GPxCnt=0;
uint16 GPS_Update_finished=0;
 void USART3_IRQHandler(void)
{
  unsigned char ch;
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
  GPS_ISR_CNT++;
  if(GPS_ISR_CNT>=2000)  GPS_ISR_CNT=2000;


  ch=USART_ReceiveData(USART3);
  if(ch=='$')
  {
    GPxCnt++;
    if(GPxCnt>=3)  {GPxCnt=1;GPS_Update_finished=1;}

    GPS_Buf[GPxCnt-1][99]='\0';//字符串数组末尾补‘\0’
    GPSx_Data_Cnt=0;
    GPSx_Finish_Flag=0;
  }
  else if(ch=='*')
  {
    GPSx_Finish_Flag=1;
    GPS_Buf[GPxCnt-1][GPSx_Data_Cnt++]=ch;
    GPS_Buf[GPxCnt-1][99]='\0';
  }

  if(GPSx_Data_Cnt<99&&GPSx_Finish_Flag==0)
  {
   GPS_Buf[GPxCnt-1][GPSx_Data_Cnt++]=ch;
  }


  USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
}
*/


u8 GPS_Buf[2][100]={0};
unsigned char GPS_GPGGA_Buf[6];
unsigned int GPS_Data_Cnt=0;
unsigned GPSx_Cnt=0,GPSx_Finish_Flag=1,GPSx_Data_Cnt=0;
u16 GPS_ISR_CNT=0;
uint8_t GPxCnt=0;

uint16 Ublox_Try_Cnt=0;
uint8 Ublox_Try_Buf[5]={0};
uint16 Ublox_Try_Makesure=0;
uint16 Ublox_Try_Start=0;

uint8 Ublox_Data[95]={0};
uint16 Ublox_Cnt=0;
uint16 Ublox_Receive=0;
uint16 GPS_Update_finished=0;
uint16 GPS_Update_finished_Correct_Flag=0;

Testime GPS_Time_Delta;
void USART2_IRQHandler(void)//解析GPS输出的UBLOX  PVT协议
{
  unsigned char ch;
if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
{

  if(GPS_ISR_CNT<=2000)
  {
    GPS_ISR_CNT++;
  }

  ch=USART_ReceiveData(USART2);

  if(Ublox_Try_Makesure==1)
  {
     Ublox_Data[Ublox_Cnt++]=ch;
     if(Ublox_Cnt==94)
     {
       Ublox_Cnt=0;
       Ublox_Try_Makesure=0;
       GPS_Update_finished=1;
       GPS_Update_finished_Correct_Flag=1;
       Test_Period(&GPS_Time_Delta);//GPS数据更新间隔测试
     }
  }
  
  if(Ublox_Try_Makesure==0
     &&ch==0xB5)//出现帧头首字节，判断帧头是否完整
  {
    Ublox_Try_Start=1;
    Ublox_Try_Cnt=0;
  }

  if(Ublox_Try_Start==1)
  {
    Ublox_Try_Cnt++;
    if(Ublox_Try_Cnt>=5)
    {
      Ublox_Try_Start=0;
      Ublox_Try_Cnt=0;

      if(ch==0x5C) Ublox_Try_Makesure=1;//确认为帧头，开始接受
      else Ublox_Try_Makesure=0;//非帧头，复位等待再次确认
    }
  }
 }
  USART_ClearITPendingBit(USART2, USART_IT_RXNE);

}

uint8_t data_to_send[50];
uint8_t ANO_Send_PID_Flag[6]={0};
uint8_t ANO_Send_PID_Flag_USB[6]={0};
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);

static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
    u8 sum = 0,i=0;
    data_to_send[0]=0xAA;
    data_to_send[1]=0xAA;
    data_to_send[2]=0xEF;
    data_to_send[3]=2;
    data_to_send[4]=head;
    data_to_send[5]=check_sum;
    for(i=0;i<6;i++)
        sum += data_to_send[i];
    data_to_send[6]=sum;
    Quad_DMA1_USART3_SEND((u32)(data_to_send),7);
    //USB_TxWrite(data_to_send, 7);
    //ANO_DT_Send_Data(data_to_send, 7);
}


/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data)
{
    static u8 RxBuffer[50];
    static u8 _data_len = 0,_data_cnt = 0;
    static u8 state = 0;

    if(state==0&&data==0xAA)
    {
        state=1;
        RxBuffer[0]=data;
    }
    else if(state==1&&data==0xAF)
    {
        state=2;
        RxBuffer[1]=data;
    }
    else if(state==2&&data<0XF1)
    {
        state=3;
        RxBuffer[2]=data;
    }
    else if(state==3&&data<50)
    {
        state = 4;
        RxBuffer[3]=data;
        _data_len = data;
        _data_cnt = 0;
    }
    else if(state==4&&_data_len>0)
    {
        _data_len--;
        RxBuffer[4+_data_cnt++]=data;
        if(_data_len==0)
            state = 5;
    }
    else if(state==5)
    {
        state = 0;
        RxBuffer[4+_data_cnt]=data;
        ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
    }
    else
        state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
uint16_t usb_test[3]={0};
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
    u8 sum = 0,i=0;
    for(i=0;i<(num-1);i++)
        sum += *(data_buf+i);
    if(!(sum==*(data_buf+num-1)))       {usb_test[0]++;return;    } //判断sum
    if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))     {usb_test[1]++;return;}     //判断帧头
    usb_test[2]++;
    if(*(data_buf+2)==0X01)
    {
        if(*(data_buf+4)==0X01)
            ;//mpu6050.Acc_CALIBRATE = 1;
        if(*(data_buf+4)==0X02)
            ;//mpu6050.Gyro_CALIBRATE = 1;
        if(*(data_buf+4)==0X03)
        {
            ;//mpu6050.Acc_CALIBRATE = 1;
            ;//mpu6050.Gyro_CALIBRATE = 1;
        }
    }

    if(*(data_buf+2)==0X02)
    {
        if(*(data_buf+4)==0X01)
        {
            ANO_Send_PID_Flag[0]=1;
            ANO_Send_PID_Flag[1]=1;
            ANO_Send_PID_Flag[2]=1;
            ANO_Send_PID_Flag[3]=1;
            ANO_Send_PID_Flag[4]=1;
            ANO_Send_PID_Flag[5]=1;
        }
        if(*(data_buf+4)==0X02)
        {

        }
        if(*(data_buf+4)==0XA0)     //读取版本信息
        {
            ;//f.send_version = 1;
        }
        if(*(data_buf+4)==0XA1)     //恢复默认参数
        {
            Sort_PID_Flag=2;
        }
    }

    if(*(data_buf+2)==0X10)                             //PID1
    {
        Total_Controller.Roll_Gyro_Control.Kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        Total_Controller.Roll_Gyro_Control.Ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        Total_Controller.Roll_Gyro_Control.Kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Total_Controller.Pitch_Gyro_Control.Kp   = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        Total_Controller.Pitch_Gyro_Control.Ki   = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        Total_Controller.Pitch_Gyro_Control.Kd   = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        Total_Controller.Yaw_Gyro_Control.Kp    = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        Total_Controller.Yaw_Gyro_Control.Ki    = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        Total_Controller.Yaw_Gyro_Control.Kd    = 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);


    }
    if(*(data_buf+2)==0X11)                             //PID2
    {

        Total_Controller.Roll_Angle_Control.Kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        Total_Controller.Roll_Angle_Control.Ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        Total_Controller.Roll_Angle_Control.Kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Total_Controller.Pitch_Angle_Control.Kp   = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        Total_Controller.Pitch_Angle_Control.Ki   = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        Total_Controller.Pitch_Angle_Control.Kd   = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        Total_Controller.Yaw_Angle_Control.Kp    = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        Total_Controller.Yaw_Angle_Control.Ki    = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        Total_Controller.Yaw_Angle_Control.Kd    = 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);

    }
    if(*(data_buf+2)==0X12)                             //PID3
    {
        Total_Controller.High_Speed_Control.Kp    = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        Total_Controller.High_Speed_Control.Ki    = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        Total_Controller.High_Speed_Control.Kd    = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Total_Controller.High_Position_Control.Kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        Total_Controller.High_Position_Control.Ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        Total_Controller.High_Position_Control.Kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        Total_Controller.Latitude_Speed_Control.Kp= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        Total_Controller.Latitude_Speed_Control.Ki= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        Total_Controller.Latitude_Speed_Control.Kd= 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        /***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
        Total_Controller.Longitude_Speed_Control.Kp=Total_Controller.Latitude_Speed_Control.Kp;
        Total_Controller.Longitude_Speed_Control.Ki=Total_Controller.Latitude_Speed_Control.Ki;
        Total_Controller.Longitude_Speed_Control.Kd=Total_Controller.Latitude_Speed_Control.Kd;

        ANO_DT_Send_Check(*(data_buf+2),sum);

    }
    if(*(data_buf+2)==0X13)                             //PID4
    {
        Total_Controller.Latitude_Position_Control.Kp    = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        Total_Controller.Latitude_Position_Control.Ki    = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        Total_Controller.Latitude_Position_Control.Kd    = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Total_Controller.High_Acce_Control.Kp            = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        Total_Controller.High_Acce_Control.Ki            = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        Total_Controller.High_Acce_Control.Kd            = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        /***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
        Total_Controller.Longitude_Position_Control.Kp=Total_Controller.Latitude_Position_Control.Kp;
        Total_Controller.Longitude_Position_Control.Ki=Total_Controller.Latitude_Position_Control.Ki;
        Total_Controller.Longitude_Position_Control.Kd=Total_Controller.Latitude_Position_Control.Kd;
        ANO_DT_Send_Check(*(data_buf+2),sum);

    }
    if(*(data_buf+2)==0X14)                             //PID5
    {
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
    if(*(data_buf+2)==0X15)                             //PID6
    {
        ANO_DT_Send_Check(*(data_buf+2),sum);
        Sort_PID_Cnt++;
        Sort_PID_Flag=1;
    }

}


/************************************************************/
//1：发送基本信息（姿态、锁定状态）
void ANO_Data_Send_Status(void)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;

	_temp = (int)(Roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(-att.angle[_YAW]*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp2 = (vs32)(100*NamelessQuad.Position[_YAW]);//单位cm
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

        data_to_send[_cnt++]=0x01;//飞行模式
        data_to_send[_cnt++]=Controler_State;//上锁0、解锁1

	data_to_send[3] = _cnt-4;
	sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
        Quad_DMA1_USART3_SEND((u32)(data_to_send),_cnt);
        //USB_TxWrite(data_to_send, _cnt);
        //UART3_Send(data_to_send, _cnt);
}

void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
    u8 _cnt=0;
    vs16 _temp;
    u8 sum = 0;
    u8 i=0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;

    _temp = a_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = g_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = m_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    Quad_DMA1_USART3_SEND((u32)(data_to_send),_cnt);
    //USB_TxWrite(data_to_send, _cnt);
    //UART3_Send(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
    u8 _cnt=0;
    u8 i=0;
    u8 sum = 0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x03;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=BYTE1(thr);
    data_to_send[_cnt++]=BYTE0(thr);
    data_to_send[_cnt++]=BYTE1(yaw);
    data_to_send[_cnt++]=BYTE0(yaw);
    data_to_send[_cnt++]=BYTE1(rol);
    data_to_send[_cnt++]=BYTE0(rol);
    data_to_send[_cnt++]=BYTE1(pit);
    data_to_send[_cnt++]=BYTE0(pit);
    data_to_send[_cnt++]=BYTE1(aux1);
    data_to_send[_cnt++]=BYTE0(aux1);
    data_to_send[_cnt++]=BYTE1(aux2);
    data_to_send[_cnt++]=BYTE0(aux2);
    data_to_send[_cnt++]=BYTE1(aux3);
    data_to_send[_cnt++]=BYTE0(aux3);
    data_to_send[_cnt++]=BYTE1(aux4);
    data_to_send[_cnt++]=BYTE0(aux4);
    data_to_send[_cnt++]=BYTE1(aux5);
    data_to_send[_cnt++]=BYTE0(aux5);
    data_to_send[_cnt++]=BYTE1(aux6);
    data_to_send[_cnt++]=BYTE0(aux6);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    Quad_DMA1_USART3_SEND((u32)(data_to_send),_cnt);
    //USB_TxWrite(data_to_send, _cnt);
    //UART3_Send(data_to_send, _cnt);
}

void ANO_DT_Send_GPSData(u8 Fixstate,
                          u8 GPS_Num,
                          u32 log,
                          u32 lat,
                          int16 gps_head)
{
    u8 sum = 0;
    u8 _cnt=0;
    u8 i=0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x04;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=Fixstate;
    data_to_send[_cnt++]=GPS_Num;

    data_to_send[_cnt++]=BYTE3(log);
    data_to_send[_cnt++]=BYTE2(log);
    data_to_send[_cnt++]=BYTE1(log);
    data_to_send[_cnt++]=BYTE0(log);

    data_to_send[_cnt++]=BYTE3(lat);
    data_to_send[_cnt++]=BYTE2(lat);
    data_to_send[_cnt++]=BYTE1(lat);
    data_to_send[_cnt++]=BYTE0(lat);

    data_to_send[_cnt++]=BYTE1(gps_head);
    data_to_send[_cnt++]=BYTE0(gps_head);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    Quad_DMA1_USART3_SEND((u32)(data_to_send),_cnt);
    //USB_TxWrite(data_to_send, _cnt);
    //UART3_Send(data_to_send, _cnt);
}


void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
    u8 _cnt=0;
	  u8 sum = 0,i=0;
    int16_t _temp;

    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x10+group-1;
    data_to_send[_cnt++]=0;


    _temp = (int16_t)(p1_p * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p1_i  * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p1_d  * 100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p2_p  * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p2_i  * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p2_d * 100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p3_p  * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p3_i  * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p3_d * 100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;


    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;

    Quad_DMA1_USART3_SEND((u32)(data_to_send),_cnt);
    //USB_TxWrite(data_to_send, _cnt);
}



int16_t ANO_Cnt=0;
void ANO_SEND_StateMachine()
{
 ANO_Cnt++;
 if(ANO_Cnt==1)
 {
    ANO_Data_Send_Status();
 }
 else if(ANO_Cnt==2)
 {
    ANO_DT_Send_Senser((int16_t)X_g_av,(int16_t)Y_g_av,(int16_t)Z_g_av,
                       (int16_t)X_w_av,(int16_t)Y_w_av,(int16_t)Z_w_av,
                       (int16_t)DataMag.x,(int16_t)DataMag.y,(int16_t)DataMag.z);
 }
 else if(ANO_Cnt==3)
 {
  ANO_DT_Send_RCData(PPM_Databuf[2],PPM_Databuf[3],
                     PPM_Databuf[0],PPM_Databuf[1],
                     PPM_Databuf[4],PPM_Databuf[5],
                     PPM_Databuf[6],PPM_Databuf[7],0,0);
 }
 else if(ANO_Cnt==4
         &&ANO_Send_PID_Flag[0]==0
           &&ANO_Send_PID_Flag[1]==0
             &&ANO_Send_PID_Flag[2]==0
               &&ANO_Send_PID_Flag[3]==0
                 &&ANO_Send_PID_Flag[4]==0
                   &&ANO_Send_PID_Flag[5]==0)//提前终止发送队列
 {
    ANO_DT_Send_GPSData(1,GPS_Sate_Num,Longitude_Origion,Latitude_Origion,10);
    ANO_Cnt=0;
 }
  else if(ANO_Cnt==4)
 {
    ANO_DT_Send_GPSData(1,GPS_Sate_Num,Longitude_Origion,Latitude_Origion,10);
 }
 else if(ANO_Cnt==5
         &&ANO_Send_PID_Flag[0]==1)
 {
     ANO_DT_Send_PID(1,Total_Controller.Roll_Gyro_Control.Kp,
                      Total_Controller.Roll_Gyro_Control.Ki,
                      Total_Controller.Roll_Gyro_Control.Kd,
                      Total_Controller.Pitch_Gyro_Control.Kp,
                      Total_Controller.Pitch_Gyro_Control.Ki,
                      Total_Controller.Pitch_Gyro_Control.Kd,
                      Total_Controller.Yaw_Gyro_Control.Kp,
                      Total_Controller.Yaw_Gyro_Control.Ki,
                      Total_Controller.Yaw_Gyro_Control.Kd);
     ANO_Send_PID_Flag[0]=0;
 }
  else if(ANO_Cnt==6
          &&ANO_Send_PID_Flag[1]==1)
 {
    ANO_DT_Send_PID(2,Total_Controller.Roll_Angle_Control.Kp,
                      Total_Controller.Roll_Angle_Control.Ki,
                      Total_Controller.Roll_Angle_Control.Kd,
                      Total_Controller.Pitch_Angle_Control.Kp,
                      Total_Controller.Pitch_Angle_Control.Ki,
                      Total_Controller.Pitch_Angle_Control.Kd,
                      Total_Controller.Yaw_Angle_Control.Kp,
                      Total_Controller.Yaw_Angle_Control.Ki,
                      Total_Controller.Yaw_Angle_Control.Kd);
    ANO_Send_PID_Flag[1]=0;
 }
   else if(ANO_Cnt==7
           &&ANO_Send_PID_Flag[2]==1)
 {
    ANO_DT_Send_PID(3,Total_Controller.High_Speed_Control.Kp,
                      Total_Controller.High_Speed_Control.Ki,
                      Total_Controller.High_Speed_Control.Kd,
                      Total_Controller.High_Position_Control.Kp,
                      Total_Controller.High_Position_Control.Ki,
                      Total_Controller.High_Position_Control.Kd,
                      Total_Controller.Latitude_Speed_Control.Kp,
                      Total_Controller.Latitude_Speed_Control.Ki,
                      Total_Controller.Latitude_Speed_Control.Kd);
    ANO_Send_PID_Flag[2]=0;
 }
    else if(ANO_Cnt==8
            &&ANO_Send_PID_Flag[3]==1)
 {
    ANO_DT_Send_PID(4,Total_Controller.Latitude_Position_Control.Kp,
                      Total_Controller.Latitude_Position_Control.Ki,
                      Total_Controller.Latitude_Position_Control.Kd,
                      Total_Controller.High_Acce_Control.Kp,
                      Total_Controller.High_Acce_Control.Ki,
                      Total_Controller.High_Acce_Control.Kd,
                      0,0,0);
    ANO_Send_PID_Flag[3]=0;
 }
     else if(ANO_Cnt==9
             &&ANO_Send_PID_Flag[4]==1)
 {
    ANO_DT_Send_PID(5,0,0,0,
                      0,0,0,
                      0,0,0);
    ANO_Send_PID_Flag[4]=0;
 }

  else if(ANO_Cnt==10
          &&ANO_Send_PID_Flag[5]==1)
 {
    ANO_DT_Send_PID(6,0,0,0,
                      0,0,0,
                      0,0,0);
    ANO_Send_PID_Flag[5]=0;
    ANO_Cnt=0;
 }
}


/***********************************************************************/
static void ANO_DT_Send_Check_USE_USB(u8 head, u8 check_sum)
{
    u8 sum = 0,i=0;
    data_to_send[0]=0xAA;
    data_to_send[1]=0xAA;
    data_to_send[2]=0xEF;
    data_to_send[3]=2;
    data_to_send[4]=head;
    data_to_send[5]=check_sum;
    for(i=0;i<6;i++)
        sum += data_to_send[i];
    data_to_send[6]=sum;
    USB_TxWrite(data_to_send, 7);
}


void ANO_DT_Data_Receive_Anl_USE_USB(u8 *data_buf,u8 num)
{
    u8 sum = 0,i=0;
    for(i=0;i<(num-1);i++)
        sum += *(data_buf+i);
    if(!(sum==*(data_buf+num-1)))       {usb_test[0]++;return;    } //判断sum
    if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))     {usb_test[1]++;return;}     //判断帧头
    usb_test[2]++;
    if(*(data_buf+2)==0X01)
    {
        if(*(data_buf+4)==0X01)
            ;//mpu6050.Acc_CALIBRATE = 1;
        if(*(data_buf+4)==0X02)
            ;//mpu6050.Gyro_CALIBRATE = 1;
        if(*(data_buf+4)==0X03)
        {
            ;//mpu6050.Acc_CALIBRATE = 1;
            ;//mpu6050.Gyro_CALIBRATE = 1;
        }
    }

    if(*(data_buf+2)==0X02)
    {
        if(*(data_buf+4)==0X01)
        {
            ANO_Send_PID_Flag_USB[0]=1;
            ANO_Send_PID_Flag_USB[1]=1;
            ANO_Send_PID_Flag_USB[2]=1;
            ANO_Send_PID_Flag_USB[3]=1;
            ANO_Send_PID_Flag_USB[4]=1;
            ANO_Send_PID_Flag_USB[5]=1;
        }
        if(*(data_buf+4)==0X02)
        {

        }
        if(*(data_buf+4)==0XA0)     //读取版本信息
        {
            ;//f.send_version = 1;
        }
        if(*(data_buf+4)==0XA1)     //恢复默认参数
        {
            Sort_PID_Flag=3;
        }
    }

    if(*(data_buf+2)==0X10)                             //PID1
    {
        Total_Controller.Roll_Gyro_Control.Kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        Total_Controller.Roll_Gyro_Control.Ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        Total_Controller.Roll_Gyro_Control.Kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Total_Controller.Pitch_Gyro_Control.Kp   = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        Total_Controller.Pitch_Gyro_Control.Ki   = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        Total_Controller.Pitch_Gyro_Control.Kd   = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        Total_Controller.Yaw_Gyro_Control.Kp    = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        Total_Controller.Yaw_Gyro_Control.Ki    = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        Total_Controller.Yaw_Gyro_Control.Kd    = 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check_USE_USB(*(data_buf+2),sum);


    }
    if(*(data_buf+2)==0X11)                             //PID2
    {
        Total_Controller.Roll_Angle_Control.Kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        Total_Controller.Roll_Angle_Control.Ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        Total_Controller.Roll_Angle_Control.Kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Total_Controller.Pitch_Angle_Control.Kp   = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        Total_Controller.Pitch_Angle_Control.Ki   = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        Total_Controller.Pitch_Angle_Control.Kd   = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        Total_Controller.Yaw_Angle_Control.Kp    = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        Total_Controller.Yaw_Angle_Control.Ki    = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        Total_Controller.Yaw_Angle_Control.Kd    = 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check_USE_USB(*(data_buf+2),sum);

    }
    if(*(data_buf+2)==0X12)                             //PID3
    {
        Total_Controller.High_Speed_Control.Kp    = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        Total_Controller.High_Speed_Control.Ki    = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        Total_Controller.High_Speed_Control.Kd    = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Total_Controller.High_Position_Control.Kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        Total_Controller.High_Position_Control.Ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        Total_Controller.High_Position_Control.Kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        Total_Controller.Latitude_Speed_Control.Kp= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        Total_Controller.Latitude_Speed_Control.Ki= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        Total_Controller.Latitude_Speed_Control.Kd= 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        /***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
        Total_Controller.Longitude_Speed_Control.Kp=Total_Controller.Latitude_Speed_Control.Kp;
        Total_Controller.Longitude_Speed_Control.Ki=Total_Controller.Latitude_Speed_Control.Ki;
        Total_Controller.Longitude_Speed_Control.Kd=Total_Controller.Latitude_Speed_Control.Kd;
        ANO_DT_Send_Check_USE_USB(*(data_buf+2),sum);

    }
    if(*(data_buf+2)==0X13)                             //PID4
    {
        Total_Controller.Latitude_Position_Control.Kp    = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        Total_Controller.Latitude_Position_Control.Ki    = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        Total_Controller.Latitude_Position_Control.Kd    = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Total_Controller.High_Acce_Control.Kp            = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        Total_Controller.High_Acce_Control.Ki            = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        Total_Controller.High_Acce_Control.Kd            = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        /***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
        Total_Controller.Longitude_Position_Control.Kp=Total_Controller.Latitude_Position_Control.Kp;
        Total_Controller.Longitude_Position_Control.Ki=Total_Controller.Latitude_Position_Control.Ki;
        Total_Controller.Longitude_Position_Control.Kd=Total_Controller.Latitude_Position_Control.Kd;
        ANO_DT_Send_Check_USE_USB(*(data_buf+2),sum);

    }
    if(*(data_buf+2)==0X14)                             //PID5
    {
        ANO_DT_Send_Check_USE_USB(*(data_buf+2),sum);
    }
    if(*(data_buf+2)==0X15)                             //PID6
    {
        ANO_DT_Send_Check_USE_USB(*(data_buf+2),sum);
        Sort_PID_Cnt++;
        Sort_PID_Flag=1;
    }
}


//1：发送基本信息（姿态、锁定状态）
void ANO_Data_Send_Status_USE_USB(void)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;

	_temp = (int)(Roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(-att.angle[_YAW]*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp2 = (vs32)(100*NamelessQuad.Position[_YAW]);//单位cm
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

        data_to_send[_cnt++]=0x01;//飞行模式
        data_to_send[_cnt++]=Controler_State;//上锁0、解锁1

	data_to_send[3] = _cnt-4;
	sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
        USB_TxWrite(data_to_send, _cnt);
        //UART3_Send(data_to_send, _cnt);
}

void ANO_DT_Send_Senser_USE_USB(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
    u8 _cnt=0;
    vs16 _temp;
    u8 sum = 0;
    u8 i=0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;

    _temp = a_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = g_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = m_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    USB_TxWrite(data_to_send, _cnt);
    //UART3_Send(data_to_send, _cnt);
}
void ANO_DT_Send_RCData_USE_USB(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
    u8 _cnt=0;
    u8 i=0;
    u8 sum = 0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x03;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=BYTE1(thr);
    data_to_send[_cnt++]=BYTE0(thr);
    data_to_send[_cnt++]=BYTE1(yaw);
    data_to_send[_cnt++]=BYTE0(yaw);
    data_to_send[_cnt++]=BYTE1(rol);
    data_to_send[_cnt++]=BYTE0(rol);
    data_to_send[_cnt++]=BYTE1(pit);
    data_to_send[_cnt++]=BYTE0(pit);
    data_to_send[_cnt++]=BYTE1(aux1);
    data_to_send[_cnt++]=BYTE0(aux1);
    data_to_send[_cnt++]=BYTE1(aux2);
    data_to_send[_cnt++]=BYTE0(aux2);
    data_to_send[_cnt++]=BYTE1(aux3);
    data_to_send[_cnt++]=BYTE0(aux3);
    data_to_send[_cnt++]=BYTE1(aux4);
    data_to_send[_cnt++]=BYTE0(aux4);
    data_to_send[_cnt++]=BYTE1(aux5);
    data_to_send[_cnt++]=BYTE0(aux5);
    data_to_send[_cnt++]=BYTE1(aux6);
    data_to_send[_cnt++]=BYTE0(aux6);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    USB_TxWrite(data_to_send, _cnt);
    //UART3_Send(data_to_send, _cnt);
}

void ANO_DT_Send_GPSData_USE_USB(u8 Fixstate,
                          u8 GPS_Num,
                          u32 log,
                          u32 lat,
                          int16 gps_head)
{
    u8 sum = 0;
    u8 _cnt=0;
    u8 i=0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x04;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=Fixstate;
    data_to_send[_cnt++]=GPS_Num;

    data_to_send[_cnt++]=BYTE3(log);
    data_to_send[_cnt++]=BYTE2(log);
    data_to_send[_cnt++]=BYTE1(log);
    data_to_send[_cnt++]=BYTE0(log);

    data_to_send[_cnt++]=BYTE3(lat);
    data_to_send[_cnt++]=BYTE2(lat);
    data_to_send[_cnt++]=BYTE1(lat);
    data_to_send[_cnt++]=BYTE0(lat);

    data_to_send[_cnt++]=BYTE1(gps_head);
    data_to_send[_cnt++]=BYTE0(gps_head);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    USB_TxWrite(data_to_send, _cnt);
}


void ANO_DT_Send_PID_USE_USB(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
    u8 _cnt=0;
	  u8 sum = 0,i=0;
    int16_t _temp;

    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x10+group-1;
    data_to_send[_cnt++]=0;


    _temp = (int16_t)(p1_p * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p1_i  * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p1_d  * 100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p2_p  * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p2_i  * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p2_d * 100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p3_p  * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p3_i  * 1000);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int16_t)(p3_d * 100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;


    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    USB_TxWrite(data_to_send, _cnt);
}

int16_t ANO_Cnt_USE_USB=0;
void ANO_SEND_StateMachine_USE_USB()
{
 ANO_Cnt_USE_USB++;
 if(ANO_Cnt_USE_USB==1)
 {
    ANO_Data_Send_Status_USE_USB();
 }
 else if(ANO_Cnt_USE_USB==2)
 {
    ANO_DT_Send_Senser_USE_USB((int16_t)X_g_av,(int16_t)Y_g_av,(int16_t)Z_g_av,
                       (int16_t)X_w_av,(int16_t)Y_w_av,(int16_t)Z_w_av,
                       (int16_t)DataMag.x,(int16_t)DataMag.y,(int16_t)DataMag.z);
 }
 else if(ANO_Cnt_USE_USB==3)
 {
  ANO_DT_Send_RCData_USE_USB(PPM_Databuf[2],PPM_Databuf[3],
                     PPM_Databuf[0],PPM_Databuf[1],
                     PPM_Databuf[4],PPM_Databuf[5],
                     PPM_Databuf[6],PPM_Databuf[7],0,0);
 }
 else if(ANO_Cnt_USE_USB==4
         &&ANO_Send_PID_Flag_USB[0]==0
           &&ANO_Send_PID_Flag_USB[1]==0
             &&ANO_Send_PID_Flag_USB[2]==0
               &&ANO_Send_PID_Flag_USB[3]==0
                 &&ANO_Send_PID_Flag_USB[4]==0
                   &&ANO_Send_PID_Flag_USB[5]==0)//提前终止发送队列
 {
    ANO_DT_Send_GPSData_USE_USB(1,GPS_Sate_Num,Longitude_Origion,Latitude_Origion,10);
    ANO_Cnt_USE_USB=0;
 }
  else if(ANO_Cnt_USE_USB==4)
 {
    ANO_DT_Send_GPSData_USE_USB(1,GPS_Sate_Num,Longitude_Origion,Latitude_Origion,10);
 }
 else if(ANO_Cnt_USE_USB==5&&ANO_Send_PID_Flag_USB[0]==1)
 {
     ANO_DT_Send_PID_USE_USB(1,Total_Controller.Roll_Gyro_Control.Kp,
                      Total_Controller.Roll_Gyro_Control.Ki,
                      Total_Controller.Roll_Gyro_Control.Kd,
                      Total_Controller.Pitch_Gyro_Control.Kp,
                      Total_Controller.Pitch_Gyro_Control.Ki,
                      Total_Controller.Pitch_Gyro_Control.Kd,
                      Total_Controller.Yaw_Gyro_Control.Kp,
                      Total_Controller.Yaw_Gyro_Control.Ki,
                      Total_Controller.Yaw_Gyro_Control.Kd);
     ANO_Send_PID_Flag_USB[0]=0;
 }
  else if(ANO_Cnt_USE_USB==6&&ANO_Send_PID_Flag_USB[1]==1)
 {
    ANO_DT_Send_PID_USE_USB(2,Total_Controller.Roll_Angle_Control.Kp,
                      Total_Controller.Roll_Angle_Control.Ki,
                      Total_Controller.Roll_Angle_Control.Kd,
                      Total_Controller.Pitch_Angle_Control.Kp,
                      Total_Controller.Pitch_Angle_Control.Ki,
                      Total_Controller.Pitch_Angle_Control.Kd,
                      Total_Controller.Yaw_Angle_Control.Kp,
                      Total_Controller.Yaw_Angle_Control.Ki,
                      Total_Controller.Yaw_Angle_Control.Kd);
    ANO_Send_PID_Flag_USB[1]=0;
 }
   else if(ANO_Cnt_USE_USB==7&&ANO_Send_PID_Flag_USB[2]==1)
 {
    ANO_DT_Send_PID_USE_USB(3,Total_Controller.High_Speed_Control.Kp,
                      Total_Controller.High_Speed_Control.Ki,
                      Total_Controller.High_Speed_Control.Kd,
                      Total_Controller.High_Position_Control.Kp,
                      Total_Controller.High_Position_Control.Ki,
                      Total_Controller.High_Position_Control.Kd,
                      Total_Controller.Latitude_Speed_Control.Kp,
                      Total_Controller.Latitude_Speed_Control.Ki,
                      Total_Controller.Latitude_Speed_Control.Kd);
    ANO_Send_PID_Flag_USB[2]=0;
 }
    else if(ANO_Cnt_USE_USB==8&&ANO_Send_PID_Flag_USB[3]==1)
 {
    ANO_DT_Send_PID_USE_USB(4,Total_Controller.Latitude_Position_Control.Kp,
                      Total_Controller.Latitude_Position_Control.Ki,
                      Total_Controller.Latitude_Position_Control.Kd,
                      Total_Controller.High_Acce_Control.Kp,
                      Total_Controller.High_Acce_Control.Ki,
                      Total_Controller.High_Acce_Control.Kd,
                      0,0,0);
    ANO_Send_PID_Flag_USB[3]=0;
 }
     else if(ANO_Cnt_USE_USB==9&&ANO_Send_PID_Flag_USB[4]==1)
 {
    ANO_DT_Send_PID_USE_USB(5,0,0,0,
                      0,0,0,
                      0,0,0);
    ANO_Send_PID_Flag_USB[4]=0;
 }
  else if(ANO_Cnt_USE_USB==10&&ANO_Send_PID_Flag_USB[5]==1)
 {
    ANO_DT_Send_PID_USE_USB(6,0,0,0,
                      0,0,0,
                      0,0,0);
    ANO_Send_PID_Flag_USB[5]=0;
    ANO_Cnt_USE_USB=0;
 }
}


uint16_t DMA_SEND_CNT=0;
//由于共用同一个DMA，不能同时发否则会有一个丢帧
void DMA_SEND_Tradeoff(void)
{
     DMA_SEND_CNT++;
     if(DMA_SEND_CNT<=16&&DMA_SEND_CNT>=4)
     {
         DMA_Send_StateMachine();//DMA传输
     }
     else if(DMA_SEND_CNT==20)//100ms进入一次
     {
         ANO_SEND_StateMachine();
         DMA_SEND_CNT=0;
     }
}


