#include "Headfile.h"
#include "IIC.h"
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

void IIC_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  delay_ms(10);
  GPIO_SetBits(GPIOB,GPIO_Pin_6 | GPIO_Pin_7);

}

void SDA_IOOUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void SDA_IOIN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void I2C_delay(void)
{
    //volatile int i = 1;	//7
    //while (i)
    //    i--;
}


//**************************************
void I2C_Start(void)
{
	SDA_IOOUT();
	SCL_H;
	SDA_H;
	I2C_delay();
	SDA_L;
	I2C_delay();
}
//**************************************
void I2C_Stop(void)
{
	SDA_IOOUT();
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}

//**************************************
unsigned char I2C_SlaveAck(void)
{
	SDA_IOOUT();
	SCL_L;
	I2C_delay();
	SDA_H;

	SDA_IOIN();
	I2C_delay();
	SCL_H;

	I2C_delay();

	if(SDA_read)
	{
		SCL_L;
		return 0;
	}
	SCL_L;
	I2C_delay();
	return 1;
}

//**************************************
void I2C_SendByte(unsigned char byte)
{
   unsigned char i = 8;
   SDA_IOOUT();
   while (i--)
  {
    SCL_L;
    I2C_delay();
    if (byte & 0x80)
    SDA_H;
    else
    SDA_L;
    byte <<= 1;
    I2C_delay();
    SCL_H;
    I2C_delay();
  }
   SCL_L;
   if(I2C_SlaveAck()==0)
   {
     return ;
   }
}

//**************************************
unsigned char I2C_RecvByte(void)
{
  unsigned char i;
  unsigned char dat = 0;
	SDA_IOIN();
  for (i=0; i<8; i++)
  {
    dat <<= 1;
    SCL_H;
    I2C_delay();
    dat |= SDA_read;
    SCL_L;
    I2C_delay();
   }
  return dat;
}

//**************************************
void Single_WriteI2C(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
  I2C_Start();
  I2C_SendByte(SlaveAddress);
  I2C_SendByte(REG_Address);
  I2C_SendByte(REG_data);
  I2C_Stop();
}

//**************************************
unsigned char Single_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
	unsigned char REG_data;
	I2C_Start();
	I2C_SendByte(SlaveAddress);
	I2C_SendByte(REG_Address);
	I2C_Start();
	I2C_SendByte(SlaveAddress+1);
	REG_data=I2C_RecvByte();
	I2C_SlaveAck();
	I2C_Stop();
	return REG_data;
}

unsigned short int Double_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
	unsigned char msb , lsb ;
	msb = Single_ReadI2C(SlaveAddress,REG_Address);
	lsb = Single_ReadI2C(SlaveAddress,REG_Address+1);
	return ( ((unsigned short int)msb) << 8 | lsb) ;
}

unsigned long int Three_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
	unsigned char msb , lsb,xlsb;
	msb = Single_ReadI2C(SlaveAddress,REG_Address);
	lsb = Single_ReadI2C(SlaveAddress,REG_Address+1);
        xlsb= Single_ReadI2C(SlaveAddress,REG_Address+2);
	return (((unsigned long int)msb<< 16)|((unsigned long int)lsb<<8)|xlsb) ;
}



static void I2C_Delay_Adjust(void)
{
    volatile int i = 1;	//7
    while (i)
        i--;
}


//**************************************
void I2C_Start_Adjust(void)
{
	SDA_IOOUT();
	SCL_H;
	SDA_H;
	I2C_Delay_Adjust();
	SDA_L;
	I2C_Delay_Adjust();
}
//**************************************
void I2C_Stop_Adjust(void)
{
	SDA_IOOUT();
	SCL_L;
	I2C_Delay_Adjust();
	SDA_L;
	I2C_Delay_Adjust();
	SCL_H;
	I2C_Delay_Adjust();
	SDA_H;
	I2C_Delay_Adjust();
}

//**************************************
unsigned char I2C_SlaveAck_Adjust(void)
{
	SDA_IOOUT();
	SCL_L;
	I2C_Delay_Adjust();
	SDA_H;

	SDA_IOIN();
	I2C_Delay_Adjust();
	SCL_H;

	I2C_Delay_Adjust();

	if(SDA_read)
	{
		SCL_L;
		return 0;
	}
	SCL_L;
	I2C_Delay_Adjust();
	return 1;
}

//**************************************
void I2C_SendByte_Adjust(unsigned char byte)
{
   unsigned char i = 8;
   SDA_IOOUT();
   while (i--)
  {
    SCL_L;
    I2C_Delay_Adjust();
    if (byte & 0x80)
    SDA_H;
    else
    SDA_L;
    byte <<= 1;
    I2C_Delay_Adjust();
    SCL_H;
    I2C_Delay_Adjust();
  }
   SCL_L;
   if(I2C_SlaveAck_Adjust()==0)
   {
     return ;
   }
}

//**************************************
unsigned char I2C_RecvByte_Adjust(void)
{
  unsigned char i;
  unsigned char dat = 0;
	SDA_IOIN();
  for (i=0; i<8; i++)
  {
    dat <<= 1;
    SCL_H;
    I2C_Delay_Adjust();
    dat |= SDA_read;
    SCL_L;
    I2C_Delay_Adjust();
   }
  return dat;
}

//**************************************
void Single_WriteI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
  I2C_Start_Adjust();
  I2C_SendByte_Adjust(SlaveAddress);
  I2C_SendByte_Adjust(REG_Address);
  I2C_SendByte_Adjust(REG_data);
  I2C_Stop_Adjust();
}

//**************************************
unsigned char Single_ReadI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address)
{
	unsigned char REG_data;
	I2C_Start_Adjust();
	I2C_SendByte_Adjust(SlaveAddress);
	I2C_SendByte_Adjust(REG_Address);
	I2C_Start_Adjust();
	I2C_SendByte_Adjust(SlaveAddress+1);
	REG_data=I2C_RecvByte_Adjust();
	I2C_SlaveAck_Adjust();
	I2C_Stop_Adjust();
	return REG_data;
}

unsigned short int Double_ReadI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address)
{
	unsigned char msb , lsb ;
	msb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address);
	lsb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address+1);
	return ( ((unsigned short int)msb) << 8 | lsb) ;
}

unsigned long int Three_ReadI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address)
{
	unsigned char msb , lsb,xlsb;
	msb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address);
	lsb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address+1);
        xlsb= Single_ReadI2C_Adjust(SlaveAddress,REG_Address+2);
	return (((unsigned long int)msb<< 16)|((unsigned long int)lsb<<8)|xlsb) ;
}

