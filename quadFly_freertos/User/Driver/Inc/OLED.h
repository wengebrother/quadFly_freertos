#ifndef __OLED_H
#define __OLED_H

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
	*		�����ƴ���Դ�ɿ� V1.1	�人�Ƽ���ѧ  By.YuYi
	*		CSDN����: http://blog.csdn.net/u011992534
	*               �ſ�ID��NamelessCotrun����С��
	*               �����ƴ���Դ�ɿ�QQȺ��540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               �ٶ�����:�����ƴ���Դ�ɿ�
        *               �޸�����:2017/10/30
        *               �汾��V1.1
        *               ��Ȩ���У�����ؾ���
        *               Copyright(C) �人�Ƽ���ѧ�����ƴ��Ŷ� 2017-2019
        *               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#define LCD_DCout(data)		GPIO_WriteBit(GPIOC,GPIO_Pin_0,data)//DC
#define LCD_RSTout(data)	GPIO_WriteBit(GPIOC,GPIO_Pin_1,data)//RES
#define LCD_SDAout(data)	GPIO_WriteBit(GPIOC,GPIO_Pin_2,data)//D1
#define LCD_SCLout(data)	GPIO_WriteBit(GPIOC,GPIO_Pin_3,data)//D0


void LCD_P6x8Char(unsigned char x,unsigned char  y,unsigned char ch);
void LCD_P6x8Str(unsigned char x,unsigned char  y,unsigned char ch[]);
void write_6_8_number(unsigned char x,unsigned char  y,float number);
void LCD_P8x16Char(unsigned char x,unsigned char  y,unsigned char ch);
void LCD_P8x16Str(unsigned char x,unsigned char  y,unsigned char ch[]);
void write_8_16_number(unsigned char x,unsigned char  y,float number);
void LCD_clear_L(unsigned char x,unsigned char y);
void OLED_Init(void);
void OLED_Init_Fast(void);
void Draw_Logo(void);
void LCD_CLS(void);
#endif



