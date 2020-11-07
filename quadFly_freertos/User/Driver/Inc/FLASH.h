#ifndef __FLASH_H
#define __FLASH_H
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
#define Accel_Offset_Address  0
#define Accel_Scale_Address   12
#define Mag_Offset_Address  24//�����ƽ������ݵ�ַ

int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum);
void WriteFlashOneWord(uint32_t WriteAddress,uint32_t WriteData);
void WriteFlashHarfWord(uint32_t WriteAddress,uint16_t WriteData);

void WriteFlashNineFloat(uint32_t WriteAddress,
                         float WriteData1,
                         float WriteData2,
                         float WriteData3,
                         float WriteData4,
                         float WriteData5,
                         float WriteData6,
                         float WriteData7,
                         float WriteData8,
                         float WriteData9);

uint8_t ReadFlashThreeFloat(uint32_t WriteAddress,
                         float *WriteData1,
                         float *WriteData2,
                         float *WriteData3);


uint8_t ReadFlash_RC(uint32_t ReadAddress,uint32_t *WriteData1,uint32_t *WriteData2);

void WriteFlash_RC(uint32_t WriteAddress,
                         Vector_RC *ch0,
                         Vector_RC *ch1,
                         Vector_RC *ch2,
                         Vector_RC *ch3,
                         Vector_RC *ch4,
                         Vector_RC *ch5,
                         Vector_RC *ch6,
                         Vector_RC *ch7);


void WriteFlash_ESC(uint32_t WriteAddress,uint32_t WriteData);
uint8_t ReadFlash_ESC(uint32_t ReadAddress,uint32_t *WriteData);

#endif
