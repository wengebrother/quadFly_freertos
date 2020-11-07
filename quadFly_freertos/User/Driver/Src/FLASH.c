#include "Headfile.h"
#include "stm32f10x_flash.h"//flash�����ӿ��ļ����ڿ��ļ��У�������Ҫ����
#include "FLASH.h"
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

union {
float Bit32;
unsigned char Bit8[4];
}flash;

/****************************************************************
*Function:	STM32F103ϵ���ڲ�Flash��д����
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:		�ó�����ֱ�ӱ������У�ֻ������Flash��д����
****************************************************************/
//#define  STARTADDR  0x08010000                   	 //STM32F103RB �����ͺŻ������ã�δ����
#define  STARTADDR  0x0803A000//0x080350A8   2K=2048=0x800
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;      //Flash����״̬����
/****************************************************************
*Name:		ReadFlashNBtye
*Function:	���ڲ�Flash��ȡN�ֽ�����
*Input:		ReadAddress�����ݵ�ַ��ƫ�Ƶ�ַ��ReadBuf������ָ��	ReadNum����ȡ�ֽ���
*Output:	��ȡ���ֽ���
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:
****************************************************************/
int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum)
{
        int DataNum = 0;
		ReadAddress = (uint32_t)STARTADDR + ReadAddress;
        while(DataNum < ReadNum)
		{
           *(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++;
           DataNum++;
        }
        return DataNum;
}

/****************************************************************
*Name:		WriteFlashOneWord
*Function:	���ڲ�Flashд��32λ����
*Input:		WriteAddress�����ݵ�ַ��ƫ�Ƶ�ַ��WriteData��д������
*Output:	NULL
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:
****************************************************************/
void WriteFlashOneWord(uint32_t WriteAddress,uint32_t WriteData)
{
	FLASH_UnlockBank1();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASHStatus = FLASH_ErasePage(STARTADDR);
	if(FLASHStatus == FLASH_COMPLETE)
	{
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress, WriteData);    //flash.c ��API����
		//FLASHStatus = FLASH_ProgramWord(StartAddress+4, 0x56780000);//��Ҫд���������ʱ����
		//FLASHStatus = FLASH_ProgramWord(StartAddress+8, 0x87650000);//��Ҫд���������ʱ����
	}
	FLASH_LockBank1();
}


void WriteFlashHarfWord(uint32_t WriteAddress,uint16_t WriteData)
{
	FLASH_UnlockBank1();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASHStatus = FLASH_ErasePage(STARTADDR);
	if(FLASHStatus == FLASH_COMPLETE)
	{
            FLASHStatus = FLASH_ProgramHalfWord(STARTADDR + WriteAddress, WriteData);    //flash.c ��API����
	}
	FLASH_LockBank1();
}

void WriteFlashNineFloat(uint32_t WriteAddress,
                         float WriteData1,
                         float WriteData2,
                         float WriteData3,
                         float WriteData4,
                         float WriteData5,
                         float WriteData6,
                         float WriteData7,
                         float WriteData8,
                         float WriteData9)
{
        uint32_t Buf[9]={0};
        Buf[0]=*(uint32_t *)(&WriteData1);//���ڴ��������ĸ��ֽ�д�뵽Flash
        Buf[1]=*(uint32_t *)(&WriteData2);
        Buf[2]=*(uint32_t *)(&WriteData3);
        Buf[3]=*(uint32_t *)(&WriteData4);
        Buf[4]=*(uint32_t *)(&WriteData5);
        Buf[5]=*(uint32_t *)(&WriteData6);
        Buf[6]=*(uint32_t *)(&WriteData7);
        Buf[7]=*(uint32_t *)(&WriteData8);
        Buf[8]=*(uint32_t *)(&WriteData9);

        FLASH_UnlockBank1();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASHStatus = FLASH_ErasePage(STARTADDR);
	if(FLASHStatus == FLASH_COMPLETE)
	{
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress,Buf[0]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+4,Buf[1]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+8,Buf[2]);
                FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+12,Buf[3]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+16,Buf[4]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+20,Buf[5]);
                FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+24,Buf[6]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+28,Buf[7]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+32,Buf[8]);

	}
	FLASH_LockBank1();
}

uint8_t ReadFlashThreeFloat(uint32_t ReadAddress,
                         float *WriteData1,
                         float *WriteData2,
                         float *WriteData3)
{
    uint8_t buf[12];
    uint16_t i=0;
    uint8_t flag=0x00;
    ReadAddress = (uint32_t)STARTADDR + ReadAddress;
    *WriteData1=*(float *)(ReadAddress);
    *WriteData2=*(float *)(ReadAddress+4);
    *WriteData3=*(float *)(ReadAddress+8);
    FLASH_LockBank1();

    for(i=0;i<12;i++)//���ֽ�����
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


#define RC_STARTADDR 0x0803B000
volatile FLASH_Status RC_FLASHStatus = FLASH_COMPLETE;      //Flash����״̬����
void WriteFlash_RC(uint32_t WriteAddress,
                         Vector_RC *ch0,
                         Vector_RC *ch1,
                         Vector_RC *ch2,
                         Vector_RC *ch3,
                         Vector_RC *ch4,
                         Vector_RC *ch5,
                         Vector_RC *ch6,
                         Vector_RC *ch7)
{
        uint32_t Buf[16]={0};
        uint16_t i=0;
        Buf[0]=*(uint32_t *)(&ch0->min);//���ڴ��������ĸ��ֽ�д�뵽Flash
        Buf[1]=*(uint32_t *)(&ch0->max);
        Buf[2]=*(uint32_t *)(&ch1->min);
        Buf[3]=*(uint32_t *)(&ch1->max);
        Buf[4]=*(uint32_t *)(&ch2->min);
        Buf[5]=*(uint32_t *)(&ch2->max);
        Buf[6]=*(uint32_t *)(&ch3->min);
        Buf[7]=*(uint32_t *)(&ch3->max);
        Buf[8]=*(uint32_t *)(&ch4->min);
        Buf[9]=*(uint32_t *)(&ch4->max);
        Buf[10]=*(uint32_t *)(&ch5->min);
        Buf[11]=*(uint32_t *)(&ch5->max);
        Buf[12]=*(uint32_t *)(&ch6->min);
        Buf[13]=*(uint32_t *)(&ch6->max);
        Buf[14]=*(uint32_t *)(&ch7->min);
        Buf[15]=*(uint32_t *)(&ch7->max);
        FLASH_UnlockBank1();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        RC_FLASHStatus = FLASH_ErasePage(RC_STARTADDR);
	if(RC_FLASHStatus == FLASH_COMPLETE)
	{
          for(i=0;i<16;i++)
          {
             RC_FLASHStatus = FLASH_ProgramWord(RC_STARTADDR + WriteAddress+4*i,Buf[i]);
          }
	}
	FLASH_LockBank1();
}


uint8_t ReadFlash_RC(uint32_t ReadAddress,uint32_t *WriteData1,uint32_t *WriteData2)
{
    uint8_t buf[8];
    uint16_t i=0;
    uint8_t flag=0x00;
    ReadAddress = (uint32_t)RC_STARTADDR + ReadAddress;
    *WriteData1=*(uint32_t *)(ReadAddress);
    *WriteData2=*(uint32_t *)(ReadAddress+4);
    FLASH_LockBank1();

    for(i=0;i<8;i++)//���ֽ�����
    {
        *(buf+i)=*(__IO uint8_t*) ReadAddress++;
    }
    if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff))
       flag=flag|0x01;
    if((buf[4]==0xff&&buf[5]==0xff&&buf[6]==0xff&&buf[7]==0xff))
       flag=flag|0x02;
    return flag;
}


#define ESC_STARTADDR 0x0803B800
volatile FLASH_Status ESC_FLASHStatus = FLASH_COMPLETE;      //Flash����״̬����
void WriteFlash_ESC(uint32_t WriteAddress,uint32_t WriteData)
{
  FLASH_UnlockBank1();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
  ESC_FLASHStatus = FLASH_ErasePage(ESC_STARTADDR);
  if(ESC_FLASHStatus == FLASH_COMPLETE)
  {
      ESC_FLASHStatus = FLASH_ProgramWord(ESC_STARTADDR + WriteAddress, WriteData);    //flash.c ��API����
  }
  FLASH_LockBank1();
}

uint8_t ReadFlash_ESC(uint32_t ReadAddress,uint32_t *WriteData)
{
    uint8_t buf[4];
    uint16_t i=0;
    uint8_t flag=0x00;
    ReadAddress = (uint32_t)ESC_STARTADDR + ReadAddress;
    *WriteData=*(uint32_t *)(ReadAddress);
    FLASH_LockBank1();
    for(i=0;i<8;i++)//���ֽ�����
    {
        *(buf+i)=*(__IO uint8_t*) ReadAddress++;
    }
    if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff))
       flag=flag|0x01;
    return flag;
}


