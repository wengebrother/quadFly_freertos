#include "Headfile.h"
#include "ringbuf.h"

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


/**
* @brief  RingBuff_Init
* @param  void
* @return void
* @author �ܽ�
* @date   2018
* @version v1.0
* @note   ��ʼ�����λ�����
*/
void RingBuff_Init(RingBuff_t *ringBuff)
{
   //��ʼ�������Ϣ
   ringBuff->Head = 0;
   ringBuff->Tail = 0;
   ringBuff->Lenght = 0;
}

/**
* @brief  Write_RingBuff
* @param  u8 data
* @return FLASE:���λ�����������д��ʧ��;TRUE:д��ɹ�
* @author �ܽ�
* @date   2018
* @version v1.0
* @note   �����λ�����д��u8���͵�����
*/
uint8_t Write_RingBuff(uint8_t data,RingBuff_t *ringBuff)
{
   if(ringBuff->Lenght >= RINGBUFF_LEN) //�жϻ������Ƿ�����
    {
      return 0;
    }
    ringBuff->Ring_Buff[ringBuff->Tail]=data;
    //ringBuff.Tail++;
    ringBuff->Tail = (ringBuff->Tail+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
    ringBuff->Lenght++;
    return 1;
}

/**
* @brief  Read_RingBuff
* @param  u8 *rData�����ڱ����ȡ������
* @return FLASE:���λ�����û�����ݣ���ȡʧ��;TRUE:��ȡ�ɹ�
* @author �ܽ�
* @date   2018
* @version v1.0
* @note   �ӻ��λ�������ȡһ��u8���͵�����
*/
uint8_t Read_RingBuff(uint8_t *rData,RingBuff_t *ringBuff)
{
   if(ringBuff->Lenght == 0)//�жϷǿ�
    {
       return 0;
    }
   *rData = ringBuff->Ring_Buff[ringBuff->Head];//�Ƚ��ȳ�FIFO���ӻ�����ͷ��
   //ringBuff.Head++;
   ringBuff->Head = (ringBuff->Head+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
   ringBuff->Lenght--;
   return 1;
}



void RingBuf_Write(unsigned char data,RingBuff_t *ringBuff)
{
  ringBuff->Ring_Buff[ringBuff->Tail]=data;//��β��׷��
  if(++ringBuff->Tail>=RINGBUFF_LEN)//β�ڵ�ƫ��
    ringBuff->Tail=0;//����������󳤶� ���� �γɻ��ζ���  
  if(ringBuff->Tail==ringBuff->Head)//���β���ڵ�׷��ͷ���ڵ㣬���޸�ͷ�ڵ�ƫ��λ�ö�����������
  {
    if((++ringBuff->Head)>=RINGBUFF_LEN)  
    ringBuff->Head=0; 
  }
}

uint8_t RingBuf_Read(unsigned char* pData,RingBuff_t *ringBuff)
{
  if(ringBuff->Head==ringBuff->Tail)  return 1;//���ͷβ�Ӵ���ʾ������Ϊ��
  else 
  {  
    *pData=ringBuff->Ring_Buff[ringBuff->Head];//����������ǿ���ȡͷ�ڵ�ֵ��ƫ��ͷ�ڵ�
    if((++ringBuff->Head)>=RINGBUFF_LEN)   ringBuff->Head=0;
    return 0;//����0����ʾ��ȡ���ݳɹ�
  }
}

