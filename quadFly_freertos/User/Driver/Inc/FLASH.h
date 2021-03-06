#ifndef __FLASH_H
#define __FLASH_H
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
	*		无名科创开源飞控 V1.1	武汉科技大学  By.YuYi
	*		CSDN博客: http://blog.csdn.net/u011992534
	*               优酷ID：NamelessCotrun无名小哥
	*               无名科创开源飞控QQ群：540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               百度贴吧:无名科创开源飞控
        *               修改日期:2017/10/30
        *               版本：V1.1
        *               版权所有，盗版必究。
        *               Copyright(C) 武汉科技大学无名科创团队 2017-2019
        *               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#define Accel_Offset_Address  0
#define Accel_Scale_Address   12
#define Mag_Offset_Address  24//磁力计矫正数据地址

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
