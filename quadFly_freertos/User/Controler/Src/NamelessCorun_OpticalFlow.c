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
#include "NamelessCorun_OpticalFlow.h"


float set_lpf_alpha_optflow(int16_t cutoff_frequency, float time_step)
{
    // calculate alpha
    float lpf_alpha;
    float rc = 1/(2*PI*cutoff_frequency);
    lpf_alpha = time_step / (time_step + rc);
    return lpf_alpha;
}


OpticalFlow_Data NamelessCotrun_OptFlow;
float OptFlow_Vel_X_Origion,OptFlow_Vel_Y_Origion;
float OptFlow_Vel_X,OptFlow_Vel_Y;
float Last_OptFlow_Vel_X,Last_OptFlow_Vel_Y;
void Optflow_Prase()//50hz
{
    if(OpticalFlow_Data_IS_Okay==1)
    {
       NamelessCotrun_OptFlow.x_pixel_vel=(int16_t)((int16_t)(OpticalFlow_Data_Buffer[3]<<8)|OpticalFlow_Data_Buffer[4]);
       NamelessCotrun_OptFlow.y_pixel_vel=(int16_t)((int16_t)(OpticalFlow_Data_Buffer[5]<<8)|OpticalFlow_Data_Buffer[6]);
       NamelessCotrun_OptFlow.x_integral=(int16_t)((int16_t)(OpticalFlow_Data_Buffer[7]<<8)|OpticalFlow_Data_Buffer[8]);
       NamelessCotrun_OptFlow.y_integral=(int16_t)((int16_t)(OpticalFlow_Data_Buffer[9]<<8)|OpticalFlow_Data_Buffer[10]);
       NamelessCotrun_OptFlow.ground_distance=(int16_t)((int16_t)(OpticalFlow_Data_Buffer[11]<<8)|OpticalFlow_Data_Buffer[12]);
       NamelessCotrun_OptFlow.quality=(uint8_t)(OpticalFlow_Data_Buffer[13]);
       OpticalFlow_Data_IS_Okay=0;
    }
}


float Optflow_Parse_Cnt=0;
Optical_INS NamelessCotrunOptical;
Testime Opt_Delta;
float Opt_Dt=0;
void Optflow_Statemachine()
{
    Optflow_Prase();
    //Test_Period(&Opt_Delta);
    //Opt_Dt=Opt_Delta.Time_Delta/1000.0;

    //OptFlow_Vel_X_Origion=NamelessCotrun_OptFlow.x_integral;
    //OptFlow_Vel_Y_Origion=NamelessCotrun_OptFlow.y_integral;

    //OptFlow_Vel_X=Last_OptFlow_Vel_X+set_lpf_alpha_optflow(1,0.04)*(OptFlow_Vel_X_Origion-Last_OptFlow_Vel_X);
    //OptFlow_Vel_Y=Last_OptFlow_Vel_Y+set_lpf_alpha_optflow(1,0.04)*(OptFlow_Vel_Y_Origion-Last_OptFlow_Vel_Y);


}
