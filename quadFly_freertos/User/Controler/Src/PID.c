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
#include "PID.h"
/*
1ƫ���޷���־��  2�����޷���־��3���ַ����־��   4������
5����            6ƫ�        7�ϴ�ƫ�       8ƫ���޷�ֵ��
9���ַ���ƫ��ֵ��10����ֵ       11�����޷�ֵ��    12���Ʋ���Kp��
13���Ʋ���Ki��   14���Ʋ���Kd�� 15�������������  16�ϴο����������
17������޷���
*/
const float Control_Unit[15][17]=
{
/*                                         Kp        Ki        Kd            */
 /*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17*/
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,30  ,0  ,0 , 20,   4.50   ,0.0000  ,0.00  ,0  ,0 , 300},//Pitch_Angle;ƫ���Ƕ�
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 , 200,  0.95   ,3.5000  ,10.0  ,0  ,0 , 500},//Pitch_Gyro;ƫ�����ٶ�
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,30  ,0  ,0 , 20,   4.50   ,0.0000  ,0.00  ,0  ,0 , 300},//Roll_Angle;�����
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 , 200,  0.95   ,3.5000  ,10.0  ,0  ,0 , 500},//Roll_Gyro;������ٶ�
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,45  ,0  ,0 , 150 , 5.00   ,0.0000  ,0.00  ,0  ,0 , 300},//Yaw_Angle;ƫ����
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,250 ,0  ,0 , 100,  1.50   ,1.0000  ,4.50  ,0  ,0 , 300},//Yaw_Gyro;ƫ�����ٶ�

 //���߲���
 //�߶ȵ���������ƣ���ƫ���޷����������Ϊ����������½��ٶ�400cm/s
 //Z���ٶȱ���+���ֿ��ƣ���ƫ���޷�
#if (YAW_Pos_Control_Accel_Disable==1)
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 ,50 ,   1.0     ,0.000   ,0    ,0   ,0 ,400},//High_Position;���θ߶�λ��
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,400 ,0  ,0 ,400 ,  2.0     ,10.00   ,0.15  ,0  ,0 ,600},//High_Speed;���������ٶ�
#else
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 ,100 ,  1.0     ,0.000   ,0    ,0  ,0 ,400},//High_Position;���θ߶�λ��
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,400 ,0  ,0 ,500 ,  3.0     ,0.200   ,0.1  ,0  ,0 ,500},//High_Speed;���������ٶ�
#endif
 /*
1ƫ���޷���־��  2�����޷���־��3���ַ����־��   4������
5����            6ƫ�        7�ϴ�ƫ�       8ƫ���޷�ֵ��
9���ַ���ƫ��ֵ��10����ֵ       11�����޷�ֵ��    12���Ʋ���Kp��
13���Ʋ���Ki��   14���Ʋ���Kd�� 15�������������  16�ϴο����������
17������޷���
*/
/*                                       Kp        Ki        Kd            */
 /*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17*/
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,180  ,0 ,0 ,8,   0.150    ,0.000    ,0    ,0    ,0 ,150},//Longitude_Position;ˮƽ����λ��
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,300 ,0  ,0 ,15,  0.080    ,0.030    ,0    ,0    ,0 ,25},//Longitude_Speed;ˮƽ�����ٶ�
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,180 ,0  ,0 ,8,   0.150    ,0.000    ,0    ,0    ,0 ,150},//Latitude_Position;ˮƽγ��λ��
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,300 ,0  ,0 ,15,  0.080    ,0.030    ,0    ,0    ,0 ,25},//Latitude_Speed;ˮƽγ���ٶ�
  /*************���ٶȿ�����****************/
 //���������ٶ�500cm/s^2
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,600  ,0  ,0 ,500,0.12    ,0.4000    ,0.0  ,0   ,0 ,600},//��ֱ���ٶȿ�����
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0  ,0 ,3,  0.32    ,0.0000    ,0    ,0   ,0 ,150},//ˮƽ���ȷ�����ٶȿ�����
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0  ,0 ,15, 0.45    ,0.0000    ,0.0  ,0   ,0 ,25},//ˮƽά�ȷ�����ٶȿ�����
};

/*
Butter_Parameter Control_Device_Div_LPF_Parameter={
 //200---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};

Butter_Parameter Control_Device_Err_LPF_Parameter={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};
*/
Butter_Parameter Control_Device_Div_LPF_Parameter;
Butter_Parameter Control_Device_Err_LPF_Parameter;

AllControler Total_Controller;//ϵͳ�ܿ�����
void PID_Init(PID_Controler *Controler,Controler_Label Label)
{
  Controler->Err_Limit_Flag=(uint8)(Control_Unit[Label][0]);//1ƫ���޷���־
  Controler->Integrate_Limit_Flag=(uint8)(Control_Unit[Label][1]);//2�����޷���־
  Controler->Integrate_Separation_Flag=(uint8)(Control_Unit[Label][2]);//3���ַ����־
  Controler->Expect=Control_Unit[Label][3];//4����
  Controler->FeedBack=Control_Unit[Label][4];//5����ֵ
  Controler->Err=Control_Unit[Label][5];//6ƫ��
  Controler->Last_Err=Control_Unit[Label][6];//7�ϴ�ƫ��
  Controler->Err_Max=Control_Unit[Label][7];//8ƫ���޷�ֵ
  Controler->Integrate_Separation_Err=Control_Unit[Label][8];//9���ַ���ƫ��ֵ
  Controler->Integrate=Control_Unit[Label][9];//10����ֵ
  Controler->Integrate_Max=Control_Unit[Label][10];//11�����޷�ֵ
  Controler->Kp=Control_Unit[Label][11];//12���Ʋ���Kp
  Controler->Ki=Control_Unit[Label][12];//13���Ʋ���Ki
  Controler->Kd=Control_Unit[Label][13];//14���Ʋ���Ki
  Controler->Control_OutPut=Control_Unit[Label][14];//15�����������
  Controler->Last_Control_OutPut=Control_Unit[Label][15];//16�ϴο����������
  Controler->Control_OutPut_Limit=Control_Unit[Label][16];//16�ϴο����������
}

void Total_PID_Init(void)
{
 PID_Init(&Total_Controller.Pitch_Angle_Control,Pitch_Angle_Controler);
 PID_Init(&Total_Controller.Pitch_Gyro_Control,Pitch_Gyro_Controler);
 PID_Init(&Total_Controller.Roll_Angle_Control,Roll_Angle_Controler);
 PID_Init(&Total_Controller.Roll_Gyro_Control,Roll_Gyro_Controler);
 PID_Init(&Total_Controller.Yaw_Angle_Control,Yaw_Angle_Controler);
 PID_Init(&Total_Controller.Yaw_Gyro_Control,Yaw_Gyro_Controler);
 PID_Init(&Total_Controller.High_Position_Control,High_Position_Controler);
 PID_Init(&Total_Controller.High_Speed_Control,High_Speed_Controler);
 PID_Init(&Total_Controller.Longitude_Position_Control,Longitude_Position_Controler);
 PID_Init(&Total_Controller.Longitude_Speed_Control,Longitude_Speed_Controler);
 PID_Init(&Total_Controller.Latitude_Position_Control,Latitude_Position_Controler);
 PID_Init(&Total_Controller.Latitude_Speed_Control,Latitude_Speed_Controler);

 PID_Init(&Total_Controller.High_Acce_Control,High_Acce_Controler);
 PID_Init(&Total_Controller.Longitude_Acce_Control,Longitude_Acce_Controler);
 PID_Init(&Total_Controller.Latitude_Acce_Control,Latitude_Acce_Controler);

 Set_Cutoff_Frequency(Sampling_Freq, 5 ,&Control_Device_Err_LPF_Parameter);
 Set_Cutoff_Frequency(Sampling_Freq, 20,&Control_Device_Div_LPF_Parameter);
}

float PID_Control(PID_Controler *Controler)
{
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
                         +Controler->Integrate//����
                         +Controler->Kd*(Controler->Err-Controler->Last_Err);//΢��
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Yaw(PID_Controler *Controler)
{
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
/***********************ƫ����ƫ���+-180����*****************************/
  if(Controler->Err<-180)  Controler->Err=Controler->Err+360;
  if(Controler->Err>180)  Controler->Err=Controler->Err-360;

  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
                         +Controler->Integrate//����
                         +Controler->Kd*(Controler->Err-Controler->Last_Err);//΢��
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}



float Control_Device_LPF(float curr_inputer,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
        /* ���ٶȼ�Butterworth�˲� */
	/* ��ȡ����x(n) */
        Buffer->Input_Butter[2]=curr_inputer;
	/* Butterworth�˲� */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) ���б��� */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) ���б��� */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
        return (Buffer->Output_Butter[2]);
}

float PID_Control_Div_LPF(PID_Controler *Controler)
{
  int16  i=0;
  float tempa,tempb,tempc,max,min;//���ڷ������˲�
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//ԭʼ΢��

  /******************************************/
  //��ֵ�˲�����֤�õ����ݲ����䣬����������Ծʱ��΢������쳣
  tempa=Controler->Pre_Last_Dis_Err_LPF;
  tempb=Controler->Last_Dis_Err_LPF;
  tempc=Controler->Dis_Err;
  max = tempa > tempb ? tempa:tempb;
  max = max > tempc ? max:tempc;
  min = tempa < tempb ? tempa:tempb;
  min = min < tempc ? min:tempc;
  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
  /*****************************************/

  for(i=4;i>0;i--)//���ֵ�ͨ��΢�����
  {
  Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Div_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz

  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
                         +Controler->Integrate//����
                         //+Controler->Kd*Controler->Dis_Err;//΢��
                         +Controler->Kd*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}


float PID_Control_Err_LPF(PID_Controler *Controler)
{
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//ԭʼ΢��

  Controler->Last_Err_LPF=Controler->Err_LPF;
  Controler->Err_LPF=Control_Device_LPF(Controler->Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Err_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz

  Controler->Dis_Err_LPF=Controler->Err_LPF-Controler->Last_Err_LPF;//ƫ�����ͨ���΢����

  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err_LPF>=Controler->Err_Max)   Controler->Err_LPF= Controler->Err_Max;
  if(Controler->Err_LPF<=-Controler->Err_Max)  Controler->Err_LPF=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err_LPF)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err_LPF//����
                         +Controler->Integrate//����
                          +Controler->Kd*Controler->Dis_Err_LPF;//�Ѷ�ƫ���ͨ���˴����ٶ�΢�������ͨ
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}



float Differential_Forward_PID_Control_Div_LPF(PID_Controler *Controler)//΢������PID������
{
  int16  i=0;
  float tempa,tempb,tempc,max,min;//���ڷ������˲�
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
  Controler->Dis_Err=-(Controler->FeedBack-Controler->Last_FeedBack);//ֻ�Է����ź�΢��
  Controler->Last_FeedBack=Controler->FeedBack;//��¼�ϴη���ֵ
  /******************************************/
  //��ֵ�˲�����֤�õ����ݲ����䣬����������Ծʱ��΢������쳣
  tempa=Controler->Pre_Last_Dis_Err_LPF;
  tempb=Controler->Last_Dis_Err_LPF;
  tempc=Controler->Dis_Err;
  max = tempa > tempb ? tempa:tempb;
  max = max > tempc ? max:tempc;
  min = tempa < tempb ? tempa:tempb;
  min = min < tempc ? min:tempc;
  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
  /*****************************************/

  for(i=4;i>0;i--)//���ֵ�ͨ��΢�����
  {
  Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Div_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz

  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
                         +Controler->Integrate//����
                         //+Controler->Kd*Controler->Dis_Err;//΢��
                         +Controler->Kd*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Div_LPF_For_Gyro(PID_Controler *Controler)
{
  int16  i=0;
  float tempa,tempb,tempc,max,min;//���ڷ������˲�
  float controller_dt=0;
  Test_Period(&Controler->PID_Controller_Dt);
  controller_dt=Controler->PID_Controller_Dt.Time_Delta/1000.0;
  if(controller_dt<0.001) return 0;
/*******ƫ�����*********************/
  Controler->Pre_Last_Err=Controler->Last_Err;//���ϴ�ƫ��
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
  Controler->Dis_Err=(Controler->Err-Controler->Pre_Last_Err);//�����һ�β�����΢��
  /******************************************/
  //��ֵ�˲�����֤�õ����ݲ����䣬����������Ծʱ��΢������쳣
  tempa=Controler->Pre_Last_Dis_Err_LPF;
  tempb=Controler->Last_Dis_Err_LPF;
  tempc=Controler->Dis_Err;
  max = tempa > tempb ? tempa:tempb;
  max = max > tempc ? max:tempc;
  min = tempa < tempb ? tempa:tempb;
  min = min < tempc ? min:tempc;
  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
  /*****************************************/
  for(i=4;i>0;i--)//���ֵ�ͨ��΢�����
  {
  Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Div_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz
  Controler->Dis_Error_History[0]=constrain_float(Controler->Dis_Error_History[0],-200,200);//΢�����޷�
  Controler->Adaptable_Kd=Controler->Kd*(1+Controler->Dis_Error_History[0]/200);//����Ӧ΢�ֲ���
    
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
                         +Controler->Integrate//����
                         //+Controler->Kd*Controler->Dis_Err;//΢��
                         //+Controler->Kd*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
                         +Controler->Adaptable_Kd*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}


void  PID_Integrate_Reset(PID_Controler *Controler)  {Controler->Integrate=0.0f;}

void Take_Off_Reset(void)
{
  PID_Integrate_Reset(&Total_Controller.Roll_Gyro_Control);//���ǰ���λ���
  PID_Integrate_Reset(&Total_Controller.Pitch_Gyro_Control);
  PID_Integrate_Reset(&Total_Controller.Yaw_Gyro_Control);
  PID_Integrate_Reset(&Total_Controller.Pitch_Angle_Control);
  PID_Integrate_Reset(&Total_Controller.Roll_Angle_Control);
  PID_Integrate_Reset(&Total_Controller.Yaw_Angle_Control);

  PID_Integrate_Reset(&Total_Controller.Longitude_Speed_Control);//λ�ÿ����ٶȻ�������
  PID_Integrate_Reset(&Total_Controller.Latitude_Speed_Control);
}

void Throttle_Control_Reset(void)
{
  PID_Integrate_Reset(&Total_Controller.High_Acce_Control);
  PID_Integrate_Reset(&Total_Controller.High_Speed_Control);
  PID_Integrate_Reset(&Total_Controller.High_Position_Control);
}

volatile FLASH_Status FLASHStatus1 = FLASH_COMPLETE;      //Flash����״̬����
Vector3f_pid PID_Parameter[11]={0};
void WriteFlash_PID_Float(uint32_t WriteAddress,
                         Vector3f_pid pid1,
                         Vector3f_pid pid2,
                         Vector3f_pid pid3,
                         Vector3f_pid pid4,
                         Vector3f_pid pid5,
                         Vector3f_pid pid6,
                         Vector3f_pid pid7,
                         Vector3f_pid pid8,
                         Vector3f_pid pid9,
                         Vector3f_pid pid10,
                         Vector3f_pid pid11)
{
        uint32_t Buf[33]={0};
        uint16_t i=0;
        Buf[0]=*(uint32_t *)(&pid1.p);//���ڴ��������ĸ��ֽ�д�뵽Flash
        Buf[1]=*(uint32_t *)(&pid1.i);
        Buf[2]=*(uint32_t *)(&pid1.d);
        Buf[3]=*(uint32_t *)(&pid2.p);
        Buf[4]=*(uint32_t *)(&pid2.i);
        Buf[5]=*(uint32_t *)(&pid2.d);
        Buf[6]=*(uint32_t *)(&pid3.p);
        Buf[7]=*(uint32_t *)(&pid3.i);
        Buf[8]=*(uint32_t *)(&pid3.d);
        Buf[9]=*(uint32_t *)(&pid4.p);
        Buf[10]=*(uint32_t *)(&pid4.i);
        Buf[11]=*(uint32_t *)(&pid4.d);
        Buf[12]=*(uint32_t *)(&pid5.p);
        Buf[13]=*(uint32_t *)(&pid5.i);
        Buf[14]=*(uint32_t *)(&pid5.d);
        Buf[15]=*(uint32_t *)(&pid6.p);
        Buf[16]=*(uint32_t *)(&pid6.i);
        Buf[17]=*(uint32_t *)(&pid6.d);
        Buf[18]=*(uint32_t *)(&pid7.p);
        Buf[19]=*(uint32_t *)(&pid7.i);
        Buf[20]=*(uint32_t *)(&pid7.d);
        Buf[21]=*(uint32_t *)(&pid8.p);
        Buf[22]=*(uint32_t *)(&pid8.i);
        Buf[23]=*(uint32_t *)(&pid8.d);
        Buf[24]=*(uint32_t *)(&pid9.p);
        Buf[25]=*(uint32_t *)(&pid9.i);
        Buf[26]=*(uint32_t *)(&pid9.d);
        Buf[27]=*(uint32_t *)(&pid10.p);
        Buf[28]=*(uint32_t *)(&pid10.i);
        Buf[29]=*(uint32_t *)(&pid10.d);
        Buf[30]=*(uint32_t *)(&pid11.p);
        Buf[31]=*(uint32_t *)(&pid11.i);
        Buf[32]=*(uint32_t *)(&pid11.d);
        FLASH_UnlockBank1();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASHStatus1 = FLASH_ErasePage(PID_STARTADDR);
	if(FLASHStatus1 == FLASH_COMPLETE)
	{
          for(i=0;i<33;i++)
          {
             FLASHStatus1 = FLASH_ProgramWord(PID_STARTADDR + WriteAddress+4*i,Buf[i]);
          }
	}
	FLASH_LockBank1();
}


uint8_t ReadFlash_PID_Float(uint32_t ReadAddress,
                         float *WriteData1,
                         float *WriteData2,
                         float *WriteData3)
{
    uint8_t buf[12];
    uint16_t i=0;
    uint8_t flag=0x00;
    ReadAddress = (uint32_t)PID_STARTADDR + ReadAddress;
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

uint8_t Sort_PID_Cnt=0;
uint8_t Sort_PID_Flag=0;
void Save_Or_Reset_PID_Parameter()
{
    if(Sort_PID_Flag==1)//������վ����PID����д��Flash
    {
        PID_Parameter[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
        PID_Parameter[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
        PID_Parameter[0].d=Total_Controller.Pitch_Gyro_Control.Kd;

        PID_Parameter[1].p=Total_Controller.Roll_Gyro_Control.Kp;
        PID_Parameter[1].i=Total_Controller.Roll_Gyro_Control.Ki;
        PID_Parameter[1].d=Total_Controller.Roll_Gyro_Control.Kd;

        PID_Parameter[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
        PID_Parameter[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
        PID_Parameter[2].d=Total_Controller.Yaw_Gyro_Control.Kd;

        PID_Parameter[3].p=Total_Controller.Pitch_Angle_Control.Kp;
        PID_Parameter[3].i=Total_Controller.Pitch_Angle_Control.Ki;
        PID_Parameter[3].d=Total_Controller.Pitch_Angle_Control.Kd;

        PID_Parameter[4].p=Total_Controller.Roll_Angle_Control.Kp;
        PID_Parameter[4].i=Total_Controller.Roll_Angle_Control.Ki;
        PID_Parameter[4].d=Total_Controller.Roll_Angle_Control.Kd;

        PID_Parameter[5].p=Total_Controller.Yaw_Angle_Control.Kp;
        PID_Parameter[5].i=Total_Controller.Yaw_Angle_Control.Ki;
        PID_Parameter[5].d=Total_Controller.Yaw_Angle_Control.Kd;

        PID_Parameter[6].p=Total_Controller.High_Speed_Control.Kp;
        PID_Parameter[6].i=Total_Controller.High_Speed_Control.Ki;
        PID_Parameter[6].d=Total_Controller.High_Speed_Control.Kd;

        PID_Parameter[7].p=Total_Controller.High_Position_Control.Kp;
        PID_Parameter[7].i=Total_Controller.High_Position_Control.Ki;
        PID_Parameter[7].d=Total_Controller.High_Position_Control.Kd;

        PID_Parameter[8].p=Total_Controller.Latitude_Speed_Control.Kp;
        PID_Parameter[8].i=Total_Controller.Latitude_Speed_Control.Ki;
        PID_Parameter[8].d=Total_Controller.Latitude_Speed_Control.Kd;

        PID_Parameter[9].p=Total_Controller.Latitude_Position_Control.Kp;
        PID_Parameter[9].i=Total_Controller.Latitude_Position_Control.Ki;
        PID_Parameter[9].d=Total_Controller.Latitude_Position_Control.Kd;

        PID_Parameter[10].p=Total_Controller.High_Acce_Control.Kp;
        PID_Parameter[10].i=Total_Controller.High_Acce_Control.Ki;
        PID_Parameter[10].d=Total_Controller.High_Acce_Control.Kd;

        WriteFlash_PID_Float(PID1_Address,
                             PID_Parameter[0],
                             PID_Parameter[1],
                             PID_Parameter[2],
                             PID_Parameter[3],
                             PID_Parameter[4],
                             PID_Parameter[5],
                             PID_Parameter[6],
                             PID_Parameter[7],
                             PID_Parameter[8],
                             PID_Parameter[9],
                             PID_Parameter[10]);

        Sort_PID_Flag=0;
    }
    else if(Sort_PID_Flag==2)//����λPID��������д��Flash
    {
        Total_PID_Init();//��PID��������Ϊ����Control_Unit���������
        PID_Parameter[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
        PID_Parameter[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
        PID_Parameter[0].d=Total_Controller.Pitch_Gyro_Control.Kd;

        PID_Parameter[1].p=Total_Controller.Roll_Gyro_Control.Kp;
        PID_Parameter[1].i=Total_Controller.Roll_Gyro_Control.Ki;
        PID_Parameter[1].d=Total_Controller.Roll_Gyro_Control.Kd;

        PID_Parameter[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
        PID_Parameter[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
        PID_Parameter[2].d=Total_Controller.Yaw_Gyro_Control.Kd;

        PID_Parameter[3].p=Total_Controller.Pitch_Angle_Control.Kp;
        PID_Parameter[3].i=Total_Controller.Pitch_Angle_Control.Ki;
        PID_Parameter[3].d=Total_Controller.Pitch_Angle_Control.Kd;

        PID_Parameter[4].p=Total_Controller.Roll_Angle_Control.Kp;
        PID_Parameter[4].i=Total_Controller.Roll_Angle_Control.Ki;
        PID_Parameter[4].d=Total_Controller.Roll_Angle_Control.Kd;

        PID_Parameter[5].p=Total_Controller.Yaw_Angle_Control.Kp;
        PID_Parameter[5].i=Total_Controller.Yaw_Angle_Control.Ki;
        PID_Parameter[5].d=Total_Controller.Yaw_Angle_Control.Kd;

        PID_Parameter[6].p=Total_Controller.High_Speed_Control.Kp;
        PID_Parameter[6].i=Total_Controller.High_Speed_Control.Ki;
        PID_Parameter[6].d=Total_Controller.High_Speed_Control.Kd;

        PID_Parameter[7].p=Total_Controller.High_Position_Control.Kp;
        PID_Parameter[7].i=Total_Controller.High_Position_Control.Ki;
        PID_Parameter[7].d=Total_Controller.High_Position_Control.Kd;

        PID_Parameter[8].p=Total_Controller.Latitude_Speed_Control.Kp;
        PID_Parameter[8].i=Total_Controller.Latitude_Speed_Control.Ki;
        PID_Parameter[8].d=Total_Controller.Latitude_Speed_Control.Kd;

        PID_Parameter[9].p=Total_Controller.Latitude_Position_Control.Kp;
        PID_Parameter[9].i=Total_Controller.Latitude_Position_Control.Ki;
        PID_Parameter[9].d=Total_Controller.Latitude_Position_Control.Kd;

        PID_Parameter[10].p=Total_Controller.High_Acce_Control.Kp;
        PID_Parameter[10].i=Total_Controller.High_Acce_Control.Ki;
        PID_Parameter[10].d=Total_Controller.High_Acce_Control.Kd;

        WriteFlash_PID_Float(PID1_Address,
                             PID_Parameter[0],
                             PID_Parameter[1],
                             PID_Parameter[2],
                             PID_Parameter[3],
                             PID_Parameter[4],
                             PID_Parameter[5],
                             PID_Parameter[6],
                             PID_Parameter[7],
                             PID_Parameter[8],
                             PID_Parameter[9],
                             PID_Parameter[10]);
        Sort_PID_Flag=0;
        ANO_Send_PID_Flag[0]=1;//�ظ�Ĭ�ϲ����󣬽����µ����ݷ����õ���վ
        ANO_Send_PID_Flag[1]=1;
        ANO_Send_PID_Flag[2]=1;
        ANO_Send_PID_Flag[3]=1;
        ANO_Send_PID_Flag[4]=1;
        ANO_Send_PID_Flag[5]=1;
    }
    else if(Sort_PID_Flag==3)//����λPID��������д��Flash
    {
        Total_PID_Init();//��PID��������Ϊ����Control_Unit���������
        PID_Parameter[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
        PID_Parameter[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
        PID_Parameter[0].d=Total_Controller.Pitch_Gyro_Control.Kd;

        PID_Parameter[1].p=Total_Controller.Roll_Gyro_Control.Kp;
        PID_Parameter[1].i=Total_Controller.Roll_Gyro_Control.Ki;
        PID_Parameter[1].d=Total_Controller.Roll_Gyro_Control.Kd;

        PID_Parameter[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
        PID_Parameter[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
        PID_Parameter[2].d=Total_Controller.Yaw_Gyro_Control.Kd;

        PID_Parameter[3].p=Total_Controller.Pitch_Angle_Control.Kp;
        PID_Parameter[3].i=Total_Controller.Pitch_Angle_Control.Ki;
        PID_Parameter[3].d=Total_Controller.Pitch_Angle_Control.Kd;

        PID_Parameter[4].p=Total_Controller.Roll_Angle_Control.Kp;
        PID_Parameter[4].i=Total_Controller.Roll_Angle_Control.Ki;
        PID_Parameter[4].d=Total_Controller.Roll_Angle_Control.Kd;

        PID_Parameter[5].p=Total_Controller.Yaw_Angle_Control.Kp;
        PID_Parameter[5].i=Total_Controller.Yaw_Angle_Control.Ki;
        PID_Parameter[5].d=Total_Controller.Yaw_Angle_Control.Kd;

        PID_Parameter[6].p=Total_Controller.High_Speed_Control.Kp;
        PID_Parameter[6].i=Total_Controller.High_Speed_Control.Ki;
        PID_Parameter[6].d=Total_Controller.High_Speed_Control.Kd;

        PID_Parameter[7].p=Total_Controller.High_Position_Control.Kp;
        PID_Parameter[7].i=Total_Controller.High_Position_Control.Ki;
        PID_Parameter[7].d=Total_Controller.High_Position_Control.Kd;

        PID_Parameter[8].p=Total_Controller.Latitude_Speed_Control.Kp;
        PID_Parameter[8].i=Total_Controller.Latitude_Speed_Control.Ki;
        PID_Parameter[8].d=Total_Controller.Latitude_Speed_Control.Kd;

        PID_Parameter[9].p=Total_Controller.Latitude_Position_Control.Kp;
        PID_Parameter[9].i=Total_Controller.Latitude_Position_Control.Ki;
        PID_Parameter[9].d=Total_Controller.Latitude_Position_Control.Kd;

        PID_Parameter[10].p=Total_Controller.High_Acce_Control.Kp;
        PID_Parameter[10].i=Total_Controller.High_Acce_Control.Ki;
        PID_Parameter[10].d=Total_Controller.High_Acce_Control.Kd;

        WriteFlash_PID_Float(PID1_Address,
                             PID_Parameter[0],
                             PID_Parameter[1],
                             PID_Parameter[2],
                             PID_Parameter[3],
                             PID_Parameter[4],
                             PID_Parameter[5],
                             PID_Parameter[6],
                             PID_Parameter[7],
                             PID_Parameter[8],
                             PID_Parameter[9],
                             PID_Parameter[10]);
        Sort_PID_Flag=0;
        ANO_Send_PID_Flag_USB[0]=1;//�ظ�Ĭ�ϲ����󣬽����µ����ݷ����õ���վ
        ANO_Send_PID_Flag_USB[1]=1;
        ANO_Send_PID_Flag_USB[2]=1;
        ANO_Send_PID_Flag_USB[3]=1;
        ANO_Send_PID_Flag_USB[4]=1;
        ANO_Send_PID_Flag_USB[5]=1;
    }
}


typedef struct
{
uint8_t No_0xFF[11];
}PID_Parameter_Flag;
PID_Parameter_Flag PID_Parameter_Read_Flag;
void PID_Paramter_Init_With_Flash()
{
  uint16_t i=0;
  Total_PID_Init();
  for(i=0;i<11;i++)
  {
    PID_Parameter_Read_Flag.No_0xFF[0]=ReadFlash_PID_Float(PID1_Address+12*i,
                        &PID_Parameter[i].p,
                        &PID_Parameter[i].i,
                        &PID_Parameter[i].d);
  }
  if(PID_Parameter_Read_Flag.No_0xFF[0]!=0x07
     &&PID_Parameter_Read_Flag.No_0xFF[1]!=0x07
       &&PID_Parameter_Read_Flag.No_0xFF[2]!=0x07
         &&PID_Parameter_Read_Flag.No_0xFF[3]!=0x07
           &&PID_Parameter_Read_Flag.No_0xFF[4]!=0x07
             &&PID_Parameter_Read_Flag.No_0xFF[5]!=0x07
               &&PID_Parameter_Read_Flag.No_0xFF[6]!=0x07
                 &&PID_Parameter_Read_Flag.No_0xFF[7]!=0x07
                   &&PID_Parameter_Read_Flag.No_0xFF[8]!=0x07
                     &&PID_Parameter_Read_Flag.No_0xFF[9]!=0x07
                       &&PID_Parameter_Read_Flag.No_0xFF[10]!=0x07)//Flash����������������PID����ֵ
  {
        Total_Controller.Pitch_Gyro_Control.Kp=PID_Parameter[0].p;
        Total_Controller.Pitch_Gyro_Control.Ki=PID_Parameter[0].i;
        Total_Controller.Pitch_Gyro_Control.Kd=PID_Parameter[0].d;

        Total_Controller.Roll_Gyro_Control.Kp=PID_Parameter[1].p;
        Total_Controller.Roll_Gyro_Control.Ki=PID_Parameter[1].i;
        Total_Controller.Roll_Gyro_Control.Kd=PID_Parameter[1].d;

        Total_Controller.Yaw_Gyro_Control.Kp=PID_Parameter[2].p;
        Total_Controller.Yaw_Gyro_Control.Ki=PID_Parameter[2].i;
        Total_Controller.Yaw_Gyro_Control.Kd=PID_Parameter[2].d;

        Total_Controller.Pitch_Angle_Control.Kp=PID_Parameter[3].p;
        Total_Controller.Pitch_Angle_Control.Ki=PID_Parameter[3].i;
        Total_Controller.Pitch_Angle_Control.Kd=PID_Parameter[3].d;

        Total_Controller.Roll_Angle_Control.Kp=PID_Parameter[4].p;
        Total_Controller.Roll_Angle_Control.Ki=PID_Parameter[4].i;
        Total_Controller.Roll_Angle_Control.Kd=PID_Parameter[4].d;

        Total_Controller.Yaw_Angle_Control.Kp=PID_Parameter[5].p;
        Total_Controller.Yaw_Angle_Control.Ki=PID_Parameter[5].i;
        Total_Controller.Yaw_Angle_Control.Kd=PID_Parameter[5].d;

        Total_Controller.High_Speed_Control.Kp=PID_Parameter[6].p;
        Total_Controller.High_Speed_Control.Ki=PID_Parameter[6].i;
        Total_Controller.High_Speed_Control.Kd=PID_Parameter[6].d;

        Total_Controller.High_Position_Control.Kp=PID_Parameter[7].p;
        Total_Controller.High_Position_Control.Ki=PID_Parameter[7].i;
        Total_Controller.High_Position_Control.Kd=PID_Parameter[7].d;

        Total_Controller.Latitude_Speed_Control.Kp=PID_Parameter[8].p;
        Total_Controller.Latitude_Speed_Control.Ki=PID_Parameter[8].i;
        Total_Controller.Latitude_Speed_Control.Kd=PID_Parameter[8].d;

        Total_Controller.Latitude_Position_Control.Kp=PID_Parameter[9].p;
        Total_Controller.Latitude_Position_Control.Ki=PID_Parameter[9].i;
        Total_Controller.Latitude_Position_Control.Kd=PID_Parameter[9].d;
/***********************λ�ÿ��ƣ�λ�á��ٶȲ�������һ��PID����**********************************************************/
        Total_Controller.Longitude_Speed_Control.Kp=PID_Parameter[8].p;
        Total_Controller.Longitude_Speed_Control.Ki=PID_Parameter[8].i;
        Total_Controller.Longitude_Speed_Control.Kd=PID_Parameter[8].d;

        Total_Controller.Longitude_Position_Control.Kp=PID_Parameter[9].p;
        Total_Controller.Longitude_Position_Control.Ki=PID_Parameter[9].i;
        Total_Controller.Longitude_Position_Control.Kd=PID_Parameter[9].d;

        Total_Controller.High_Acce_Control.Kp=PID_Parameter[10].p;
        Total_Controller.High_Acce_Control.Ki=PID_Parameter[10].i;
        Total_Controller.High_Acce_Control.Kd=PID_Parameter[10].d;
  }
}

