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
#include "control_config.h"
#include "control_althold.h"


Vector3f UAV_Cushion_Stop_Point;

float Yaw_Vel_Feedforward_Output=0;//��ֱ�ٶ�ǰ�����������;
float Yaw_Vel_Feedforward_Rate=1.0;//��ֱ�ٶ�ǰ����������APM����Ϊ1��0.45;
float Yaw_Vel_Feedforward_Delta=0;//��ֱ�����ٶȱ仯��;
float Last_Yaw_Vel_Target=0;
float Yaw_Vel_Target=0;


uint16_t  Deadband=200;//������λ����

/*************����ΪĬ��ֵ��У׼ң��������Զ���ֵ**********************/
uint16_t  Deadzone_Min=400;
uint16_t  Deadzone_Max=600;
uint16_t  Thr_Top=1000;//����������г�
uint16_t  Thr_Buttom=0;//����������г�
uint16_t  Thr_Middle=500;//����������г�



void Thr_Scale_Set(Vector_RC *rc_date)
{
  Thr_Buttom=0;
  Thr_Top=rc_date[2].max-rc_date[2].min;
  Thr_Middle=(rc_date[2].max-rc_date[2].min)/2;
  //Deadband=rc_date[2].deadband;//������λ������������
  Deadzone_Min=Thr_Middle-Deadband/2;
  Deadzone_Max=Thr_Middle+Deadband/2;
}

void ncq_control_althold()
{
      static uint16 High_Pos_Control_Cnt=0;//�߶�λ�ÿ��Ƽ�����
      static uint16 High_Vel_Control_Cnt=0;//�߶��ٶȿ��Ƽ�����
      static uint8_t move_flag=0;
      float step_mapping=0,scale_mapping=0;
      Thr_Scale_Set(&RC_Calibration[2]);
      /*�߶ȿ�������1��*/
      /********
          **
          **
          **
          **
          **
       ********/
      /*******************************�߶ȿ�������ʼ****************************************/
      /****************���ߣ��߶�λ�û�+�ٶȻ�+���ٶȻ����������ڷֱ�Ϊ8ms��4ms��4ms*******************/
            if(RC_NewData[0]>=Deadzone_Min
               &&RC_NewData[0]<=Deadzone_Max)
           {
                //�߶�λ�û���������ٶ�����
                if(Total_Controller.High_Position_Control.Expect==0)//���Ż��к�ֻ����һ��
                {
                  Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];//���Ż��к󣬸��¸߶�����
                }
                High_Pos_Control_Cnt++;
                if(High_Pos_Control_Cnt>=2)//��ֱ�߶ȿ�������2*5=10ms
                {
                    High_Pos_Control_Cnt=0;
                    Total_Controller.High_Position_Control.FeedBack=NamelessQuad.Position[_YAW];//����
                    PID_Control(&Total_Controller.High_Position_Control);//���θ߶�λ�ÿ�����
                    //�ڻ��ٶ�����
      #ifdef YAW_VEL_FEEDFORWARD  //�ٶ�ǰ��������
                    Yaw_Vel_Target=Total_Controller.High_Position_Control.Control_OutPut;//�����ٶ�����
                    Yaw_Vel_Feedforward_Delta=(Yaw_Vel_Target-Last_Yaw_Vel_Target)/(2*Delta);//�ٶ������仯��
                    Yaw_Vel_Feedforward_Output=Yaw_Vel_Feedforward_Rate*Yaw_Vel_Feedforward_Delta;
                    Last_Yaw_Vel_Target=Yaw_Vel_Target;//�ϴ��ٶ�����
      #endif
                    Total_Controller.High_Speed_Control.Expect=Total_Controller.High_Position_Control.Control_OutPut;
                }
                move_flag=0;
           }
           else if(RC_NewData[0]>Deadzone_Max)//���������ٶ�����
           {
                  //���Ÿ����ơ������ٶ�����
                 step_mapping=(float)(RC_NewData[0]-Deadzone_Max)/(Thr_Top-Deadzone_Max);//��Χ0~1
                 scale_mapping=constrain_float(step_mapping*step_mapping,0,1);
                 Total_Controller.High_Speed_Control.Expect=Climb_Up_Speed_Max*scale_mapping;//��������ٶ�50cm/s
                 Total_Controller.High_Position_Control.Expect=0;//λ�û�������0
                 move_flag=1;
           }
           else if(RC_NewData[0]<Deadzone_Min)//�����½��ٶ�����
           {
                  //���Ÿ����ơ������ٶ�����
                 step_mapping=(float)(RC_NewData[0]-Deadzone_Min)/(Deadzone_Min-Thr_Buttom);//��Χ0~-1
                 scale_mapping=constrain_float(step_mapping*step_mapping,0,1);
                 Total_Controller.High_Speed_Control.Expect=-Climb_Down_Speed_Max*scale_mapping;//����½��ٶ�40cm/s
                 Total_Controller.High_Position_Control.Expect=0;//λ�û�������0
                 move_flag=2;
           }

      /*�߶ȿ�������2��*/
      /********
              *
              *
         ****
       *
       *
       ********/

      /*******************************��ֱ�ٶȿ�������ʼ*******************************************************************/
           High_Vel_Control_Cnt++;
           if(High_Vel_Control_Cnt>=1)//��ֱ�߶ȿ�������5*1=5ms
           {
              High_Vel_Control_Cnt=0;
              Total_Controller.High_Speed_Control.FeedBack=NamelessQuad.Speed[_YAW];//�ߵ��ٶȹ��Ƹ��ٶȷ���
              PID_Control_Div_LPF(&Total_Controller.High_Speed_Control);//���θ߶��ٶȿ���
           }
      /*******************************�����½��������������ٶ��޷���������*******************************************************************/     
           if(move_flag==1)//�������Ƹ�ʱ���ٶȿ�����������������������½����ٶȽ��зֱ��޷���ȷ�������½�ʱ��̬ƽ��
           {
             Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
                                                                                 -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
           }
           else if(move_flag==2)
           {
             Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
                                                                                 -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
           }  
      /*******************************��ֱ�ٶȿ���������******************************************************************/

      /*�߶ȿ�������3��*/
      /********
             **
           **
         **
           **
             **
       ********/

      /*******************************��ֱ���ٶȿ�������ʼ******************************************************************/
            Total_Controller.High_Acce_Control.Expect=Total_Controller.High_Speed_Control.Control_OutPut;//���ٶ�����
      #ifdef YAW_VEL_FEEDFORWARD  //�ٶ�ǰ��������
            Total_Controller.High_Acce_Control.Expect+=Yaw_Vel_Feedforward_Output;//����ǰ�����������
      #endif
            Total_Controller.High_Acce_Control.FeedBack=Filter_Feedback_NamelessQuad.Acceleration[_YAW];//���ٶȷ���
            PID_Control_Err_LPF(&Total_Controller.High_Acce_Control);//���θ߶ȼ��ٶȿ���
            /**************************************
            ���ٶȻ�ǰ������������ʱ��ע������
            ��ͣ���� = ���ٶȻ�����ֵ + ��׼��ͣ����
            ��ʱ����� F = mg
            ����Ҫ���a�ļ��ٶ�ʱ������� F1=m(g+a)
              F1/F = 1 + a/g
            ��˴�ʱӦ�������ͣ����*(1 + a/g)
            **************************************/
            Total_Controller.High_Acce_Control.Control_OutPut+=(High_Hold_Throttle+Total_Controller.High_Acce_Control.Integrate-Thr_Start)*
                                                                (Total_Controller.High_Acce_Control.Expect/980);
      /*******************************��ֱ���ٶȿ���������******************************************************************/
            if(YAW_Pos_Control_Accel_Disable==0)
          {
            Throttle=Int_Sort(//Throttle_Hover//��ͣ����
                    High_Hold_Throttle+Total_Controller.High_Acce_Control.Control_OutPut);//������Դ�ڸ߶ȼ��ٶȿ��������
          }
          else
          {
            Throttle=Int_Sort(//Throttle_Hover//��ͣ����
                    High_Hold_Throttle+Total_Controller.High_Speed_Control.Control_OutPut);//������Դ�ڸ߶��ٶȿ��������
          }
      /*****************************************�߶ȿ������������������ſ�����***********************************************************/
}




