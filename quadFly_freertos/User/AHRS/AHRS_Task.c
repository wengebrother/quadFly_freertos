#include "AHRS_Task.h"

/************������������Ϣ*************/
#define AHRS_TASK_STK_ZIZE   128     //��ջ��С
#define AHRS_TaskPrio   2    //�����������ȼ�
static TaskHandle_t AHRS_Task_Handle = NULL; //������
static void AHRS_Task(void* pvParameters); //������




void AHRS_Task(void* pvParameters)
{

   while(1){
   
	  //Led_Off(GPIOC,GPIO_Pin_4);
     printf("��̬������������\r\n");	  
	   //AHRSUpdate_GraDes_TimeSync(X_w_av,Y_w_av,Z_w_av,X_g_av,Y_g_av,Z_g_av);//��̬����
	   vTaskDelay( 10 );//������ʱ1s 
    //Led_ON(GPIOC,GPIO_Pin_4);
		 QuadShow();//LCD��ʾ����
   }
   
   
}



void createAHRS_Task(void)
{

   xTaskCreate((TaskFunction_t )AHRS_Task,
	             (const char*    )"AHRS_Task",
							 (uint16_t       )AHRS_TASK_STK_ZIZE,
							 (void*          )NULL, 
							 (UBaseType_t    )AHRS_TaskPrio, 
							 (TaskHandle_t*  )&AHRS_Task_Handle);



}

