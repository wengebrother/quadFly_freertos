#include "sensorTask.h"
/************������������Ϣ*************/
#define sensor_TASK_STK_ZIZE   128     //��ջ��С
#define sensorTaskPrio   2    //�����������ȼ�
static TaskHandle_t sensor_Task_Handle = NULL; //������
static void sensor_Task(void* pvParameters); //������



void sensor_Task(void* pvParameters)
{
  float imuGyroData[3]={0};
   while(1){
		 
		 
		   gyroDataFlilter(imuGyroData);
   
	  //Led_Off(GPIOC,GPIO_Pin_4);
     printf("����1����\r\n");
		 GET_MPU_DATA();//����������
	  //SPL06_001_StateMachine();
	   //AHRSUpdate_GraDes_TimeSync(X_w_av,Y_w_av,Z_w_av,X_g_av,Y_g_av,Z_g_av);//��̬����
	   vTaskDelay( 10 );//������ʱ1s 
    //Led_ON(GPIOC,GPIO_Pin_4);
		 //QuadShow();//LCD��ʾ����
   }
   
   
}



void createSensorTask(void)
{

   xTaskCreate((TaskFunction_t )sensor_Task,
	             (const char*    )"sensor_Task",
							 (uint16_t       )sensor_TASK_STK_ZIZE,
							 (void*          )NULL, 
							 (UBaseType_t    )sensorTaskPrio, 
							 (TaskHandle_t*  )&sensor_Task_Handle);



}


