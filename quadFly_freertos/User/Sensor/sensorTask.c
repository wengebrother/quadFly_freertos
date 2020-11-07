#include "sensorTask.h"
/************传感器任务信息*************/
#define sensor_TASK_STK_ZIZE   128     //堆栈大小
#define sensorTaskPrio   2    //启动任务优先级
static TaskHandle_t sensor_Task_Handle = NULL; //任务句柄
static void sensor_Task(void* pvParameters); //任务函数



void sensor_Task(void* pvParameters)
{
  float imuGyroData[3]={0};
   while(1){
		 
		 
		   gyroDataFlilter(imuGyroData);
   
	  //Led_Off(GPIOC,GPIO_Pin_4);
     printf("任务1运行\r\n");
		 GET_MPU_DATA();//传感器数据
	  //SPL06_001_StateMachine();
	   //AHRSUpdate_GraDes_TimeSync(X_w_av,Y_w_av,Z_w_av,X_g_av,Y_g_av,Z_g_av);//姿态解算
	   vTaskDelay( 10 );//阻塞延时1s 
    //Led_ON(GPIOC,GPIO_Pin_4);
		 //QuadShow();//LCD显示程序
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


