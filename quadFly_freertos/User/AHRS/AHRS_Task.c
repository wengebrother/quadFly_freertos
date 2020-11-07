#include "AHRS_Task.h"

/************传感器任务信息*************/
#define AHRS_TASK_STK_ZIZE   128     //堆栈大小
#define AHRS_TaskPrio   2    //启动任务优先级
static TaskHandle_t AHRS_Task_Handle = NULL; //任务句柄
static void AHRS_Task(void* pvParameters); //任务函数




void AHRS_Task(void* pvParameters)
{

   while(1){
   
	  //Led_Off(GPIOC,GPIO_Pin_4);
     printf("姿态解算任务运行\r\n");	  
	   //AHRSUpdate_GraDes_TimeSync(X_w_av,Y_w_av,Z_w_av,X_g_av,Y_g_av,Z_g_av);//姿态解算
	   vTaskDelay( 10 );//阻塞延时1s 
    //Led_ON(GPIOC,GPIO_Pin_4);
		 QuadShow();//LCD显示程序
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

