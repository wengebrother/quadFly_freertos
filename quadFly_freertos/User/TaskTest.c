#include "TaskTest.h"
#include "Headfile.h"
#include "Message_Queue.h"
BaseType_t test1TaskPass=pdPASS,test2TaskPass=pdPASS;

/************��������1��Ϣ*************/
#define Test1_TASK_STK_ZIZE   128     //��ջ��С
#define Test1TaskPrio   2    //�����������ȼ�
static TaskHandle_t Test1_Task_Handle = NULL;
static void Test1_Task(void* pvParameters);


/************��������2��Ϣ*************/
#define Test2_TASK_STK_ZIZE   128     //��ջ��С
#define Test2TaskPrio   3    //�����������ȼ�
static TaskHandle_t Test2_Task_Handle = NULL;
static void Test2_Task(void* pvParameters);



void Test1_Task(void* pvParameters)
{
  uint32_t send_data1 = 666;
	BaseType_t xReturn = pdPASS;
   while(1){
       
		 
		 xReturn = xQueueSend(Test_Message_Queue,&send_data1,0);
		 
		 if(xReturn==pdPASS){
			 
		 xSemaphoreGive( Test_BinarySem_handlet );
		   printf("��Ϣ���ͳɹ�\r\n");
		 }
	  //Led_Off(GPIOC,GPIO_Pin_4);
     printf("����1����\r\n");
	   vTaskDelay( 1000 );//������ʱ1s 
     Led_ON(GPIOC,GPIO_Pin_4);
		
   }
   
   
}




void Test2_Task(void* pvParameters)
{
  BaseType_t xReturn = pdTRUE;
  uint32_t r_queue=0;
	while(1){
	
		
		
		
		if(xSemaphoreTake(Test_BinarySem_handlet,portMAX_DELAY)==pdPASS){
		
		
		   xReturn = xQueueReceive(Test_Message_Queue,&r_queue,portMAX_DELAY);
		
		}
		 if(xReturn==pdPASS){
		 
		   printf("��Ϣ���ճɹ�\r\n");
		 }
	  printf("����2����\r\n");
    
		
		if(r_queue==666){
		
		    Led_Off(GPIOC,GPIO_Pin_4);
			  r_queue=0;
		}
	   
	   //vTaskDelay( 500 );//������ʱ0.5s 
	}
 
  

}

void createTask(void)
{

   test1TaskPass=xTaskCreate((TaskFunction_t )Test1_Task,
	             (const char*    )"Test1_Task",
							 (uint16_t       )Test1_TASK_STK_ZIZE,
							 (void*          )NULL, 
							 (UBaseType_t    )Test1TaskPrio, 
							 (TaskHandle_t*  )&Test1_Task_Handle);
							 
							 
     
//        if(test1TaskPass==pdPASS)
//				{
//				   printf("����1�����ɹ�\r\n");
//				
//				}
//        else{
//				
//				
//				printf("����1����ʧ��\r\n");
//				}

     	
	  	 test2TaskPass=xTaskCreate((TaskFunction_t )Test2_Task,
	             (const char*    )"Test2_Task",
							 (uint16_t       )Test2_TASK_STK_ZIZE,
							 (void*          )NULL, 
							 (UBaseType_t    )Test2TaskPrio, 
							 (TaskHandle_t*  )&Test2_Task_Handle);	
				
//				 if(test2TaskPass==pdPASS)
//				{
//				   printf("����2�����ɹ�\r\n");
//				
//				}
//         else
//				 {
//				   printf("����2����ʧ��\r\n");
//				 }


}
