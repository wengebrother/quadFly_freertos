#include "Headfile.h"
#include "FreeRTOS.h" 
#include "task.h"
#include "queue.h"
#include "TaskTest.h"
#include "sensorTask.h"
#include "AHRS_Task.h"
#include "Message_Queue.h"
/*************************************************************************************************************************
//----------------------------------------------------------------------------------------------------------------------//
	*		四旋翼飞行器飞控板V1.0	武汉科技大学  By.YuYi
	*		CSDN博客: http://blog.csdn.net/u011992534
	*               优酷ID：NamelessCotrun无名小哥
	*               无名科创开源飞控QQ群：540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
	*		MCU: 	STM32F103RCT6  72MHz
	*		接口映射表:
	*		默认IMU(MPU6050 + IST8310 + SPL06_001):
	*				IMU_SCL						-->	PB6
	*				IMU_SDA						-->	PB7
	*		OLED:
	*				OLED_D0						-->	PC3
	*				OLED_D1						-->	PC2
	*				OLED_RST					-->	PC1
	*				OLED_DC						-->	PC0
	*				OLED_CS						-->	GND
	*		电调输出:
	*				MOTOR1						-->	PB1	-->	TIM3_CH1
	*				MOTOR2						-->	PB0	-->	TIM3_CH2
	*				MOTOR3						-->	PA7	-->	TIM3_CH3
	*				MOTOR4						-->	PA6	-->	TIM3_CH4
	*		RC输入:
	*				PPM信号					        -->	PA8	-->	TIM1_CH1
	*				SBUS信号					-->	PB11-->	USART3_RX
	*		超声波:
	*				TRIG						-->	PB9  TRIG
	*				ECHO						-->	PA1  IO4
	*		按键:
	*				S1						-->	PC8
	*				S2						-->	PC9
	*		LED指示灯:
	*				LED2(Green)			                -->	PA5
	*				LED3(Blue)				        -->	PC10
	*				LED4(Yellow)			                -->	PC5
	*				LED5(Red)			                -->	PC4
	*		SPI(Extended,3.3V)    自制作遥控器  &  传感器模块  MPU6500+HMC5983+MS5611
	*				SPI2_IRQ				        -->	PB12
	*				SPI2_SCK				        -->	PB13
	*				SPI2_MISO				        -->	PB14
	*				SPI2_MOSI				        -->	PB15
	*				SPI2_CE				          -->	PC6
	*				SPI2_CSN				        -->	PC7
	*		USART1(Wireless,3.3V)   山外多功能调试助手  &  Mavlink
	*				USART1_TX				        -->	PA9
	*				USART1_RX				        -->	PA10
	*		USART2(Extended,5V)    外接GPS
	*				USART2_TX				        -->	PA2
	*				USART2_RX				        -->	PA3
	*		USART3(Extended,3.3V)  备用串口  ANO上位机、遥控器SBUS信号输入
	*				USART3_TX				        -->	PB10
	*				USART3_RX				        -->	PB11
	*		预留IO口
	*				IO1				              -->	PC13
	*				IO2				              -->	PC14
	*				IO3				              -->	PC15
	*				IO4				              -->	PA1
************************************************************************************************************************/
/***************************While(1)里面只进行按键、显示、标定等程序****************************************************/
/***************************主要核心：传感器滤波、姿态解算、惯导、控制等代码在TIME.c里面运行**********************************/


/************启动任务信息*************/
#define START_TASK_STK_ZIZE   500     //堆栈大小
#define StartTaskPrio   1    //启动任务优先级
static TaskHandle_t StartTask_Handle = NULL; //启动任务句柄
static void StartTask(void* pvParameters);  //任务函数





int main()
{
	
  BaseType_t startTaskPass=0;//用于判断启动任务函数是否创建成功pdPASS
  BSP_Init();
	delay_ms(500);
	//创建启动任务函数，其他所有任务在这个函数中被创建
	 startTaskPass=xTaskCreate((TaskFunction_t )StartTask,
	             (const char*    )"StartTask",
							 (uint16_t       )START_TASK_STK_ZIZE,
							 (void*          )NULL, 
							 (UBaseType_t    )StartTaskPrio, 
							 (TaskHandle_t*  )&StartTask_Handle);

	//任务创建成功后启动调度器
	 if(startTaskPass)
	 {
		 //printf("任务创建成功\r\n");
		   
	   vTaskStartScheduler();
		 
	 
	 }
	 
	 else
	 {
	    printf("任务创建失败\r\n");
	 }
	
  while(1){
	
		printf("进入主循环\r\n");
  }
}
/******************************移植*************************************************/ 


static void StartTask(void* pvParameters)  //任务函数
{
  	   //printf("进入启动任务\r\n");	
	     taskENTER_CRITICAL();//进入临界区
	  	 
	     Test_BinarySem_handlet=xSemaphoreCreateBinary();//创建二值信号量
	     Test_Message_Queue=xQueueCreate((UBaseType_t ) Test_QUEUE_LEN,(UBaseType_t ) Test_QUEUE_SIZE);
	     if(Test_Message_Queue==NULL){
			    
			    printf("队列创建失败\r\n");
			 
			 }
				 
	
	
	
			 //create_TestTask();//创建测试任务
	
	     createSensorTask();//创建传感器任务
	     //createAHRS_Task();//创建姿态解算任务
	
	
			 vTaskDelete(StartTask_Handle); //启动任务只执行一次，删除启动任务			
			 taskEXIT_CRITICAL();            //退出临界区 
				
}

