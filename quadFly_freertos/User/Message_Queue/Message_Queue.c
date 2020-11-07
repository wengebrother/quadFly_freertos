#include "Message_Queue.h"


/********消息队列句柄*******/
QueueHandle_t Test_Message_Queue =NULL;
QueueHandle_t Gyro_Message_Queue =NULL;//陀螺仪



/*******二值信号量用于任务同步*******/
SemaphoreHandle_t Test_BinarySem_handlet ;


