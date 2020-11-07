#ifndef  MESSAGE_QUEUE_H
#define  MESSAGE_QUEUE_H
#include "FreeRTOS.h"
#include "stdio.h"
#include "queue.h"
#include "semphr.h"

extern QueueHandle_t Test_Message_Queue;

extern SemaphoreHandle_t Test_BinarySem_handlet ;

#define Test_QUEUE_LEN  4 //队列长度，包含消息的个数
#define Test_QUEUE_SIZE 4 //消息的大小，单位：字节


#endif



