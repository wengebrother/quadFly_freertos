#ifndef  MESSAGE_QUEUE_H
#define  MESSAGE_QUEUE_H
#include "FreeRTOS.h"
#include "stdio.h"
#include "queue.h"
#include "semphr.h"

extern QueueHandle_t Test_Message_Queue;

extern SemaphoreHandle_t Test_BinarySem_handlet ;

#define Test_QUEUE_LEN  4 //���г��ȣ�������Ϣ�ĸ���
#define Test_QUEUE_SIZE 4 //��Ϣ�Ĵ�С����λ���ֽ�


#endif



