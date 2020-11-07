#ifndef _SBUS_H
#define _SBUS_H

void Process(uint8_t *raw,uint16_t* result);
bool SBUS_Linear_Calibration(void);


extern uint16_t SBUS_Channel[16];
extern uint8_t Sbus_Receive_Flag;

#endif

