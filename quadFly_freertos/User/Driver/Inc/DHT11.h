#ifndef _DHT11_H_
#define _DHT11_H_


u8 Dht11Init(void);//��ʼ��DHT11  
  
u8 Dht11ReadData(u8 *temp,u8 *humi);//��ȡ��ʪ��  
  
u8 Dht11ReadByte(void);//����һ���ֽ�  
  
u8 Dht11ReadBit(void);//����һ��λ  
  
  
u8 Dht11Check(void);//����Ƿ����DHT11  
  
  
void Dht11Rst(void);//��λDHT11    
  
void Dht11Show(void); 
void DHT11_TimeRead(void);

extern float DHT11_Data[2];
#endif

