#ifndef __HMC5883_H_
#define __HMC5883_H_


//*****B寄存器配置增益量程***********/
#define MAG_GAIN_SCALE0 1370//0x00   0.88Ga
#define MAG_GAIN_SCALE1 1090//0x20   1.30Ga
#define MAG_GAIN_SCALE2 820//0x40    1.90Ga
#define MAG_GAIN_SCALE3 660//0x60    2.50Ga
#define MAG_GAIN_SCALE4 440//0x80    4.00Ga
#define MAG_GAIN_SCALE5 390//0xA0    4.70Ga
#define MAG_GAIN_SCALE6 330//0xC0    5.66Ga
#define MAG_GAIN_SCALE7 230//0xE0    8.10Ga


extern float Mag_Data[3];

#define N2 10
extern float Data_X_MAG[N2];
extern float Data_Y_MAG[N2];
extern float Data_Z_MAG[N2];

float GildeAverageValueFilter_MAG(float NewValue,float *Data);
#define	HMC5883L_Addr   0x3C	//磁场传感器器件地址
void HMC5883L_Initial(void);
void HMC5883L_Read(void);
void HMC5883L_StateMachine(void);

extern float MAGData[3];
extern uint8_t Extern_Mag_Work_Flag;


#endif


