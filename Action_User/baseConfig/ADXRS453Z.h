#ifndef __ADXRS453_H
#define __ADXRS453_H
#include "config.h"
// Mode
#define WRITE                0x00
#define READ								 0x01

#define   ADI_RATE1   0x00
#define   ADI_RATE0   0x01
#define   ADI_TEM1    0x02
#define   ADI_TEM0    0x03
#define   ADI_LOSCT1  0x04
#define   ADI_LOSCT0  0x05
#define   ADI_HICST1  0x06
#define   ADI_HICST0  0x07
#define   ADI_QUAD1   0x08
#define   ADI_QUAD0   0x09
#define   ADI_FAULT1  0x0A
#define   ADI_FAULT0  0x0B
#define   ADI_PID1    0x0C
#define   ADI_PID0    0x0D
#define   ADI_SN3     0x0E
#define   ADI_SN2     0x0F
#define   ADI_SN1     0x10
#define   ADI_SN0     0x11

void ADXRS453Z_init(void);
void ADI_UpdateData(float * gyr_temp,float * temp_temp);
void ADXRS453StartUp(void);
int ADXRS453SingleRead(unsigned char Address);
int  ADXRS453SensorData(void);
#endif

