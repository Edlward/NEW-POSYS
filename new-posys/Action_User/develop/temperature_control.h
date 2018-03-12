#ifndef __TEMPERATURE_CONTROL_H
#define __TEMPERATURE_CONTROL_H

#include "stdint.h"
#include "config.h"
void ICM_HeatingPower(int gyroNum,int value);
int TempErgodic(int gyroNum,int reset);
void pwm_init(uint32_t arr,uint32_t psc);
void temp_pid_ctr(int gyro,float val_ex);
uint32_t Heating(void);
double LowPassFilter(float newValue,int gyro);
int HeatingInit(float temp_temp[GYRO_NUMBER]);
int getTempInitSuces(void);
void SetTempInitSuces(void);
#endif // !__TEMPERATURE_CONTROL_H


