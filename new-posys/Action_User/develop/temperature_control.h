#ifndef __TEMPERATURE_CONTROL_H
#define __TEMPERATURE_CONTROL_H

#include "stdint.h"

#define ICM_HeatingPower(a)  TIM_SetCompare3(TIM3,a/100.0*1000); 

int TempErgodic(float minTemp,float scale,float minute);
void pwm_init(uint32_t arr,uint32_t psc);
void temp_pid_ctr(float val_ex,float val_or);
uint32_t Heating(void);
void temperature_control(float temp);
#endif // !__TEMPERATURE_CONTROL_H


