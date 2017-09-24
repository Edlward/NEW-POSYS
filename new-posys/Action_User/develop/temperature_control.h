#ifndef __TEMPERATURE_CONTROL_H
#define __TEMPERATURE_CONTROL_H

#include "stdint.h"

#define ICM_HeatingPower(a)  TIM_SetCompare3(TIM3,a/100.0*1000); 

#define Kp_summer  16.0f
#define Ki_summer  0.007f
#define Kd_summer  0.00f

int TempErgodic(float minTemp,float scale,int minute);
void pwm_init(uint32_t arr,uint32_t psc);
void temperature_control(float temp);
void temp_pid_ctr(float val_ex,float val_or);

#endif // !__TEMPERATURE_CONTROL_H


