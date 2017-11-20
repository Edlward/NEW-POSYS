#ifndef __TEMPERATURE_CONTROL_H
#define __TEMPERATURE_CONTROL_H

#include "stdint.h"

#define ICM_1_HeatingPower(a)  TIM_SetCompare3(TIM2,(uint32_t)((double)a/100.0*1000)); 
#define ICM_2_HeatingPower(a)  TIM_SetCompare4(TIM2,(uint32_t)((double)a/100.0*1000)); 
#define ICM_3_HeatingPower(a)  TIM_SetCompare2(TIM3,(uint32_t)((double)a/100.0*1000)); 

int TempErgodic(int reset);
void pwm_init(uint32_t arr,uint32_t psc);
void temp_pid_ctr(float val_ex);
uint32_t Heating(void);
#endif // !__TEMPERATURE_CONTROL_H


