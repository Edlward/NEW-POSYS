#ifndef __TEMPERATURE_CONTROL_H
#define __TEMPERATURE_CONTROL_H

#include "stdint.h"

#define ICM_HeatingPower(a)  TIM_SetCompare3(TIM3,a/100.0*1000); 

int TempErgodic(void);
void pwm_init(uint32_t arr,uint32_t psc);
void temp_pid_ctr(float val_ex);
uint32_t Heating(void);
#endif // !__TEMPERATURE_CONTROL_H


