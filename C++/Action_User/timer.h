#ifndef __timer_h
#define __timer_h

#include "stm32f4xx_tim.h"


void TIM_Init(TIM_TypeDef * TIMx, uint16_t arr, uint16_t psr, uint16_t prepri, uint16_t subpri); 
void TIM_Delayms(TIM_TypeDef * TIMx, uint32_t DelayMs);
void TIM_Delayus(TIM_TypeDef * TIMx, uint16_t Delayus);
void TIM_Delay100us(TIM_TypeDef * TIMx, uint16_t Delay100us);
void Delay_us(uint32_t nTime);
void Delay_ms(uint32_t nTime);
void set_time_sig(void);
uint8_t get_time_sig(void);
#endif



