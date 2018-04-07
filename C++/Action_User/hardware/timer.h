#ifndef __timer_h
#define __timer_h

#ifdef __cplusplus  
extern "C"
{
#endif
	
#include "stm32f4xx_tim.h"

#define  SYSCLK        168        
#define  A             3           
#define  B             3           
#define  delay_us(nus)   wait(((nus)*(SYSCLK)-(B))/(A))
#define  delay_ms(nms)   delay_us((nms)*1000)
#define  delay_s(ns)     delay_ms((ns)*1000)
	
	
#define TIMER_DELAY(a)	TIM_Delayms(TIM1,a)
#define TIMER_DELAY_US(a) TIM_Delayus(TIM1,a)
	
#define ADXRS453_UPDATE_FREQ 5	

/*main procedure cycle period*/
#define PERIOD    				(200)
	

void  wait(uint32_t n);

void TIM_Init(TIM_TypeDef * TIMx, uint16_t arr, uint16_t psr,uint16_t prepri,uint16_t subpri); 

void TIM7_IRQHandler(void);
uint8_t getTimeFlag(void);	
void Delay_us(uint32_t nTime);
void Delay_ms(uint32_t nTime);
#ifdef __cplusplus 
}
#endif	
	
	
#endif



