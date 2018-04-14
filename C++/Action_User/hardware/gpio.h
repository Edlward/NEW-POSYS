#ifndef __GPIO_H
#define __GPIO_H
#ifdef __cplusplus  
extern "C"
{
#endif
#include "stm32f4xx_gpio.h"
void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);
void Led_Init(void);
void pwm_init(uint32_t arr,uint32_t psc);
#ifdef __cplusplus  
}
#endif
#endif


