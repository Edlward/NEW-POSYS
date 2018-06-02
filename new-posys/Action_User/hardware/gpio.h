#ifndef __GPIO_H

#define __GPIO_H

#include "stm32f4xx_gpio.h"
#include "config.h"

#define LED_ON 				GPIO_SetBits(GPIOA, GPIO_Pin_3);GPIO_ResetBits(GPIOA, GPIO_Pin_2);
#define LED_OFF 			GPIO_ResetBits(GPIOA, GPIO_Pin_3);GPIO_SetBits(GPIOA, GPIO_Pin_2);
 
void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);
	
void Led_Init(void);
#endif
