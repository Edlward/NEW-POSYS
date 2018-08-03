#ifndef __GPIO_H

#define __GPIO_H

#include "stm32f4xx_gpio.h"
#include "config.h"

#define LED1_ON 			GPIO_SetBits(GPIOB, GPIO_Pin_0);
#define LED1_OFF		  GPIO_ResetBits(GPIOB, GPIO_Pin_0);
#define LED2_ON			  GPIO_SetBits(GPIOB, GPIO_Pin_1);
#define LED2_OFF 			GPIO_ResetBits(GPIOB, GPIO_Pin_1);
 
void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);
	
void Led_Init(void);
#endif
