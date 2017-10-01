#include "stm32f4xx.h"
#include "usart.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "stm32f4xx_it.h"
#include "gpio.h"
#include "usart.h"
#include "math.h"
#include "stm32f4xx_usart.h"
#include "calculate.h"
#include "config.h"
#include "dma.h"
#include "spi.h"
void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,99,83,0,0);				
	
	SPI1_Init();
	SPI2_Init();
	SPI3_Init();
	CS_Config();
	
	USART1_DMA_INIT();
	

}
static uint8_t CPUUsage=0;
int main(void)
{
	init();
	
	while(1)
	{
		while(getTimeFlag())
		{
			readSensorData();
			#ifndef CORRECT
			run();
			#endif
			debugMode();
			CPUUsage=getTimeCount();
		}
	}
}

