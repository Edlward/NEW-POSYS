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
#include "can.h"
#include "elmo.h"
#include "config.h"
#include "dma.h"
void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,99,83,0,0);					//主周期定时5ms	
		//陀螺仪接收串口
	//USART1_DMA_INIT();
	
void GyroscopeUsartInit(uint32_t BaudRate);
	#ifndef SIMULATION
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);
	elmo_Init();
	elmo_Disable(1);
	elmo_Disable(2);
	elmo_Disable(3);
	//WirelessBluetoothUsartInit(115200);
	
	#endif
}
static uint8_t CPUUsage=0;
int main(void)
{
	init();
	
	while(1)
	{
		while(getTimeFlag())
		{
			#ifndef SIMULATION
			readSensorData();
			#endif
			run();
		  debugMode();
			CPUUsage=getTimeCount();
		}
	}
}

