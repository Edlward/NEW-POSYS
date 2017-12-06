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
#include "can.h"
#include "elmo.h"



Robot_t gRobot;
extern int ready;
void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,99,83,0,0);				
	
	GYR_Init(921600);
	DebugBLE_Init(921600);
//	while(!gRobot.posSystemReady);
	
//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);
//	
//	ElmoInit(CAN1);
//	MotorOff(CAN1,1);
//	MotorOff(CAN1,2);
//	MotorOff(CAN1,3);
//	MotorOff(CAN1,4);
	
		Delay_ms(1000);
	while(ready==0)
	{
		Delay_ms(5);
		USART_OUT(USART3,"AT+begin\r\n");
	}
}

static uint8_t CPUUsage=0;
int main(void)
{
	init();
	
	while(1)
	{
		{
			//readSensorData();
			//run();
			CPUUsage=getTimeCount();
		}
	}
}






