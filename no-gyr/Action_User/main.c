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

void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,999,83,0,0);					//主周期定时5ms	
		//陀螺仪接收串口
	GyroscopeUsartInit(115200);
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);
	
	elmo_Init();
	elmo_Disable(1);
	elmo_Disable(2);
	elmo_Disable(3);
		//蓝牙串口
	WirelessBluetoothUsartInit(115200);
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
			run();
			debugMode();
			CPUUsage=getTimeCount();
		}
	}
}

