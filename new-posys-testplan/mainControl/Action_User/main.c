#include "stm32f4xx.h"
#include "usart.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "stm32f4xx_it.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "math.h"
#include "stm32f4xx_usart.h"
#include "arm_math.h"

void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,4999,83,0,0);					//主周期定时10ms	
	
	USART6_Init(115200);//电脑
	USART3_Init(115200);//转台
	UART5_Init(115200);//定位系统
	
}

int main(void)
{
	init();
	
	while(1)
	{
	while(getTimeFlag())
	{
		
	}
	}
	}

