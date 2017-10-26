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
#include "stdlib.h"
#include "stdio.h"
void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,499,839,0,0);					//主周期定时10ms	
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_9, GPIO_Pin_8);
	
	USART1_Init(921600);//电脑
	UART5_Init(921600);//定位系统
	
	elmo_Init();
	elmo_Enable(2);
	Vel_cfg(2,300000,300000);
}
extern int posFlag;
int main(void)
{
  uint32_t pos_old=0;
	uint32_t pos=0;
  float vel_real=0.0f;
  uint8_t circle=0;
	init();
	while(1)
	{
		while(getTimeFlag())
		{
			char buffer[10];
			ReadActualPos(2);
			pos=GetRealPos();
			vel_real=pos-pos_old;
			if(vel_real>20480000)
				vel_real-=40960000;
			if(vel_real<-20480000)
				vel_real+=40960000;
			vel_real=vel_real/40960000*360;
			pos_old=pos;
//			pos-=circle*40960000;
//				if(pos_old<40960000&&pos>40960000){
//				circle++;
//				USART_SendData(USART3,'S');
			}
		}
	}






