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
	USART3_Init(115200);
	
	CAN_Config(CAN1,500,GPIOA,GPIO_Pin_11, GPIO_Pin_12);
	elmo_Init();
		
	elmo_Enable(2);
	Vel_cfg(2,300000,300000);
	
}
extern int posFlag;
int main(void)
{
	init();
	
	while(1)
	{
	while(getTimeFlag())
	{
		static int32_t vel_old=0;
		static uint32_t pos_old=0;
		static uint8_t circle=0;
		char buffer[10];
		int32_t vel=0;
		float vel_f=0.0f;
		uint32_t pos=0;
		ReadActualVel(2);
		ReadActualPos(2);
		vel=getCommand();
		pos=GetPos();
		if(abs(vel)>15*4096*10000/360)
			vel=0;
		if(vel!=vel_old)
	  VelCrl(2, vel);
		vel_old=vel;
		pos-=circle*40960000;
		if(pos_old<40960000&&pos>40960000){
			circle++;
			USART_SendData(USART3,'S');
		}
		pos_old=pos;
		vel_f=GetVal()*360.f/4096.f/10000.f;
//		sprintf( buffer, "%f", vel_f );
//	  USART_OUT(USART3,(uint8_t*)buffer);
////		if(posFlag)
//		{
//	  USART_OUT(USART3,(uint8_t*)"\t");
//		sprintf( buffer, "%d", pos );
//	  USART_OUT(USART3,(uint8_t*)buffer);
//	  USART_OUT(USART3,(uint8_t*)"\r\n");
//		}
//		else{
//	  USART_OUT(USART3,(uint8_t*)"\r\n");
//		}
		
	}
	}
}

