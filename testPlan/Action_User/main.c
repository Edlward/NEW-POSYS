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
uint32_t posInit=0;
extern uint32_t Pos;
void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,499,839,0,0);					//
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_9, GPIO_Pin_8);
	
	UART5_Init(921600);//电脑
	USART1_Init(921600);//定位系统
	
	elmo_Init();
	elmo_Enable(2);
	ReadActualPos(2);
	Vel_cfg(2,300000,300000);
	delay_ms(500);
	posInit=GetRealPos();
}
extern int posFlag;
extern struct{
	float anglex;
	float angley;
	float anglez;
	float wx;
	float wy;
	float wz;
}para_t;
  float angle=0.f;
int main(void)
{
  uint32_t pos_old=0;
  int64_t  delta_pos=0;
	double vel_real=0.f;
	init();
	
	VelCrl(2, 1.0*4096.0*10201.0/360.0);
	
	while(1)
	{
		while(getTimeFlag())
		{
				char buffer[10];
				ReadActualPos(2);
			
				VelCrl(2,0.0*4096.0*10201.0/360.0);
			  delta_pos=Pos-posInit;
				if(delta_pos>0xffffffff/2)
					delta_pos-=0xffffffff;
				if(delta_pos<-0xffffffff/2)
					delta_pos+=0xffffffff;
				angle=delta_pos/40960000.0*360.0;
				delta_pos=Pos-pos_old;
				if(delta_pos>2048*10201)
					delta_pos-=4096*10201;
				if(delta_pos<-2048*10201)
					delta_pos+=4096*10201;
				vel_real=delta_pos*0.00172317664934810312714439760808;
				pos_old=Pos;
					USART_OUT_F(angle);
					USART_OUT_F(vel_real);
					USART_OUT_F(para_t.anglex);
					USART_OUT_F(para_t.angley);
					USART_OUT_F(para_t.anglez);
					USART_OUT_F(para_t.wx);
					USART_OUT_F(para_t.wy);
					USART_OUT_F(para_t.wz);
					USART_Enter();
		}
	}
}