#include "temperature_control.h"
#include "motion_attitude_algorithm.h"
#include "timer.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "maa_config.h"
#include "math.h"
#include "usart.h"


void temperature_control(float temp)
{
	static float icm_temp;

	icm_read_temp(&icm_temp);
	
	temp_pid_ctr(ICM20608G, temp,icm_temp);
}
void temp_pid_ctr(uint8_t device,float val_ex,float val_or)
{
	static float err;
	static float err_sum[2];
	static float err_last[2];
	static float err_v;
	
	uint8_t ch;
	
	static double ctr;
	if(device==ICM20608G)
		ch=1;
	else if(device==LIS3MDL)
		ch=0;
	else
    while(1);
	
	err=val_ex-val_or;

	err_sum[ch]=err_sum[ch]+err;
	err_v=err-err_last[ch];
	err_last[ch]=err;
	

	if(err_sum[ch]>70.0f/Ki)
		err_sum[ch]=70.0f/Ki;
	if(err_sum[ch]<-70.0f/Ki)
		err_sum[ch]=-70.0f/Ki;
			
	if(ctr<0)
	{
		ctr=0;
	}
	else
	{
		ctr=Kp*err+Ki*err_sum[ch]+Kd*err_v;
	}
	if(ctr>100)
	{
    ctr=100;
	}
	if(ch==0)
	{
	}
	if(ch==1)
	{
	 ICM_HeatingPower(ctr);
	}
}

//PB015
/**
  * @brief  初始化PWM波,用于陀螺仪温度控制
  *
  * @PB0  P
  * @PB1  PD12
  * @PB5  
  *
  * @param[in] arr,定时器分频
  * @param[in] psc
  * @retval None
  */
void pwm_init(uint32_t arr,uint32_t psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  				//TIM3时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3); 		//
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
	GPIO_Init(GPIOB,&GPIO_InitStructure);              
	
	
  TIM_TimeBaseStructure.TIM_Prescaler = psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3

	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2                                           //???????
 	TIM_OCInitStructure.TIM_Pulse=1000*0.05;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC3
	
	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR3上的预装载寄存器
	
	
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3	

	TIM_SetCompare3(TIM3,0.0*1000);		
}  


