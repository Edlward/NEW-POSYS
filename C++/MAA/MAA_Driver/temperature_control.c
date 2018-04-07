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
  * @brief  ��ʼ��PWM��,�����������¶ȿ���
  *
  * @PB0  P
  * @PB1  PD12
  * @PB5  
  *
  * @param[in] arr,��ʱ����Ƶ
  * @param[in] psc
  * @retval None
  */
void pwm_init(uint32_t arr,uint32_t psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  				//TIM3ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3); 		//
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
	GPIO_Init(GPIOB,&GPIO_InitStructure);              
	
	
  TIM_TimeBaseStructure.TIM_Prescaler = psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����ʱ��3

	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2                                           //???????
 	TIM_OCInitStructure.TIM_Pulse=1000*0.05;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC3
	
	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
	
	
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ��
	
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3	

	TIM_SetCompare3(TIM3,0.0*1000);		
}  


