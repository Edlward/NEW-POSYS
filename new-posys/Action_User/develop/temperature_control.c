#include "temperature_control.h"
#include "figureAngle.h"
#include "timer.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "config.h"
#include "arm_math.h"
#include "usart.h"



/*
�ֱ������豸����,�����¶�,��ʵ�¶�
*/

extern float  temp_icm;
void temp_pid_ctr(float val_ex)
{
  static float err;
  static float err_sum;
  static float err_last;
  static float err_v;
  float Kp_summer = 13.0f;
  float Ki_summer = 0.007f;
  float Kd_summer = 0.00f;
  
  static double ctr;
  /*���*/
  err=val_ex-temp_icm;
  /*����*/
  err_sum=err_sum+err;
  /*΢����*/
  err_v=err-err_last;
  err_last=err;
  
  if(val_ex-temp_icm>3)
    Kp_summer=16;
  /*
#define Kp  15.0f
#define Ki  0.007f
#define Kd  0.02f
  */
  /*����������ֵ*/
  if(err_sum>70.0f/Ki_summer)
    err_sum=70.0f/Ki_summer;
  if(err_sum<-70.0f/Ki_summer)
    err_sum=-70.0f/Ki_summer;
  
  if(ctr<0)
  {
    ctr=0;
  }
  else
  {
    ctr=Kp_summer*err+Ki_summer*err_sum+Kd_summer*err_v;
  }
  /*��������*/
  if(ctr>60)
  {
    ctr=60;
  }
  /*#define ICM_HeatingPower(a)  TIM_SetCompare3(TIM3,a/100.0*1000); */
  /*֮�������ֵΪ1000,����Ϊ�ö�ʱ����װ��ֵΪ1000*/
  ICM_HeatingPower(ctr);
}

//uint32_t Heating(void){
//  float icm_temp;
//	static float temp_init=0.0f;
//	static int ignore=0;
//	static double sum=0.0;
//	
//	icm_update_temp();
//	icm_read_temp(&icm_temp);
//	ignore++;
//	if(ignore<101){
//		temp_init+=icm_temp;
//		return 0u;
//	}
//	else if(ignore==101){
//		temp_init/=100.f;
//	}else 
//		ignore=101;
//	
//	temp_pid_ctr(temp_init+2,icm_temp);
//	
//	if(icm_temp>temp_init+2){
//		return 1u;
//	}else
//		return 0u;
//}
//#define MODE_ONE
#ifdef MODE_ONE
int TempErgodic(int reset){
  
  static uint32_t circle_count=0;
  static uint8_t flag=0;
  if(reset)  flag=0;
  switch(flag){
  case 0:
    circle_count++;
    break;
  case 1:
    circle_count--;
    break;
  }
  temp_pid_ctr(TempTable_min+(TempTable_max-TempTable_min)*circle_count*PERIOD/(float)HEATTIME/60.f);
  if(circle_count==(int)(HEATTIME*60.f/PERIOD)){
    flag=1;
    //USART_OUT(USART1,"finish rise\r\n");
  }else if(circle_count==0){
    //USART_OUT(USART1,"finish decrease\r\n");
    flag=3;
  }
  return flag;
}
#else 
/*��λ����*/
#define TIME_BEAR	10
/*ÿ��TIME_BEAR���ж�һ�Σ�������ʱʱ�̶̿���ǰһ�����¶��жϣ������Ļ���ӵúܿ죩
���һ����֮���¶�С��0.1�㣬�Ǿ͸ı�PWM
��ô����Ϊ�˲����£����»���ɲ��ȶ���ʱ��Ч�������󣬿������˲��ͺ�Ҳ������*/
int TempErgodic(int reset){
  
  static uint32_t success=0;
  static int direction=1;
  static float PWM=0.1f;
  static float temp_last;
  static uint32_t time = 0;
	if(reset) success=0;
  /*200/s,12000/min*/
  time++;
  /*ÿ��TIME_BEAR���ж�һ��*/
  if(time%(TIME_BEAR*200)==0)
  {
    switch(direction)
    {
    case -1:
      if((temp_icm-temp_last)>=-0.1f)
      {
        if(PWM>0.1f)
          PWM-=0.1f;
        else if(PWM>0.f)
          PWM=0.f;
        else if(PWM==0.f)
				{
          direction=1;
					PWM+=0.1f;
					success=3;
				}
      }
      break;
    case 1:
      if((temp_icm-temp_last)<=0.1f)
      {
        if(PWM<99.9f)
          PWM+=0.1f;
        else
          PWM=100.f;
        break;
      }
    }
    temp_last=temp_icm;
  }
  if(temp_icm>TempTable_max) 
    direction=-1;
  else if(temp_icm<TempTable_min)
	{
		/*�ų�����ʼ�¶�С����Сֵ�����*/
		if(direction==-1)
			success=3;
		
    direction=1;
	}
  ICM_HeatingPower(PWM);
  return success;
}
#endif

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
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1       
  TIM_OCInitStructure.TIM_Pulse=1000*0.05;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC3
  
  
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
  
  
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ��
  
  TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3	
  
  TIM_SetCompare3(TIM3,0.0*1000);		
}  


