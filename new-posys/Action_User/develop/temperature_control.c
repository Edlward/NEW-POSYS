#include "temperature_control.h"
#include "figureAngle.h"
#include "timer.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "config.h"
#include "arm_math.h"
#include "usart.h"



/*
分别输入设备名称,期望温度,真实温度
*/

extern float  temp_icm;
void temp_pid_ctr(float val_ex)
{
  static float err;
  static float err_sum[2];
  static float err_last[2];
  static float err_v;
  float Kp_summer = 10.0f;
  float Ki_summer = 0.007f;
  float Kd_summer = 0.00f;
  uint8_t ch;
  
  static double ctr;
  ch=1;
  /*误差*/
  err=val_ex-temp_icm;
  /*积分*/
  err_sum[ch]=err_sum[ch]+err;
  /*微分量*/
  err_v=err-err_last[ch];
  err_last[ch]=err;
  
  if(val_ex-temp_icm>3)
    Kp_summer=16;
  /*
#define Kp  15.0f
#define Ki  0.007f
#define Kd  0.02f
  */
  /*积分量的阈值*/
  if(err_sum[ch]>70.0f/Ki_summer)
    err_sum[ch]=70.0f/Ki_summer;
  if(err_sum[ch]<-70.0f/Ki_summer)
    err_sum[ch]=-70.0f/Ki_summer;
  
  if(ctr<0)
  {
    ctr=0;
  }
  else
  {
    ctr=Kp_summer*err+Ki_summer*err_sum[ch]+Kd_summer*err_v;
  }
  /*调节上限*/
  if(ctr>40)
  {
    ctr=40;
  }
  if(ch==0)
  {
  }
  if(ch==1)
  {
    /*#define ICM_HeatingPower(a)  TIM_SetCompare3(TIM3,a/100.0*1000); */
    /*之所以最大值为1000,是因为该定时器的装载值为1000*/
    ICM_HeatingPower(ctr);
  }
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
int TempErgodic(void){
  
  static uint32_t circle_count=0;
  static uint8_t flag=0;
  
  switch(flag){
  case 0:
    circle_count++;
    temp_pid_ctr(TempTable_min+(TempTable_max-TempTable_min)*circle_count*PERIOD/(float)HEATTIME/60.f);
    break;
  case 1:
    circle_count--;
    temp_pid_ctr(TempTable_min+(TempTable_max-TempTable_min)*circle_count*PERIOD/(float)HEATTIME/60.f);
    break;
  }
  if(circle_count==(int)(HEATTIME*60.f/PERIOD)){
    flag=1;
  }else if(circle_count==0){
    flag=3;
  }
  return flag;
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
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1       
  TIM_OCInitStructure.TIM_Pulse=1000*0.05;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC3
  
  
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR3上的预装载寄存器
  
  
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能
  
  TIM_Cmd(TIM3, ENABLE);  //使能TIM3	
  
  TIM_SetCompare3(TIM3,0.0*1000);		
}  


