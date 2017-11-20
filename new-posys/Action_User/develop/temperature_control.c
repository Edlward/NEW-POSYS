
#include "config.h"



/*
分别输入设备名称,期望温度,真实温度
*/

extern AllPara_t allPara;
/*
分别输入设备名称,期望温度,真实温度
*/

void temp_pid_ctr(float val_ex)
{
  static float err;
  static float err_sum;
  static float err_last;
  static float err_v;
  float Kp_summer = 12.0f;
  float Ki_summer = 0.00125f;
  float Kd_summer = 900.0f;
  
  static double ctr;
  /*误差*/
  err=val_ex-allPara.GYRO_Temperature;
  /*积分*/
  err_sum=err_sum+err;
  /*微分量*/
  err_v=err-err_last;
  err_last=err;
  
//  /*积分量的阈值为70*/
//  if(err_sum>70.0f/Ki_summer)
//    err_sum=70.0f/Ki_summer;
//  if(err_sum<-70.0f/Ki_summer)
//    err_sum=-70.0f/Ki_summer;
//  
  if(ctr<0)
  {
    ctr=0;
  }
  else
  {
    ctr=Kp_summer*err+Ki_summer*err_sum+Kd_summer*err_v;
		/*调节上限*/
		if(ctr>100)
		{
			ctr=100;
		}
  }
	
  /*#define ICM_HeatingPower(a)  TIM_SetCompare3(TIM3,a/100.0*1000); */
  /*之所以最大值为1000,是因为该定时器的装载值为1000*/
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
/*单位是秒*/
#define TIME_BEAR	20
/*每隔TIME_BEAR秒判断一次，而不是时时刻刻都和前一分钟温度判断（那样的话会加得很快）
如果一分钟之间温度小于0.1°，那就改变PWM
这么做是为了不控温，控温会造成不稳定，时滞效果会增大，卡尔曼滤波滞后也会增大*/
int TempErgodic(int reset){
  
  static uint32_t success=0;
  static int direction=1;
  static float PWM=0.f;
  static float temp_last;
  static uint32_t time = 0;
	if(reset) success=0;
  /*200/s,12000/min*/
  time++;
  /*每隔TIME_BEAR秒判断一次*/
  if(time%(TIME_BEAR*200)==0)
  {
    switch(direction)
    {
    case -1:
      if((allPara.GYRO1_Temperature-temp_last)>=-0.1f)
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
      if((allPara.GYRO1_Temperature-temp_last)<=0.1f)
      {
        if(PWM<99.9f)
				{
					if((double)allPara.GYRO1_Temperature>TempTable_min)
						PWM+=0.1f;
					else
						PWM+=0.5f;
				}
        else
          PWM=100.f;
        break;
      }
    }
    temp_last=allPara.GYRO1_Temperature;
  }
  if((double)allPara.GYRO1_Temperature>TempTable_max) 
    direction=-1;
  else if((double)allPara.GYRO1_Temperature<TempTable_min)
	{
		/*排除掉起始温度小于最小值的情况*/
		if(direction==-1)
			success=3;
		
    direction=1;
	}
  ICM_1_HeatingPower(PWM);
  ICM_2_HeatingPower(PWM);
  ICM_3_HeatingPower(PWM);
  return success;
}
#endif




void pwm1_init(uint32_t arr,uint32_t psc)
{		 					 
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  				//TIM2时钟使能
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2); 		//
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
  GPIO_Init(GPIOB,&GPIO_InitStructure);              
  
  
  TIM_TimeBaseStructure.TIM_Prescaler = psc;  //定时器分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
  TIM_TimeBaseStructure.TIM_Period = arr;   //自动重装载值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//初始化定时器3
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1       
  TIM_OCInitStructure.TIM_Pulse=1000*0.05;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC3
  
  
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR3上的预装载寄存器
  
  
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPE使能
  
  TIM_Cmd(TIM2, ENABLE);  //使能TIM2	
  
  TIM_SetCompare3(TIM2,0.0*1000);		
}  
void pwm2_init(uint32_t arr,uint32_t psc)
{		 					 
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  				//TIM2时钟使能
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2); 		//
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
  GPIO_Init(GPIOB,&GPIO_InitStructure);              
  
  
  TIM_TimeBaseStructure.TIM_Prescaler = psc;  //定时器分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
  TIM_TimeBaseStructure.TIM_Period = arr;   //自动重装载值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//初始化定时器3
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1       
  TIM_OCInitStructure.TIM_Pulse=1000*0.05;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC3
  
  
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR3上的预装载寄存器
  
  
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPE使能
  
  TIM_Cmd(TIM2, ENABLE);  //使能TIM2	
  
  TIM_SetCompare4(TIM2,0.0*1000);		
}  
void pwm3_init(uint32_t arr,uint32_t psc)
{		 					 
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  				//TIM3时钟使能
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 		//
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
  GPIO_Init(GPIOC,&GPIO_InitStructure);              
  
  
  TIM_TimeBaseStructure.TIM_Prescaler = psc;  //定时器分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
  TIM_TimeBaseStructure.TIM_Period = arr;   //自动重装载值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1       
  TIM_OCInitStructure.TIM_Pulse=1000*0.05;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC3
  
  
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR3上的预装载寄存器
  
  
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能
  
  TIM_Cmd(TIM3, ENABLE);  //使能TIM3	
  
  TIM_SetCompare2(TIM3,0.0*1000);		
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
  pwm1_init(arr, psc);
	pwm2_init(arr, psc);
	pwm3_init(arr, psc);
}  


