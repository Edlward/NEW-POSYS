
#include "config.h"

/*
分别输入设备名称,期望温度,真实温度
*/

extern AllPara_t allPara;
/*
分别输入设备名称,期望温度,真实温度
*/

void ICM_HeatingPower(int gyroNum,int value)
{
	switch(gyroNum)
	{
		case 0:
			TIM_SetCompare3(TIM2,(uint32_t)((double)value/100.0*1000));
		break;
		case 1:
			TIM_SetCompare4(TIM2,(uint32_t)((double)value/100.0*1000));
		break;
		case 2:
			TIM_SetCompare2(TIM3,(uint32_t)((double)value/100.0*1000));
		break;
	}
}
static int tempInitSuces=0;
int HeatingInit(float temp_temp[GYRO_NUMBER])
{
	static int time=0;
	time++;
	
		for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
		{
			allPara.GYRO_Temperature[gyro]=LowPassFilter(temp_temp[gyro],gyro)/100.f;
		}
	if(time==101)
	{
		for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
		{
			allPara.GYRO_TemperatureAim[gyro]=allPara.GYRO_Temperature[gyro]+0.05f;
//			USART_OUT_F(allPara.GYRO_TemperatureAim[gyro]);
		}
//		USART_Enter();
//		USART_Enter();
//		USART_Enter();
		tempInitSuces=1;
		return 1;
	}
	return 0;
}

int getTempInitSuces(void)
{
	return tempInitSuces;
}

void temp_pid_ctr(int gyro,float val_ex)
{
  static float err[GYRO_NUMBER];
  static float err_sum[GYRO_NUMBER];
  static float err_last[GYRO_NUMBER];
  static float err_v[GYRO_NUMBER];
  float Kp_summer[GYRO_NUMBER] = { 2550.0f , 2550.0f ,2550.0f };
  float Ki_summer[GYRO_NUMBER] = { 0.75f , 0.75f ,0.75f };
  float Kd_summer[GYRO_NUMBER] = { 0.0f , 0.0f ,0.0f };
  
  static double ctr[GYRO_NUMBER];
	
		/*误差*/
		err[gyro]=val_ex-allPara.GYRO_Temperature[gyro];
		/*积分*/
		err_sum[gyro]=err_sum[gyro]+err[gyro];
		/*微分量*/
		err_v[gyro]=err[gyro]-err_last[gyro];
		err_last[gyro]=err[gyro];

			ctr[gyro]=Kp_summer[gyro]*err[gyro]+Ki_summer[gyro]*err_sum[gyro]+Kd_summer[gyro]*err_v[gyro];
			/*调节上限*/
			if(ctr[gyro]>50)
			{
				ctr[gyro]=100;
			}
			if(ctr[gyro]<0)
			{
				ctr[gyro]=0;
			}
		
		/*#define ICM_HeatingPower(a)  TIM_SetCompare3(TIM3,a/100.0*1000); */
		/*之所以最大值为1000,是因为该定时器的装载值为1000*/
		ICM_HeatingPower(gyro,ctr[gyro]);
	
}


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
  temp_pid_ctr(TempTable_min*100.0+(TempTable_max*100.0-TempTable_min*100.0)*circle_count*PERIOD/(float)HEATTIME/60.f);
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
#define TIME_BEAR	15
/*每隔TIME_BEAR秒判一次，而不是时时刻刻都和前一分钟温度判断（那样的话会加得很快）
如果一分钟之间温度小于0.1°，那就改变PWM
这么做是为了不控温，控温会造成不稳定，时滞效果会增大，卡尔曼滤波滞后也会增大*/
int TempErgodic(int gyroNum,int reset){
  static uint32_t success[GYRO_NUMBER];
  static int direction[GYRO_NUMBER]={1,1,1};
  static float PWM[GYRO_NUMBER]={0.f,0.f,0.f};
  static float temp_last[GYRO_NUMBER];
  static uint32_t time[GYRO_NUMBER] = { 0 };
	if(reset) success[gyroNum]=0;
  /*200/s,12000/min*/
  time[gyroNum]++;
  /*每隔TIME_BEAR秒判断一次*/
  if(time[gyroNum]%(int)(TIME_BEAR*200)==0)
  {
    switch(direction[gyroNum])
    {
    case -1:
      if((allPara.GYRO_Temperature[gyroNum]-temp_last[gyroNum])>=-0.1f)
      {
        if(PWM[gyroNum]>0.1f)
          PWM[gyroNum]-=0.1f;
        else if(PWM[gyroNum]>0.f)
          PWM[gyroNum]=0.f;
        else if(PWM[gyroNum]==0.f)
				{
          direction[gyroNum]=1;
					PWM[gyroNum]+=0.1f;
					success[gyroNum]=3;
				}
      }
      break;
    case 1:
      if((allPara.GYRO_Temperature[gyroNum]-temp_last[gyroNum])<=0.1f)
      {
        if(PWM[gyroNum]<99.9f)
				{
					if((double)allPara.GYRO_Temperature[gyroNum]>TempTable_min)
						PWM[gyroNum]+=0.1f;
					else
						PWM[gyroNum]+=0.5f;
				}
        else
          PWM[gyroNum]=100.f;
        break;
      }
    }
    temp_last[gyroNum]=allPara.GYRO_Temperature[gyroNum];
  }
  if((double)allPara.GYRO_Temperature[gyroNum]>TempTable_max) 
    direction[gyroNum]=-1;
  else if((double)allPara.GYRO_Temperature[gyroNum]<TempTable_min)
	{
		/*排除掉起始温度小于最小值的情况*/
		if(direction[gyroNum]==-1)
			success[gyroNum]=3;
		
    direction[gyroNum]=1;
	}
  ICM_HeatingPower(gyroNum,PWM[gyroNum]);
  return success[gyroNum];
}
#endif

#define THRESHOLD_TEMP 		 0.01f
#define THRESHOLD_COUNT  	 100
#define STD_VALUE					 0.2f
float LowPassFilter(float newValue,int gyro)
{
	static uint8_t Dr_flagLst[GYRO_NUMBER] = { 0 };
	static uint8_t Dr_flag[GYRO_NUMBER] = { 0 };
	static uint8_t F_count[GYRO_NUMBER] = { 0 };
	static float coeff[GYRO_NUMBER] = { 0.f };
	static float valueLast[GYRO_NUMBER] = { 0.f };
	float diffValue[GYRO_NUMBER] = { 0.f };
	if(valueLast[gyro]>newValue)
	{
		diffValue[gyro] = valueLast[gyro] - newValue;
    Dr_flag[gyro] = 0;
	}
	else
	{
    diffValue[gyro] = newValue - valueLast[gyro];
    Dr_flag[gyro] = 1;
	}		
  if (!(Dr_flag[gyro]^Dr_flagLst[gyro]))    //前后数据变化方向一致
	{
    F_count[gyro]=F_count[gyro]+2;
    if (diffValue[gyro] >= THRESHOLD_TEMP)
		{
			F_count[gyro]=F_count[gyro]+4;
		}
    if (F_count[gyro] >= THRESHOLD_COUNT)
      F_count[gyro] = THRESHOLD_COUNT;
    coeff[gyro] = STD_VALUE * F_count[gyro];
	}
  else{
    coeff[gyro] = STD_VALUE;
    F_count[gyro] = 0;
  }
  //一阶滤波算法
  if (Dr_flag[gyro] == 0)     //当前值小于前一个值
    newValue = valueLast[gyro] - coeff[gyro]*(valueLast[gyro] - newValue) / 256;
  else
		newValue = valueLast[gyro] + coeff[gyro]*(newValue - valueLast[gyro]) / 256;
	
  valueLast[gyro]=newValue;
  Dr_flagLst[gyro] = Dr_flag[gyro];
	
	return newValue;
}



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


