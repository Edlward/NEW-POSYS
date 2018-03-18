
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
			if(allPara.GYRO_TemperatureAim[gyro]>0.42f)
			{
				allPara.GYRO_TemperatureAim[gyro]=0.42f;
			}
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

void SetTempInitSuces(void)
{
		tempInitSuces=1;
}
//有时间的话可以把这个改成随环境变化的（把上一次稳定参数存下来）但是测试的时候可能就不能那么玩
void temp_pid_ctr(int gyro,float val_ex)
{
  static float err[GYRO_NUMBER];
  static float err_sum[GYRO_NUMBER];
  static float err_last[GYRO_NUMBER];
  static float err_v[GYRO_NUMBER];
	static int justforfirst[3]={1,1,1};
	static int count[3]={0};
  float Kp_summer[GYRO_NUMBER] = { 2550.0f , 2550.0f ,2550.0f };
  float Ki_summer[GYRO_NUMBER] = { 0.75f , 0.75f ,0.75f };
  float Kd_summer[GYRO_NUMBER] = { 0.0f , 0.0f ,0.0f };
  
  static double ctr[GYRO_NUMBER];
	
		/*误差*/
		err[gyro]=val_ex-allPara.GYRO_Temperature[gyro];
	
		allPara.GYRO_TemperatureDif[gyro]=err[gyro]*100;
		/*积分*/
		err_sum[gyro]=err_sum[gyro]+err[gyro];
		/*微分量*/
		err_v[gyro]=err[gyro]-err_last[gyro];
		err_last[gyro]=err[gyro];

			ctr[gyro]=Kp_summer[gyro]*err[gyro]+Ki_summer[gyro]*err_sum[gyro]+Kd_summer[gyro]*err_v[gyro];
			/*调节上限*/
			if(ctr[gyro]>20)
			{
				ctr[gyro]=100;
			}else if(ctr[gyro]<0)
			{
				ctr[gyro]=0;
			}else{
				if(justforfirst[gyro]==1){
					//有时间的话可以把这个改成随环境变化的（把上一次稳定参数存下来）
					err_sum[gyro]=8.874924f/Ki_summer[gyro];
					justforfirst[gyro]=0;
				}
			}
		/*#define ICM_HeatingPower(a)  TIM_SetCompare3(TIM3,a/100.0*1000); */
		/*之所以最大值为1000,是因为该定时器的装载值为1000*/
		if(allPara.GYRO_Temperature[gyro]>0.45f)
			ctr[gyro]=0;
		if(fabs(allPara.GYRO_Temperature[gyro]-allPara.GYRO_TemperatureAim[gyro])>0.04f)
			count[gyro]++;
		else
			count[gyro]=0;
		if(count[gyro]>4*200)
			ctr[gyro]=20;
		
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



/*延时大，不用了！*/
#define Threshold_1 		 0.02f				//阈值1用于一阶带参滤波器，变化角度大于此值时，计数增加
#define Threshold_2			 10				//阈值2用于一阶带参滤波器，计数值大于此值时，增大参数，增强滤波跟随
double LowPassFilter(float newValue,int gyro)
{
  static double K_x[GYRO_NUMBER]={0.0}; //滤波系数
  static double k[GYRO_NUMBER]={0.0};
  static uint32_t num_x[GYRO_NUMBER]={0};//滤波计数器
  static uint8_t new_flag_x[GYRO_NUMBER]={0};//本次数据变化方向
  static uint8_t old_flag_x[GYRO_NUMBER]={0};
  static double valueLast[GYRO_NUMBER]={0.0};
  
	if(valueLast[gyro]==0.0)
		valueLast[gyro]=newValue;
	
  //角度变化方向，new_flag=1表示角度增加，=0表示角度正在减小
  if((newValue-valueLast[gyro])>0.0)
  {
    new_flag_x[gyro]=1;
  }
  if((newValue-valueLast[gyro])<0.0)
  {
    new_flag_x[gyro]=0;
  }
  
  if(new_flag_x[gyro]==old_flag_x[gyro])  //此次变化与前一次变化方向是否一致，相等表示角度变化方向一致
  {
    num_x[gyro]=num_x[gyro]+1;
    if(fabs(newValue-valueLast[gyro])>Threshold_1)
    {
      //当变化角度大于Threshold_1度的时候，进行计数器num快速增加，以达到快速增大K值，提高跟随性
      num_x[gyro]=num_x[gyro]+5;   
    }
  
		if(num_x[gyro]>Threshold_2)   //计数阈值设置，当角度递增或递减速度达到一定速率时，增大K值
		{
			K_x[gyro]=k[gyro]+0.07;  //0.2为K_x[gyro]的增长值，看实际需要修改
			if(K_x[gyro]>1.0) 
				K_x[gyro]=1.0; 
			num_x[gyro]=0;
		}
	}
  else 
  {
    num_x[gyro]=0;
		K_x[gyro]=K_x[gyro]-0.07;
		if(K_x[gyro]<0.01)
			K_x[gyro]=0.01; //角度变化稳定时K_x[gyro]值，看实际修改
  }
  valueLast[gyro]=(1-K_x[gyro])*valueLast[gyro]+K_x[gyro]*newValue;
  old_flag_x[gyro]=new_flag_x[gyro];
  
  return valueLast[gyro];
  
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


