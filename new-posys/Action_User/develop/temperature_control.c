
#include "config.h"

/*
�ֱ������豸����,�����¶�,��ʵ�¶�
*/

extern AllPara_t allPara;
/*
�ֱ������豸����,�����¶�,��ʵ�¶�
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
	
		/*���*/
		err[gyro]=val_ex-allPara.GYRO_Temperature[gyro];
		/*����*/
		err_sum[gyro]=err_sum[gyro]+err[gyro];
		/*΢����*/
		err_v[gyro]=err[gyro]-err_last[gyro];
		err_last[gyro]=err[gyro];

			ctr[gyro]=Kp_summer[gyro]*err[gyro]+Ki_summer[gyro]*err_sum[gyro]+Kd_summer[gyro]*err_v[gyro];
			/*��������*/
			if(ctr[gyro]>50)
			{
				ctr[gyro]=100;
			}
			if(ctr[gyro]<0)
			{
				ctr[gyro]=0;
			}
		
		/*#define ICM_HeatingPower(a)  TIM_SetCompare3(TIM3,a/100.0*1000); */
		/*֮�������ֵΪ1000,����Ϊ�ö�ʱ����װ��ֵΪ1000*/
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
/*��λ����*/
#define TIME_BEAR	15
/*ÿ��TIME_BEAR����һ�Σ�������ʱʱ�̶̿���ǰһ�����¶��жϣ������Ļ���ӵúܿ죩
���һ����֮���¶�С��0.1�㣬�Ǿ͸ı�PWM
��ô����Ϊ�˲����£����»���ɲ��ȶ���ʱ��Ч�������󣬿������˲��ͺ�Ҳ������*/
int TempErgodic(int gyroNum,int reset){
  static uint32_t success[GYRO_NUMBER];
  static int direction[GYRO_NUMBER]={1,1,1};
  static float PWM[GYRO_NUMBER]={0.f,0.f,0.f};
  static float temp_last[GYRO_NUMBER];
  static uint32_t time[GYRO_NUMBER] = { 0 };
	if(reset) success[gyroNum]=0;
  /*200/s,12000/min*/
  time[gyroNum]++;
  /*ÿ��TIME_BEAR���ж�һ��*/
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
		/*�ų�����ʼ�¶�С����Сֵ�����*/
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
  if (!(Dr_flag[gyro]^Dr_flagLst[gyro]))    //ǰ�����ݱ仯����һ��
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
  //һ���˲��㷨
  if (Dr_flag[gyro] == 0)     //��ǰֵС��ǰһ��ֵ
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
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  				//TIM2ʱ��ʹ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2); 		//
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
  GPIO_Init(GPIOB,&GPIO_InitStructure);              
  
  
  TIM_TimeBaseStructure.TIM_Prescaler = psc;  //��ʱ����Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_Period = arr;   //�Զ���װ��ֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1       
  TIM_OCInitStructure.TIM_Pulse=1000*0.05;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC3
  
  
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
  
  
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPEʹ��
  
  TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM2	
  
  TIM_SetCompare3(TIM2,0.0*1000);		
}  
void pwm2_init(uint32_t arr,uint32_t psc)
{		 					 
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  				//TIM2ʱ��ʹ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2); 		//
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
  GPIO_Init(GPIOB,&GPIO_InitStructure);              
  
  
  TIM_TimeBaseStructure.TIM_Prescaler = psc;  //��ʱ����Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_Period = arr;   //�Զ���װ��ֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1       
  TIM_OCInitStructure.TIM_Pulse=1000*0.05;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC3
  
  
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
  
  
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPEʹ��
  
  TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM2	
  
  TIM_SetCompare4(TIM2,0.0*1000);		
}  
void pwm3_init(uint32_t arr,uint32_t psc)
{		 					 
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  				//TIM3ʱ��ʹ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 		//
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
  GPIO_Init(GPIOC,&GPIO_InitStructure);              
  
  
  TIM_TimeBaseStructure.TIM_Prescaler = psc;  //��ʱ����Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_Period = arr;   //�Զ���װ��ֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1       
  TIM_OCInitStructure.TIM_Pulse=1000*0.05;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC3
  
  
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
  
  
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ��
  
  TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3	
  
  TIM_SetCompare2(TIM3,0.0*1000);		
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
  pwm1_init(arr, psc);
	pwm2_init(arr, psc);
	pwm3_init(arr, psc);
}  


