
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
			TIM_SetCompare2(TIM5,(uint32_t)((double)value/100.0*1000));
		break;
		case 1:
			TIM_SetCompare4(TIM2,(uint32_t)((double)value/100.0*1000));
		break;
		case 2:
			TIM_SetCompare2(TIM3,(uint32_t)((double)value/100.0*1000));
		break;
	}
}
#define MAX_TEMPERATURE			0.48F
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
			allPara.sDta.GYRO_TemperatureAim[gyro]=allPara.GYRO_Temperature[gyro]+0.05f;
			if(allPara.sDta.GYRO_TemperatureAim[gyro]>MAX_TEMPERATURE)
			{
				allPara.sDta.GYRO_TemperatureAim[gyro]=MAX_TEMPERATURE;
			}
//			USART_OUT_F(allPara.sDta.GYRO_TemperatureAim[gyro]);
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
//��ʱ��Ļ����԰�����ĳ��滷���仯�ģ�����һ���ȶ����������������ǲ��Ե�ʱ����ܾͲ�����ô��
void temp_pid_ctr(int gyro,float val_ex)
{
  static float err[GYRO_NUMBER];
  static float integral[GYRO_NUMBER];
  static float differential[GYRO_NUMBER];
  static float err_last[GYRO_NUMBER];
//	static int justforfirst[3]={1,1,1};
	static int count[3]={0};
  float K_p[GYRO_NUMBER] = { 10600.0f , 10600.0f ,10600.0f };
  float K_i[GYRO_NUMBER] = { 3.5f , 3.5f ,3.5f };
  float K_d[GYRO_NUMBER] = { 0.0f , 0.0f ,0.0f };
  
  static double ctr[GYRO_NUMBER];
	
	/*���*/
	err[gyro]=val_ex-allPara.GYRO_Temperature[gyro];
	/*����*/
	integral[gyro]=integral[gyro]+err[gyro];
	/*΢����*/
	differential[gyro]=err[gyro]-err_last[gyro];
	/*��һ�ε����*/
	err_last[gyro]=err[gyro];

	ctr[gyro]=K_p[gyro]*err[gyro]+K_i[gyro]*integral[gyro]+K_d[gyro]*differential[gyro];
			
	/*��������*/
	if(ctr[gyro]>100)
	{
		ctr[gyro]=100;
	}else if(ctr[gyro]<0)
	{
		ctr[gyro]=0;
	}
	else{
//		if(justforfirst[gyro]==1){
//			//��ʱ��Ļ����԰�����ĳ��滷���仯�ģ�����һ���ȶ�������������
//			integral[gyro]=8.874924f/K_i[gyro];
//			justforfirst[gyro]=0;
//		}
	}
	
	if(allPara.GYRO_Temperature[gyro]>(MAX_TEMPERATURE+0.02f))
		ctr[gyro]=0;
		
		/*��ֹ�¶�һֱ����������������һֱΪ0�����ջ�оƬ*/
	if(fabs(allPara.GYRO_Temperature[gyro]-allPara.sDta.GYRO_TemperatureAim[gyro])>0.04f)
		count[gyro]++;
	else
		count[gyro]=0;
	if(count[gyro]>4*200)
		ctr[gyro]=20;
	
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
    //USART_OUT(USART3,"finish rise\r\n");
  }else if(circle_count==0){
    //USART_OUT(USART3,"finish decrease\r\n");
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



/*��ʱ�󣬲����ˣ�*/
#define Threshold_1 		 0.02f				//��ֵ1����һ�״����˲������仯�Ƕȴ��ڴ�ֵʱ����������
#define Threshold_2			 10				//��ֵ2����һ�״����˲���������ֵ���ڴ�ֵʱ�������������ǿ�˲�����
double LowPassFilter(float newValue,int gyro)
{
  static double K_x[GYRO_NUMBER]={0.0}; //�˲�ϵ��
  static double k[GYRO_NUMBER]={0.0};
  static uint32_t num_x[GYRO_NUMBER]={0};//�˲�������
  static uint8_t new_flag_x[GYRO_NUMBER]={0};//�������ݱ仯����
  static uint8_t old_flag_x[GYRO_NUMBER]={0};
  static double valueLast[GYRO_NUMBER]={0.0};
  
	if(valueLast[gyro]==0.0)
		valueLast[gyro]=newValue;
	
  //�Ƕȱ仯����new_flag=1��ʾ�Ƕ����ӣ�=0��ʾ�Ƕ����ڼ�С
  if((newValue-valueLast[gyro])>0.0)
  {
    new_flag_x[gyro]=1;
  }
  if((newValue-valueLast[gyro])<0.0)
  {
    new_flag_x[gyro]=0;
  }
  
  if(new_flag_x[gyro]==old_flag_x[gyro])  //�˴α仯��ǰһ�α仯�����Ƿ�һ�£���ȱ�ʾ�Ƕȱ仯����һ��
  {
    num_x[gyro]=num_x[gyro]+1;
    if(fabs(newValue-valueLast[gyro])>Threshold_1)
    {
      //���仯�Ƕȴ���Threshold_1�ȵ�ʱ�򣬽��м�����num�������ӣ��Դﵽ��������Kֵ����߸�����
      num_x[gyro]=num_x[gyro]+5;   
    }
  
		if(num_x[gyro]>Threshold_2)   //������ֵ���ã����Ƕȵ�����ݼ��ٶȴﵽһ������ʱ������Kֵ
		{
			K_x[gyro]=k[gyro]+0.07;  //0.2ΪK_x[gyro]������ֵ����ʵ����Ҫ�޸�
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
			K_x[gyro]=0.01; //�Ƕȱ仯�ȶ�ʱK_x[gyro]ֵ����ʵ���޸�
  }
  valueLast[gyro]=(1-K_x[gyro])*valueLast[gyro]+K_x[gyro]*newValue;
  old_flag_x[gyro]=new_flag_x[gyro];
  
  return valueLast[gyro];
  
}

double LowPassFilterGyro(float newValue)
{
  double K_x=0.03; //�˲�ϵ��
  static double valueLast=0.0;

	if(valueLast==0.0)
		valueLast=newValue;
	
  valueLast=(1-K_x)*valueLast+K_x*newValue;
  
  return valueLast;
  
}
void pwm1_init(uint32_t arr,uint32_t psc)
{		 					 
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  				//TIM5ʱ��ʹ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); 		//
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
  GPIO_Init(GPIOA,&GPIO_InitStructure);              
  
  
  TIM_TimeBaseStructure.TIM_Prescaler = psc;  //��ʱ����Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_Period = arr;   //�Զ���װ��ֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  
  TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//��ʼ����ʱ��2
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1       
  TIM_OCInitStructure.TIM_Pulse=1000*0.05;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
  TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC3
  
  
  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR3�ϵ�Ԥװ�ؼĴ���
  
  
  TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPEʹ��
  
  TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM5	
  
  TIM_SetCompare2(TIM5,0.0*1000);		
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


