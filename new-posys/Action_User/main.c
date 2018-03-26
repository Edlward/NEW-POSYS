/**
******************************************************************************
* @file    main.c
* @author  Summer
* @version V1.0.1
* @date    13-August-2017
* @brief   the start
******************************************************************************
* @attention
* Summer is the most handsome man !
*
**/
#include "misc.h"
#include "config.h"
#include "DataRecover.h"
#include "iwdg.h"
AllPara_t allPara={0};

void init(void)
{
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	
  TIM_Init(TIM7,99,83,0,0);					//�����ڶ�ʱ5ms
	//StartCount();
	SoftWareReset();
	
  /* �����Ǽ��ȵ���PWM��ʼ��--------------------------*/
  pwm_init(999, 83);//��ʱPWM��Ƶ��Ϊ84MHz/(83+1)/(999+1)=1KHz
  
	/* SPI��ʼ��---------------------------------------*/
	//����ģʽʱ�ű�������SPI��ʼ��
	ICM_SPIInit();
	//����ģʽʱ�����ǵ�SPI��ʼ��
	SPI2_Init();
	
  //Ƭѡ�ĳ�ʼ��
  CS_Config();
	
	#ifdef TEST_SUMMER
	USART1_Init(921600);
	#else
	USART1_Init(115200);
	#endif
	
  Flash_Init();
	
	for(int gyro;gyro<GYRO_NUMBER;gyro++)
	{
		/*ICM20608Gģ���ʼ��*/
		if(!allPara.resetFlag)
			MEMS_Configure(gyro);
		/*��ʼʱ������*/
		ICM_HeatingPower(gyro,0);
	}
	
  TIM_Init(TIM2,999,83,1,0);					//�����ڶ�ʱ5ms
	
	allPara.sDta.flag=0;
	
	#ifndef TEST_SUMMER
	SetFlag(START_COMPETE);
	#endif
	
	//ֻ�е�һ������ʱ�ų�ʼ��״̬��
	if(!allPara.resetFlag)
	{
		SetFlag(HEATING);
	}
	
  //driftCoffecientInit();
	
	IWDG_Init(1,5); // 1.5ms
	
	while(!getTempInitSuces())
	{
		//����������ģ�ֱ������
		if(allPara.resetFlag)
			SetTempInitSuces();
	}
}


int main(void)
{
  init();
//	char readOrderLast=(char)-1;
  while(1)
  {
    while(getTimeFlag())
    {
			//����ʱ��������Ϊ����ൢ��5ms
//			while(readOrderLast==getReadOrder()){;}
//				readOrderLast=getReadOrder();

//      				uint8_t test[3];
//      				test[0]=SPI_Read(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_WHO_AM_I); //����ICM20608G����ȷֵΪ0XAF
			//ʹ�����ܹ�ͬ�������ǲ�ͬ���������
			//ATָ���
			if(CheckNan())
				IWDG_Reset();
			AT_CMD_Handle();
      if(!(allPara.sDta.flag&CORRECT)){
				if(allPara.sDta.flag&HEATING)
				{
					for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
						temp_pid_ctr(gyro,allPara.sDta.GYRO_TemperatureAim[gyro]);
				}
				else
				{
					for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
						temp_pid_ctr(gyro,allPara.sDta.GYRO_TemperatureAim[gyro]-0.5f);
				}
				/*�ж��Ƿ�ֹ*/
				JudgeStatic();
        //����Ƕ� 
        if(RoughHandle())
				{
					if((allPara.sDta.flag&START_COMPETE))
					{
						updateAngle();
						calculatePos();
					}
					DataSend();
				}
			}
      else{
        UpdateVDoffTable();
			}
			allPara.resetFlag=0;
      //��ʵ��ռ�ձȻ����cpuUsage�������һ����λ
      allPara.cpuUsage--;
    }
  }
}

