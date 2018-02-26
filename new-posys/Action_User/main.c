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
AllPara_t allPara;

void init(void)
{
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	
  /* �����Ǽ��ȵ���PWM��ʼ��--------------------------*/
  pwm_init(999, 83);//��ʱPWM��Ƶ��Ϊ84MHz/(83+1)/(999+1)=1KHz
  
	/* SPI��ʼ��---------------------------------------*/
	//����ģʽʱ�ű�������SPI��ʼ��
	ICM_SPIInit();
	//����ģʽʱ�����ǵ�SPI��ʼ��
	SPI2_Init();
	
  //Ƭѡ�ĳ�ʼ��
  CS_Config();
	
	USART1_Init(921600);

  Flash_Init();
	
	for(int gyro;gyro<GYRO_NUMBER;gyro++)
	{
		/*ICM20608Gģ���ʼ��*/
		ICM20608G_init(gyro);
		/*��ʼʱ������*/
		ICM_HeatingPower(gyro,0);
	}
	
  TIM_Init(TIM2,999,83,0,0);					//�����ڶ�ʱ5ms
	
	SetCommand(HEATING);
	SetCommand(ACCUMULATE);
}


int main(void)
{
  init();
  static uint32_t cpuUsage;
	char readOrderLast=(char)-1;
  while(1)
  {
    while(getTimeFlag())
    {
				if(GetCommand()&HEATING)
				{
					for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
						temp_pid_ctr(gyro,allPara.GYRO_TemperatureAim[gyro]);
				}
				else
				{
					for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
						temp_pid_ctr(gyro,allPara.GYRO_TemperatureAim[gyro]-0.5f);
				}
				
        //����Ƕ� 
        if(RoughHandle())
				{
					calculatePos();
					//DataSend();
				}
				
			allPara.cpuUsage--;
    }
  }
}

