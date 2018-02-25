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
	
  /* 陀螺仪加热电阻PWM初始化--------------------------*/
  pwm_init(999, 83);//此时PWM的频率为84MHz/(83+1)/(999+1)=1KHz
  
	/* SPI初始化---------------------------------------*/
	//单轮模式时磁编码器的SPI初始化
	ICM_SPIInit();
	//单轮模式时陀螺仪的SPI初始化
	SPI2_Init();
	
  //片选的初始化
  CS_Config();
	
	USART1_Init(115200);

  Flash_Init();
	
	for(int gyro;gyro<GYRO_NUMBER;gyro++)
	{
		/*ICM20608G模块初始化*/
		ICM20608G_init(gyro);
		/*初始时不加热*/
		ICM_HeatingPower(gyro,0);
	}
	
  TIM_Init(TIM2,999,83,0,0);					//主周期定时5ms
	
	SetCommand(HEATING);
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
				
        //计算角度 
        if(RoughHandle())
				{
					DataSend();
				}
    }
  }
}

