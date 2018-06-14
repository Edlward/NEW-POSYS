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
#include "gpio.h"
#include "odom.h"
#include "self_math.h"
AllPara_t allPara={0};

void init(void)
{
	Led_Init();
	
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
		
	//StartCount();
	SoftWareReset();
	
//	LED1_OFF;
//	LED2_OFF;
  pwm_init(999, 83);//Ϊ84MHz/(83+1)/(999+1)=1KHz
  
	ICM_SPIInit();
	SPI2_Init();
	SPI3_Init();
  CS_Config();
	
	#ifdef TEST_SUMMER
	USART1DMAInit(921600);
	#else
	USART1DMAInit(115200);
	#endif
	USART6DMAInit(115200);
//  Flash_Init();
	
	for(int gyro;gyro<GYRO_NUMBER;gyro++)
	{

		if(!allPara.resetFlag)
			MEMS_Configure(gyro);
		ICM_HeatingPower(gyro,0);
	}
	
  TIM_Init(TIM2,999,83,2,0);			
	
	allPara.sDta.flag=0;
	
	#ifdef TEST_SUMMER
	SetFlag(START_COMPETE);
	#endif
	
	#ifndef AUTOCAR
	SetFlag(START_COMPETE);
	#endif
	
	if(!allPara.resetFlag)
	{
		SetFlag(HEATING);
	}
	else
	{
		SetFlag(START_COMPETE);
	}
	
  //driftCoffecientInit();
	
	IWDG_Init(4,100); // 200ms//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).
	
	StartCount();
	while(!getTempInitSuces())
	{
		if(allPara.resetFlag)
			SetTempInitSuces();
		if(getCount()>2000)
		{
			DeadWhileReport(31);
		}
	}
	EndCnt();
}

int main(void)
{
  init();
//	char readOrderLast=(char)-1;
  while(1)
  {
    while(getTimeFlag())
    {
			allPara.sDta.time++;
			if(CheckNan()&&allPara.sDta.GYRO_Bais[2]==0.0)
			{
				IWDG_Reset();
			}
			
			if(allPara.sDta.time>200*5)
				allPara.sDta.time--;
			
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
				
				JudgeStatic();
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
      allPara.cpuUsage--;
    }
  }
}

