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
AllPara_t allPara={0};

void init(void)
{
	#ifdef NEW_BOARD
	Led_Init();
	LedAbNormal();
	#endif
	
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	
  TIM_Init(TIM7,99,83,0,0);				

	SoftWareReset();

	#ifdef NEW_BOARD
	LedNormal();
	#endif
	
	ICM_SPIInit();
	SPI2_Init();

  CS_Config();
	
	#ifdef TEST_SUMMER
	usart_Init(921600);
	#else
	usart_Init(115200);
	#endif
	
  TIM_Init(TIM2,999,83,2,0);			
	
	allPara.sDta.flag=0;

	SetFlag(START_COMPETE);
	
	IWDG_Init(1,50); // 1.5ms
	
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

			LedNormal();
			if(allPara.sDta.time>200*5)
				allPara.sDta.time--;
			
			AT_CMD_Handle();
			
			JudgeStatic();

			calculatePos();
			
			DataSend();
			
			allPara.resetFlag=0;
      allPara.cpuUsage--;
    }
  }
}

