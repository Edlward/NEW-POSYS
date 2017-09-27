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
#include "usart.h"
#include "misc.h"
#include "timer.h"
#include "gpio.h"
#include "misc.h"
#include "stm32f4xx_it.h"
#include "temperature_control.h"
#include "spi.h"
#include "flash.h"
#include "icm_20608_g.h"
#include "figureAngle.h"
#include "buildExcel.h"
#include "customer.h"
#include "config.h"
void init(void)
{
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	TIM_Init(TIM2,99,83,0,0);					//主周期定时5ms	
	/* 陀螺仪加热电阻PWM初始化--------------------------*/
	pwm_init(999, 83);//此时PWM的频率为84MHz/(83+1)/(999+1)=1KHz
	ICM_HeatingPower(0);

	/* SPI初始化---------------------------------------*/
	//单轮模式时磁编码器的SPI初始化
	SPI1_Init();
	//单轮模式时陀螺仪的SPI初始化
	SPI2_Init();
	//片选的初始化
	CS_Config();
	USART1_Init(921600);
	/* 初始化FLASH为温度表修正做准备-------------------*/
	Flash_Init();
	/* 串口初始化--------------------------------------*/
//	USART6_Init(921600);
	/* ICM20608G模块初始化-----------------------------------*/
	ICM20608G_init();
	
	while(!Heating(35));
}

int main(void)
{
	init();
	static uint32_t cpuUsage;
	while(1)
	{
	while(getTimeFlag())
	{
		if(!(GetCommand()&CORRECT))
		{	
			#ifdef HD_TEST //硬件测试，判断焊接是否正常
			uint8_t test[3];
			test[0]=SPI_Read(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_WHO_AM_I); //测试ICM20608G，正确值为0XAF
			test[1]=SPI_Read(SPI2,GPIOB,GPIO_Pin_10,ICM20608G_WHO_AM_I); //测试ICM20608G，正确值为0XAF
			test[2]=SPI_Read(SPI2,GPIOB,GPIO_Pin_12,I3G_WHO_AM_I); //测试ICM20608G，正确值为0XAF
			#endif
			/* 计算角度 */
		  RoughHandle();
      TemporaryHandle();
			if(GetCommand()&ACCUMULATE){
				updateAngle();
			}	
      /* 控制陀螺仪温度  */			
		  temperature_control(42);
			#ifndef DEBUG_ENABLE
			/* 数据发送 */
		 if(GetCommand()&ACCUMULATE)
			   	DataSend();
			#endif
		}
		else
		{
			/*更新温度零飘表*/
      UpdateVDoffTable();
		}
		//真实的占空比会等于cpuUsage或大于其一个单位
		cpuUsage=getTimeCount();
	}
	}
}

