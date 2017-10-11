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
#include "figurePos.h"
void init(void)
{
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
  /* 陀螺仪加热电阻PWM初始化--------------------------*/
  pwm_init(999, 83);//此时PWM的频率为84MHz/(83+1)/(999+1)=1KHz
  
  /* SPI初始化---------------------------------------*/
  //单轮模式时磁编码器的SPI初始化
  SPI1_Init();
  //单轮模式时陀螺仪的SPI初始化
  SPI2_Init();
  //片选的初始化
  CS_Config();
#ifdef TEST_SUMMER
  USART1_Init(921600);
#else
  USART1_Init(115200);
#endif
  /* ICM20608G模块初始化-----------------------------------*/
  ICM20608G_init();
  Flash_Init();
  TIM_Init(TIM2,999,83,0,0);					//主周期定时5ms
  ICM_HeatingPower(0);
  Delay_ms(100);//过滤开始时的错误数据
  driftCoffecientInit();
}
static int count=0;
int main(void)
{
  init();
  static uint32_t cpuUsage;
  while(1)
  {
    while(getTimeFlag())
    {
			count++;
      //				uint8_t test[3];
      //				test[0]=SPI_Read(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_WHO_AM_I); //测试ICM20608G，正确值为0XAF
      if(!(GetCommand()&CORRECT)){
        //        /* 计算角度 */
        if(!RoughHandle())
          TemporaryHandle();
        else {
         // if(GetCommand()&ACCUMULATE)
					{
            updateAngle();
            calculatePos();	
						#ifndef TEST_SUMMER
						DataSend();
						#endif
          }
        }
      }
      else
        UpdateVDoffTable();
      //真实的占空比会等于cpuUsage或大于其一个单位
      cpuUsage=getTimeCount();
    }
  }
}

