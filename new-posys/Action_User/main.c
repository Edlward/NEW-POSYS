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
#include <stdlib.h>
void init(void)
{
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
  /* �����Ǽ��ȵ���PWM��ʼ��--------------------------*/
  pwm_init(999, 83);//��ʱPWM��Ƶ��Ϊ84MHz/(83+1)/(999+1)=1KHz
  
  /* SPI��ʼ��---------------------------------------*/
  //����ģʽʱ�ű�������SPI��ʼ��
  SPI1_Init();
  //����ģʽʱ�����ǵ�SPI��ʼ��
  SPI2_Init();
  //Ƭѡ�ĳ�ʼ��
  CS_Config();
#ifdef TEST_SUMMER
  USART1_Init(921600);
#else
  USART1_Init(115200);
#endif
  /* ICM20608Gģ���ʼ��-----------------------------------*/
  Flash_Init();
  ICM20608G_init();
	
  ICM_HeatingPower(0);
  Delay_ms(100);//���˿�ʼʱ�Ĵ�������
  driftCoffecientInit();
  TIM_Init(TIM2,999,83,0,0);					//�����ڶ�ʱ5ms
}
int main(void)
{
  init();
  static uint32_t cpuUsage;
	char readOrderLast=-1;
  while(1)
  {
    while(getTimeFlag())
    {
      //				uint8_t test[3];
      //				test[0]=SPI_Read(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_WHO_AM_I); //����ICM20608G����ȷֵΪ0XAF
			//ʹ�����ܹ�ͬ�������ǲ�ͬ���������
			while(readOrderLast==getReadOrder());
			readOrderLast=getReadOrder();
      if(!(GetCommand()&CORRECT)){
        //        /* ����Ƕ� */
        if(!RoughHandle())
          TemporaryHandle();
        else {
         // if(GetCommand()&ACCUMULATE)
					{
            updateAngle();
            calculatePos();	
						#ifndef TEST_SUMMER
						//���ڱ��жϴ����Ȼ���������ͣ������˼����ӣ�
						DataSend();
						#endif
          }
        }
      }
      else
        UpdateVDoffTable();
      //��ʵ��ռ�ձȻ����cpuUsage�������һ����λ
      cpuUsage=getTimeCount();
    }
  }
}

