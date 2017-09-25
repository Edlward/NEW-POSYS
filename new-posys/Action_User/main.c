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
	TIM_Init(TIM2,99,83,0,0);					//�����ڶ�ʱ5ms	
	/* ��ʱ����ʼ��--------------------------------------*/
	/*
	ʱ�ӵ���������.���ʱ��Tout=(4999+1)*(83+1)/Tclk(ʱ��Ƶ��)(s)
	��ΪSystemInit()��ʼ��APB1Ϊ4��Ƶ,Ϊ42MHz,������ʱ����,��
	ʱ�ӷ�ƵΪ1ʱ,TIM2~7,12~14ʱ��Ƶ��ΪAPB1,����Ϊ������.
	����Ĭ�������ߵķ�Ƶ,APB1Ϊ4,APB2Ϊ2,���Ҳ����޸�,��˶�����������������.
	���,��ʱ�����ʱ��5ms
	��TIM7�жϺ����﷢���ź���
	*/
		
	/* �����Ǽ��ȵ���PWM��ʼ��--------------------------*/
	pwm_init(999, 83);//��ʱPWM��Ƶ��Ϊ84MHz/(83+1)/(999+1)=1KHz
	ICM_HeatingPower(0);
	/*
	���÷���PWM�Ķ�ʱ��TIM3��ռ�ձ�
	������PB0���Ӽ��ȵ���
	*/
	/* SPI��ʼ��---------------------------------------*/
	//����ģʽʱ�ű�������SPI��ʼ��
	SPI1_Init();
	//����ģʽʱ�����ǵ�SPI��ʼ��
	SPI2_Init();
	//Ƭѡ�ĳ�ʼ��
	CS_Config();
	/* ��ʼ��FLASHΪ�¶ȱ�������׼��-------------------*/
	Flash_Init();
	/* ���ڳ�ʼ��--------------------------------------*/
//	USART6_Init(921600);
	USART1_Init(921600);
	/* ICM20608Gģ���ʼ��-----------------------------------*/
	ICM20608G_init();
	/* ���¶��˷�������Ư�� */
	
}

int main(void)
{
	init();
	static uint32_t cpuUsage;
	while(1)
	{
	while(getTimeFlag())
	{
		if(!(GetCommand()&0x01))
		{	
			#ifdef HD_TEST //Ӳ�����ԣ��жϺ����Ƿ�����
			uint8_t test[3];
			test[0]=SPI_Read(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_WHO_AM_I); //����ICM20608G����ȷֵΪ0XAF
			test[1]=SPI_Read(SPI2,GPIOB,GPIO_Pin_10,ICM20608G_WHO_AM_I); //����ICM20608G����ȷֵΪ0XAF
			test[2]=SPI_Read(SPI2,GPIOB,GPIO_Pin_12,I3G_WHO_AM_I); //����ICM20608G����ȷֵΪ0XAF
			#endif
			/* ����Ƕ� */
		//  RoughHandle();
   //   TemporaryHandle(GetCommand());
		// if(GetCommand()){
	//			updateAngle();
//		 }
      /* �����������¶�  */			
		  //temperature_control(42);
			#ifndef DEBUG_ENABLE
			/* ���ݷ��� */
//		 if(GetCommand())
//					DataSend();
			#endif
		}
		else
		{
			/*�����¶���Ʈ��*/
      UpdateVDoffTable();
		}
		//��ʵ��ռ�ձȻ����cpuUsage�������һ����λ
		cpuUsage=getTimeCount();
	}
	}
}

