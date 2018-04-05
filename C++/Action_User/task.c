#include  <includes.h>
#include  <app_cfg.h>
#include  <includes.h>
#include  <app_cfg.h>
#include "timer.h"
#include "usart.h"
#include "can.h"
#include "misc.h"
#include "spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "usart.h"
#include "motion_attitude_algorithm.h"
#include "math.h"
#include "temperature_control.h"
#include "flash.h"
#include "vdoff.h"
#include "pos.h"
#include "customer.h"
#include "action_AHRS.h"
//////////////////�ź���������///////////////////////
 OS_EVENT 		*Update;
 
void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err; /* prevent warning... */
	
	/******************�����ź���***********************/
  Update				=	OSSemCreate(0);
	
	
  /*****************��������**************************/	
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,					//��ʼ������
	                      	(void          * ) 0,							
							(OS_STK        * )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],		
							(INT8U           ) Config_TASK_START_PRIO  );	

													
	os_err = OSTaskCreate(	(void (*)(void *))UpdateTask,		
	                      	(void          * ) 0,							
							(OS_STK        * )&Task1Stk[TASK1_START_STK_SIZE - 1],		
							(INT8U           ) UPDATE_START_PRIO  );

}

void ConfigTask(void)
{
	CPU_INT08U  os_err;
	os_err=os_err;
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	Delay_ms(10);
	/* ��ʱ����ʼ��--------------------------------------*/
	/*
	ʱ�ӵ���������.���ʱ��Tout=(4999+1)*(83+1)/Tclk(ʱ��Ƶ��)(s)
	��ΪSystemInit()��ʼ��APB1Ϊ4��Ƶ,Ϊ42MHz,������ʱ����,��
	ʱ�ӷ�ƵΪ1ʱ,TIM2~7,12~14ʱ��Ƶ��ΪAPB1,����Ϊ������.
	����Ĭ�������ߵķ�Ƶ,APB1Ϊ4,APB2Ϊ2,���Ҳ����޸�,��˶�����������������.
	���,��ʱ�����ʱ��5ms
	��TIM7�жϺ����﷢���ź���
	*/
	TIM_Init(TIM7,4999,83,0,0); 
	/* �����Ǽ��ȵ���PWM��ʼ��--------------------------*/
//	pwm_init(999, 83);//��ʱPWM��Ƶ��Ϊ84MHz/(83+1)/(999+1)=1KHz
	/*
	���÷���PWM�Ķ�ʱ��TIM3��ռ�ձ�
	������PB0���Ӽ��ȵ���
	*/
	//ICM_HeatingPower(100);
	/* SPI��ʼ��---------------------------------------*/
	//�ű�������SPI��ʼ��
	SPI1_Init();
	//�����ǵ�SPI��ʼ��
	SPI2_Init();
	//Ƭѡ�ĳ�ʼ��
	CS_Config();
	/* ��ʼ��FLASHΪ�¶ȱ�������׼��-------------------*/
//	Flash_Init();
	/* ICM20608Gģ���ʼ��-----------------------------------*/
//	ICM20608G_init();
	/* ���ڳ�ʼ��--------------------------------------*/
	USART1_Init(115200);
	
//	/* ��ʼ����չ�������˲���Ҫ�ľ��� */
//	//AHRS_Init();
//	///* ���¶��˷�������Ư�� */
//	WaitForUpdataVDoff();
	OSTaskSuspend(OS_PRIO_SELF);
}
void UpdateTask(void)
{
	CPU_INT08U     os_err;
	uint8_t flag_initilize;
	#ifdef HD_TEST
  static uint8_t test[2];
	#endif
	os_err=os_err;
	OSSemSet(Update,0,&os_err);	
	while(1)
	{
		OSSemPend(Update,0,&os_err);
		if(1)
		{	
//			#ifdef HD_TEST //Ӳ�����ԣ��жϺ����Ƿ�����
//			 test[0]=ICM_Read(ICM20608G_WHO_AM_I); //����ICM20608G����ȷֵΪ0XAF
//			#endif
//			/* ����Ƕ� */
//     flag_initilize=updateAngle(); 
//      /* �����������¶�  */			
  	  temperature_control(0.f);
			/* �������� */
			calculatePos();
			#ifndef DEBUG_ENABLE
			/* ���ݷ��� */
	//		  if(flag_initilize==1)
					DataSend();
			#endif
		}
		else
		{
			/*�����¶���Ʈ��*/
      UpdateVDoffTable();
		}
	}
}


