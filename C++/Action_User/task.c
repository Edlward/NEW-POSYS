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
//////////////////信号量定义区///////////////////////
 OS_EVENT 		*Update;
 
void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err; /* prevent warning... */
	
	/******************创建信号量***********************/
  Update				=	OSSemCreate(0);
	
	
  /*****************创建任务**************************/	
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,					//初始化任务
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
	/* 定时器初始化--------------------------------------*/
	/*
	时钟的周期设置.溢出时间Tout=(4999+1)*(83+1)/Tclk(时钟频率)(s)
	因为SystemInit()初始化APB1为4分频,为42MHz,根据是时钟树,当
	时钟分频为1时,TIM2~7,12~14时钟频率为APB1,否则为其两倍.
	由于默认外设线的分频,APB1为4,APB2为2,并且不能修改,因此都是其所在总线两倍.
	因此,定时器溢出时间5ms
	在TIM7中断函数里发送信号量
	*/
	TIM_Init(TIM7,4999,83,0,0); 
	/* 陀螺仪加热电阻PWM初始化--------------------------*/
//	pwm_init(999, 83);//此时PWM的频率为84MHz/(83+1)/(999+1)=1KHz
	/*
	设置发生PWM的定时器TIM3的占空比
	其引脚PB0连接加热电阻
	*/
	//ICM_HeatingPower(100);
	/* SPI初始化---------------------------------------*/
	//磁编码器的SPI初始化
	SPI1_Init();
	//陀螺仪的SPI初始化
	SPI2_Init();
	//片选的初始化
	CS_Config();
	/* 初始化FLASH为温度表修正做准备-------------------*/
//	Flash_Init();
	/* ICM20608G模块初始化-----------------------------------*/
//	ICM20608G_init();
	/* 串口初始化--------------------------------------*/
	USART1_Init(115200);
	
//	/* 初始化拓展卡尔曼滤波需要的矩阵 */
//	//AHRS_Init();
//	///* 最下二乘法拟合零点漂移 */
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
//			#ifdef HD_TEST //硬件测试，判断焊接是否正常
//			 test[0]=ICM_Read(ICM20608G_WHO_AM_I); //测试ICM20608G，正确值为0XAF
//			#endif
//			/* 计算角度 */
//     flag_initilize=updateAngle(); 
//      /* 控制陀螺仪温度  */			
  	  temperature_control(0.f);
			/* 计算坐标 */
			calculatePos();
			#ifndef DEBUG_ENABLE
			/* 数据发送 */
	//		  if(flag_initilize==1)
					DataSend();
			#endif
		}
		else
		{
			/*更新温度零飘表*/
      UpdateVDoffTable();
		}
	}
}


