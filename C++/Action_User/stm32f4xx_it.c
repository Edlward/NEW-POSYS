/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include  <ucos_ii.h>
#include "stm32f4xx_tim.h"
#include "timer.h"
#include "motion_attitude_algorithm.h"
#include "flash.h"
#include "stm32f4xx_usart.h"
#include "usart.h"
#include "stm32f4xx_can.h"
#include "can.h"
#include "vdoff.h"
#include "customer.h"
#include "pos.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
extern OS_EVENT *Update;
void TIM7_IRQHandler()
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	
	if(TIM_GetITStatus(TIM7, TIM_IT_Update)==SET)
  {	
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		OSSemPost(Update);   //发送信号量
		/*	time_sig=1;*/
		set_time_sig();
  }	 
	 OSIntExit();
}


static union
{
	float   val;
	uint8_t data[4];
}valGet;
void USART1_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	uint8_t data;
	static uint8_t status=0;
	static char flag=0x00;
	static float R_set=0;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(USART_GetFlagStatus(USART1,USART_IT_RXNE)==SET)
	{
		USART_ClearITPendingBit( USART1,USART_IT_RXNE);
		data=USART_ReceiveData(USART1);
		//USART_OUT(USART1,"\r\n data = %d\r\n",data);
		
		switch(status)
		{
			case 0:
				if(data=='A')
				  status++;
				break;
			case 1:
				if(data=='C')
				  status++;
				else
					status=0;
				break;
			case 2:
				if(data=='T')
				  status++;
				else
					status=0;
				break;
			case 3:
        flag=data;			
			  switch(data)
				{
					case 'R':
						status=0;	
					/*如果flag为1,则置零.结束矫正*/
						if(GetFlashUpdataFlag())
							SetFlashUpdateFlag(0);
						/*如果不为0,则要开始矫正了*/
						else
						{
							Flash_Zero(160*(TempTable_max-TempTable_min));
							Flash_Init();
							SetFlashUpdateFlag(1);
						}
						break;
					case 'S':
						R_set=0;
					  status++;
						break;
					case '0':
						resetAngle();
					  resetPos();
					  status=0;
						break;
					case 'X':
					case 'Y':
					case 'A':
						status++;
						break;
					default:
						break;
				}
			case 4:
			case 5:
			case 6:
			case 7:
				switch(flag)
				{
						case 'S':
							if(data>='0'&&data<='9')
							{
								status++;
								R_set=R_set*10+(data-'0');
								if(status>=8)
								{
									Set_R_Zaxis(R_set/1000.0f);
									status=0;
								}
							}
							else
							{
								R_set=0;
								status=0;
							}
						  break;
						case 'X':
						case 'Y':
						case 'A':
							valGet.data[status-4]=data;
						  status++;
						  if(status>=8)
							{
								status=0;
								if(flag=='X')
								{
									setPosX(valGet.val);
								}
								if(flag=='Y')
								{
									setPosY(valGet.val);
								}
								if(flag=='A')
								{
									setAngle(valGet.val);
								}
								flag=0x00;
							}
							break;
						default:
							flag=0x00;
							status=0;
							break;
				}
				break;
			default:
				status=0;
				break;
		}
	}
	OSIntExit();
}

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
   while (1)
   {
   }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
 
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
