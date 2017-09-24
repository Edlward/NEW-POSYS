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
#include "flash.h"
#include "config.h"
/*************定时器2******start************/
//每1ms调用一次  用于读取编码器的值和计算坐标

//int posx,posy;
static uint32_t timeCount=0;
static uint8_t timeFlag=0;
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
  {	
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		timeCount++;
		if(timeCount>=PERIOD*10000)
		{
			timeCount=0;
			timeFlag=1;
		}
  }	 
}

uint8_t getTimeFlag(void)
{
	uint8_t nowFlag;
	nowFlag=timeFlag;

	if(nowFlag)
	{
		timeFlag=0;
		return 1;
	}
	return 0;
}

uint32_t getTimeCount(void)
{
	return (timeCount);
}

//定时器1  
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update)==SET)    
	{                                                
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

//定时器8  
void TIM8_UP_TIM13_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM8, TIM_IT_Update)==SET)    
	{                                                
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
}

/********************************************************/
/*****************普通定时TIM5*****Start*****************/
void TIM5_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM5, TIM_IT_Update)==SET)    
	{              
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET)    
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	
	}
}



//定时器4  
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)
	{                                  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

void UART5_IRQHandler(void)
{
	//uint8_t data = 0;
	if(USART_GetITStatus(UART5, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit( UART5,USART_IT_RXNE);
		//data=USART_ReceiveData(UART5);
		
	}
	 
}

/*
第一位 陀螺仪开始积分
第二位 陀螺仪开始矫正
第三位 陀螺
*/

#define STARTACCUMULATE 0X01
#define STARTCORRECT    0X02
static uint8_t command=0;
void SetCommand(int val){
	command=val;
}
uint8_t GetCommand(void){
	return command;
}
void USART6_IRQHandler(void)
{
	uint8_t data;
	static uint8_t status=0;
	if(USART_GetITStatus(USART6,USART_IT_RXNE)==SET)
	{
		USART_ClearITPendingBit( USART6,USART_IT_RXNE);
		data=USART_ReceiveData(USART6);
		
		switch(status)
		{
			case 0:
				if(data=='A')
				  status++;
				else if(data=='S')
					USART_OUT(USART1,"\r\nok");
				else
					status=0;
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
				else if(data=='C')
					status=4;
				else
					status=0;
				break;
			case 3:	
			  switch(data)
				{
					case 'R':
						status=0;	
					SetFlashUpdateFlag(1);
						break;
					default:
						break;
				}
				break;
			case 4:
				if(data=='T')
					status++;
				else
					status=0;
				break;
			case 5:
				if(data=='0')
					status++;
				else
					status=0;
				break;
			case 6:
				if(data=='0')
				{
					SetCommand(1);
				}
					status=0;
				break;
			default:
				status=0;
				break;
		}
	}

}


void USART3_IRQHandler(void)
{
	//uint8_t data = 0;
	if(USART_GetITStatus(USART3, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit( USART3,USART_IT_RXNE);
		//data=USART_ReceiveData(USART3);
	}
	 
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

