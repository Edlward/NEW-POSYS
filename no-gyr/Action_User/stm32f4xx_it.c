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
#include "can.h"
#include "config.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

static int32_t Vel[3];
static uint32_t Pos[3];
static uint8_t flag=0;
union Position
{
	uint8_t  Data8[2];
	uint16_t Data16;
}pos;
union Vell
{
	uint8_t  Data8[2];
	int16_t  Data16;
}vell;
uint16_t encoder_right,encoder_left;
int16_t vell_right,vell_left;


void CAN1_RX0_IRQHandler(void)
{
	
	static uint8_t buffer[8];
	static uint32_t StdId=0;
	static int elmoVel=0;
	static uint32_t elmoPos=0;
union MSG
{
	uint8_t data8[8];
	int data32[2];
	float dataf[2];
}msg;
	CAN_RxMsg(CAN1, &StdId,buffer,8);

	if(StdId == 0x16)	//
	{
		pos.Data8[0]=buffer[0];
		pos.Data8[1]=buffer[1];
		encoder_right=pos.Data16;
		
		vell.Data8[0]=buffer[2];
		vell.Data8[1]=buffer[3]; 
		vell_right=vell.Data16;	
	}
	
	if(StdId == 0x18)	//
	{
		pos.Data8[0]=buffer[0];
		pos.Data8[1]=buffer[1];
		encoder_right=pos.Data16;
		
		vell.Data8[0]=buffer[2];
		vell.Data8[1]=buffer[3]; 
		vell_left=vell.Data16;	
	}
			if(StdId == 0x281)
		{
			for(uint8_t i = 0; i < 8; i++)
				msg.data8[i] = buffer[i];
			if(msg.data32[0] == 0x40005856)
			{
				//实际速度
				flag++;
				elmoVel= msg.data32[1];
				Vel[0]=elmoVel;
			}
				else if(msg.data32[0] == 0x40005850)
			{
				//实际位置	
				  flag++;
					elmoPos=msg.data32[1];
			   	Pos[0]=elmoPos;
			}
		}else if(StdId == 0x282)
		{
			for(uint8_t i = 0; i < 8; i++)
				msg.data8[i] = buffer[i];
			if(msg.data32[0] == 0x40005856)
			{
				//实际速度
				flag++;
				elmoVel= msg.data32[1];
				Vel[2]=elmoVel;
			}
				else if(msg.data32[0] == 0x40005850)
			{
				//实际位置
				  flag++;
					elmoPos=msg.data32[1];
			   	Pos[2]=elmoPos;
			}
		}else if(StdId == 0x283)
		{
			for(uint8_t i = 0; i < 8; i++)
				msg.data8[i] = buffer[i];
			if(msg.data32[0] == 0x40005856)
			{
				//实际速度
				flag++;
				elmoVel= msg.data32[1];
				Vel[1]=elmoVel;
			}
				else if(msg.data32[0] == 0x40005850)
			{
				//实际位置
				  flag++;
					elmoPos=msg.data32[1];
			   	Pos[1]=elmoPos;
			}
		}
	CAN_ClearFlag(CAN1,CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1,CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1,CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1,CAN_FLAG_LEC);
	
	CAN_ClearFlag(CAN1,CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1,CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1,CAN_FLAG_FOV1);
}

int* GetVal(void)
{
return Vel;
}

uint32_t* GetPos(void)
{
return Pos;
}

uint8_t getFlagFinish(void){
	if(flag==3)
	{
		flag=0;
		return 1;
	}else
		return 0;
}

/*************定时器2******start************/
//每1ms调用一次  用于读取编码器的值和计算坐标

//int posx,posy;
static uint16_t timeCount=0;
static uint8_t timeFlag=0;
static uint8_t cpuTimes=0;
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
  {	
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		timeCount++;
		if(timeCount>=PERIOD*1000)
		{
			cpuTimes++;
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
		cpuTimes=0;
		return 1;
	}
	return 0;
}

uint32_t getTimeCount(void)
{
	return (timeCount+cpuTimes*PERIOD*1000);
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



void USART1_IRQHandler(void)
{
	//uint8_t data = 0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit( USART1,USART_IT_RXNE);
		//data=USART_ReceiveData(USART1);
	}
	else{
		USART_ReceiveData(USART1);
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

