/**
  ******************************************************************************
  * @file    usart.cpp
  * @author  Luo Xiaoyi and Qiao Zhijian 
  * @version V1.0
  * @date    2016.10.26
  * @brief   用于控制串口
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "string.h"
#include "stdio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "arm_math.h"
#include "string.h"
#include "flash.h"
#include "user.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
#ifndef HEX_SEND
	#define DMA_SEND_SIZE   200
	#define USART_PRINT(a)  (USART_SendDataToDMA_USART1(a))
#else
	#define DMA_SEND_SIZE   28
	#define USART_PRINT(a)	
#endif

/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
#ifdef DEBUG
  static _out_stream  usart_io;
#endif

static uint8_t dmaSendBuffer[DMA_SEND_SIZE];

/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

void USART_SendDataToDMA_USART1(uint8_t data)
{
	static uint8_t tempBuffer[DMA_SEND_SIZE];
	static uint32_t count=0;
	tempBuffer[count]=data;
	count++;
	
	if(count>=DMA_SEND_SIZE)
	{
		while(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) == RESET	&&	DMA_GetCmdStatus(DMA2_Stream7)	==	ENABLE	);    
		DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
		DMA_Cmd(DMA2_Stream7,DISABLE);  
		count=0;
		memcpy(dmaSendBuffer,tempBuffer,DMA_SEND_SIZE);
		DMA_SetCurrDataCounter(DMA2_Stream7,DMA_SEND_SIZE);
		DMA_Cmd(DMA2_Stream7,ENABLE);
	}
	
}

/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

void USART1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);    
	
	DMA_DeInit(DMA2_Stream7);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dmaSendBuffer;  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;   
	DMA_InitStructure.DMA_BufferSize = DMA_SEND_SIZE;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
		 
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   	 
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);    

	DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
	DMA_Cmd(DMA2_Stream7,DISABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_USART1); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

   
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	//Usart1 NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ClearFlag(USART1, USART_FLAG_TXE);
	
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART1, ENABLE);  
}

void USART1_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART1,USART_IT_RXNE)==SET)
	{
		USART_ClearITPendingBit( USART1,USART_IT_RXNE);
		uint8_t dataR=USART_ReceiveData(USART1);
		usartCmdInput(dataR);
	}
}

void USART6_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);

  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_USART6);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
  GPIO_Init(GPIOC,&GPIO_InitStructure); 
  
  USART_InitStructure.USART_BaudRate = BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  USART_Init(USART6, &USART_InitStructure); 
  
  USART_Cmd(USART6, ENABLE);  
  
  USART_ClearFlag(USART6, USART_FLAG_TC);
  
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
  NVIC_Init(&NVIC_InitStructure);	
}

void USART6DMAInit(uint32_t BaudRate)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	USART6_Init(BaudRate);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
	
	DMA_DeInit(DMA2_Stream6);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dmaSendBuffer;  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;   
	DMA_InitStructure.DMA_BufferSize = DMA_SEND_SIZE;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
		 
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   	 
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);    
//	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);  	

	DMA_ClearFlag(DMA2_Stream6,DMA_IT_TCIF6);  
	DMA_Cmd(DMA2_Stream6,DISABLE);

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	USART_ClearFlag(USART6, USART_FLAG_TC);
	USART_ClearFlag(USART6, USART_FLAG_TXE);
	
	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART6, ENABLE); 
  
}
void USART_SendDataToDMA_USATR6(uint8_t data)
{
	static uint8_t tempBuffer[DMA_SEND_SIZE];
	static uint32_t count=0;
	tempBuffer[count]=data;
	count++;
	
	if(count>=DMA_SEND_SIZE)
	{
		while(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6) == RESET	&&	DMA_GetCmdStatus(DMA2_Stream6)	==	ENABLE);
		DMA_ClearFlag(DMA2_Stream6,DMA_IT_TCIF6);  
		DMA_Cmd(DMA2_Stream6,DISABLE);  
		count=0;
		memcpy(dmaSendBuffer,tempBuffer,DMA_SEND_SIZE);
		DMA_SetCurrDataCounter(DMA2_Stream6,DMA_SEND_SIZE);
		DMA_Cmd(DMA2_Stream6,ENABLE);
	}
}



#ifdef DEBUG
/**
	* @brief
	* @none
	* @retval  usart_io
  */
_out_stream& getUsartOut(void)
{
	return usart_io;
}

const _out_stream& _out_stream::operator<<(const int32_t value) const
{
	char buf[10];
	uint8_t len;
	len=sprintf(buf,"%d",value);
	for(uint8_t i=0;i<len;i++)
	{
		USART_PRINT(buf[i]);
	}
	return usart_io;
}
const _out_stream& _out_stream::operator<<(const uint32_t value) const
{
	char buf[10];
	uint8_t len;
	len=sprintf(buf,"%u",value);
	for(uint8_t i=0;i<len;i++)
	{
		USART_PRINT(buf[i]);
	}
	return usart_io;
}
/**
	* @brief   
	* @param   value: 
	* @retval  usart_io
  */
const _out_stream& _out_stream::operator<<(const float value) const
{
	char buf[20];
	uint8_t len;
	len=sprintf(buf,"%f",value);
	for(uint8_t i=0;i<len;i++)
	{
		USART_PRINT(buf[i]);
	}
	return usart_io;
}
/**
	* @brief   重载<<，解析字符串，并将其发送到dma
	* @param   value: 需要解析的字符串
	* @retval  usart_io
  */
const _out_stream& _out_stream::operator<<(const char* value) const
{
	uint8_t len=0;
	while(1)
	{
		if(value[len]!='\0')
		  USART_PRINT(value[len]);
		else
			break;
		len++;
		
		if(len>100)
			break;
	}
	return usart_io;
}
/**
	* @brief   重载<<，解析字符型数据，并将其发送到dma
	* @param   value: 需要解析的字符型数据
	* @retval  usart_io
  */
const _out_stream& _out_stream::operator<<(const char value) const
{
  USART_PRINT(value);
	return usart_io;
}
#endif
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
