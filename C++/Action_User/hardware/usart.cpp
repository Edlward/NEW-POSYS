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
	#define USART_PRINT(a)  (USART_SendDataToDMA(a))
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

void USART_SendDataToDMA(uint8_t data)
{
	static uint8_t tempBuffer[DMA_SEND_SIZE];
	static uint32_t count=0;
	tempBuffer[count]=data;
	count++;
	
	if(count>=DMA_SEND_SIZE)
	{
		while(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3) == RESET	&&	DMA_GetCmdStatus(DMA1_Stream3)	==	ENABLE	);    
		DMA_ClearFlag(DMA1_Stream3,DMA_IT_TCIF3);  
		DMA_Cmd(DMA1_Stream3,DISABLE);  
		count=0;
		memcpy(dmaSendBuffer,tempBuffer,DMA_SEND_SIZE);
		DMA_SetCurrDataCounter(DMA1_Stream3,DMA_SEND_SIZE);
		DMA_Cmd(DMA1_Stream3,ENABLE);
	}
	
}

void USART3_Init(uint32_t BaudRate);
	
void USART3_DMA_Init(uint32_t BaudRate)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	USART3_Init(BaudRate);
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  
	
	DMA_DeInit(DMA1_Stream3);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);  
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
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);    
//	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);  	

	DMA_ClearFlag(DMA1_Stream3,DMA_IT_TCIF3);  
	DMA_Cmd(DMA1_Stream3,DISABLE);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_TC);
	USART_ClearFlag(USART3, USART_FLAG_TXE);
	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART3, ENABLE); 
  
}


void USART3_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
  
  //串口3对应引脚复用映射
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOD8复用为USART3
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11, GPIO_AF_USART3); //GPIOD9复用为USART3
  
  //USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOD8与GPIOD9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PD8，PD9
  
  //USART1 初始化设置
  USART_InitStructure.USART_BaudRate = BaudRate;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口3
  
  USART_Cmd(USART3, ENABLE);  //使能串口3
  
  USART_ClearFlag(USART3, USART_FLAG_TC);
  
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
  
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//void DMA2_Stream7_IRQHandler(void)
//{
//	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET)   
//	{  
//			DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
//			DMA_Cmd(DMA2_Stream7,DISABLE);  
//	}  
//}
void USART3_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART3,USART_IT_RXNE)==SET)
	{
		USART_ClearITPendingBit( USART3,USART_IT_RXNE);
		uint8_t dataR=USART_ReceiveData(USART3);
		usartCmdInput(dataR);
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
