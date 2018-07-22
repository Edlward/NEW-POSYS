#include "usart.h"
#include "arm_math.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stdarg.h"
#include "stdio.h"
#include "stdint.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "config.h"


static uint8_t dmaSendBuffer[DMA_SEND_SIZE];

void USART_SendDataToDMA_USART3(uint8_t data)
{
	static uint8_t tempBuffer[DMA_SEND_SIZE];
	static uint32_t count=0;
	tempBuffer[count]=data;
	count++;
	
	if(count>=DMA_SEND_SIZE)
	{
		StartCount();
		while(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3) == RESET	&&	DMA_GetCmdStatus(DMA1_Stream3)	==	ENABLE)
		{
			if(getCount()>1000)
			{
				DeadWhileReport(33);
			}
		}
		EndCnt();
		DMA_ClearFlag(DMA1_Stream3,DMA_IT_TCIF3);  
		DMA_Cmd(DMA1_Stream3,DISABLE);  
		count=0;
		memcpy(dmaSendBuffer,tempBuffer,DMA_SEND_SIZE);
		DMA_SetCurrDataCounter(DMA1_Stream3,DMA_SEND_SIZE);
		DMA_Cmd(DMA1_Stream3,ENABLE);
	}
	
}
void USART3_Init(uint32_t BaudRate);
void USART3DMAInit(uint32_t BaudRate)
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
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
  
  //����3��Ӧ���Ÿ���ӳ��
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOD8����ΪUSART3
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11, GPIO_AF_USART3); //GPIOD9����ΪUSART3
  
  //USART3�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOD8��GPIOD9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
  GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PD8��PD9
  
  //USART3 ��ʼ������
  USART_InitStructure.USART_BaudRate = BaudRate;//����������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������3
  
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3
  
  USART_ClearFlag(USART3, USART_FLAG_TC);
  
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�
  
  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����3�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

void USART_SendDataToDMA_USATR1(uint8_t data)
{
	static uint8_t tempBuffer[DMA_SEND_SIZE];
	static uint32_t count=0;
	tempBuffer[count]=data;
	count++;
	
	if(count>=DMA_SEND_SIZE)
	{
		StartCount();
		while(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) == RESET	&&	DMA_GetCmdStatus(DMA2_Stream7)	==	ENABLE)
		{
			if(getCount()>1000)
			{
				DeadWhileReport(34);
			}
		}
		EndCnt(); 
		DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
		DMA_Cmd(DMA2_Stream7,DISABLE);  
		count=0;
		memcpy(dmaSendBuffer,tempBuffer,DMA_SEND_SIZE);
		DMA_SetCurrDataCounter(DMA2_Stream7,DMA_SEND_SIZE);
		DMA_Cmd(DMA2_Stream7,ENABLE);
	}
	
}

void USART1DMAInit(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹŜGPIOAʱד
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹŜUSART1ʱד
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
//	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);  	

	DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
	DMA_Cmd(DMA2_Stream7,DISABLE);
	
	//Ԯ�?הӦӽޅشԃӳʤ
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA9شԃΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_USART1); //GPIOA10شԃΪUSART1
	
	//USART1׋ࠚƤ�?
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9; //GPIOA9ԫGPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//شԃ٦Ŝ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//̙׈50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //ΆάشԃˤԶ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //ʏ-
	GPIO_Init(GPIOA,&GPIO_InitStructure); //ԵʼۯPA9ìPA10

   //USART1 Եʼۯʨ׃
	USART_InitStructure.USART_BaudRate = BaudRate;//Ҩ͘Êʨ׃
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//ؖӤΪ8λ˽ߝٱʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һٶֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//ϞǦżУҩλ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ϞӲݾ˽ߝ·࠘�?
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//˕עģʽ
  USART_Init(USART1, &USART_InitStructure); //ԵʼۯԮ�?

	//Usart1 NVIC Ƥ׃
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//Ԯ�?א׏ͨր
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//ȀռԅЈܶ3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//ؓԅЈܶ3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨրʹŜ
	NVIC_Init(&NVIC_InitStructure);	//ٹߝָ֨քӎ˽ԵʼۯVIC݄զǷb
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//ߪǴРژא׏
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ClearFlag(USART1, USART_FLAG_TXE);
	
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART1, ENABLE);  //ʹŜԮ�?
  
}


void USART2_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOCʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
  
  //����3��Ӧ���Ÿ���ӳ��
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOD8����ΪUSART2
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2); //GPIOD9����ΪUSART2
  
  //USART2�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOD8��GPIOD9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
  GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PD8��PD9
  
  //USART2 ��ʼ������
  USART_InitStructure.USART_BaudRate = BaudRate;//����������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������3
  
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���3
  
  USART_ClearFlag(USART2, USART_FLAG_TC);
  
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�
  
  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����3�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

void USART2DMAInit(uint32_t BaudRate)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	USART2_Init(BaudRate);
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  
	
	DMA_DeInit(DMA1_Stream6);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
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
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);    
//	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);  	

	DMA_ClearFlag(DMA1_Stream6,DMA_IT_TCIF6);  
	DMA_Cmd(DMA1_Stream6,DISABLE);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_ClearFlag(USART2, USART_FLAG_TXE);
	
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART2, ENABLE); 
  
}
void USART_SendDataToDMA_USART2(uint8_t data)
{
	static uint8_t tempBuffer[DMA_SEND_SIZE];
	static uint32_t count=0;
	tempBuffer[count]=data;
	count++;
	
	if(count>=DMA_SEND_SIZE)
	{
		StartCount();
		while(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6) == RESET	&&	DMA_GetCmdStatus(DMA1_Stream6)	==	ENABLE)
		{
			if(getCount()>1000)
			{
				DeadWhileReport(35);
			}
		}
		EndCnt(); 
		DMA_ClearFlag(DMA1_Stream6,DMA_IT_TCIF6);  
		DMA_Cmd(DMA1_Stream6,DISABLE);  
		count=0;
		memcpy(dmaSendBuffer,tempBuffer,DMA_SEND_SIZE);
		DMA_SetCurrDataCounter(DMA1_Stream6,DMA_SEND_SIZE);
		DMA_Cmd(DMA1_Stream6,ENABLE);
	}
}
/**
* @brief  Retargets the C library printf function to the USART.
* @param  None
* @retval None
*/

//PD8

void USART_OUT_F(float value)
{
  char s[10]={0};
  sprintf(s,"%f\t",value);
  USART_OUT(USART_USED,s);
}
void USART_Enter(void){
  USART_OUT(USART_USED,"\r\n");
}
void USART_OUTByDMAF(float x){
     const char *s;
		 char String[20]={0};
		 sprintf(String,"%f\t",x);
		 for (s=String; *s; s++) 
		 {
				USART_SendDataToDMA_USART2(*s);
     }
}

void USART_EnterByDMA(void){
	USART_SendDataToDMA_USART2('\r');
	USART_SendDataToDMA_USART2('\n');
}


void UART5_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIOCʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//ʹ��UART5ʱ��
  
  //����3��Ӧ���Ÿ���ӳ��
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOD8����ΪUART5
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource2, GPIO_AF_UART5); //GPIOD9����ΪUART5
  
  //UART5�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ; //GPIOD8��GPIOD9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
  GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PD8��PD9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; //GPIOD8��GPIOD9
  GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��PD8��PD9
  
  //UART5 ��ʼ������
  USART_InitStructure.USART_BaudRate = BaudRate;//����������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART5, &USART_InitStructure); //��ʼ������3
  
  USART_Cmd(UART5, ENABLE);  //ʹ�ܴ���3
  
  USART_ClearFlag(UART5, USART_FLAG_TC);
  
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//��������ж�
  
  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//����3�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}


void USART6_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART6ʱ��
  
  //����3��Ӧ���Ÿ���ӳ��
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOD8����ΪUSART6
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_USART6); //GPIOD9����ΪUSART6
  
  //USART6�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOD8��GPIOD9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
  GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PD8��PD9
  
  //USART6 ��ʼ������
  USART_InitStructure.USART_BaudRate = BaudRate;//����������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART6, &USART_InitStructure); //��ʼ������3
  
  USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���3
  
  USART_ClearFlag(USART6, USART_FLAG_TC);
  
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�
  
  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����3�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}


void USART_OUT(USART_TypeDef* USARTx,const char *Data,...){ 
  const char *s;
  int d;
  char buf[16];
  va_list ap;
  va_start(ap, Data);
  
  while(*Data!=0){				                          //�ж��Ƿ񵽴��ַ���������
    if(*Data==0x5c){									  //'\'
      switch (*++Data){
      case 'r':							          //�س���
        USART_SendData(USARTx, 0x0d);	   
        
        Data++;
        break;
      case 'n':							          //���з�
        USART_SendData(USARTx, 0x0a);	
        Data++;
        break;
        
      default:
        Data++;
        break;
      }
      
      
    }
    else if(*Data=='%'){									  //
      switch (*++Data){				
      case 's':										  //�ַ���
        s = va_arg(ap, const char *);
        for ( ; *s; s++) {
          USART_SendData(USARTx,*s);
          while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
        }
        Data++;
        break;
      case 'd':										  //ʮ����
        d = va_arg(ap, int);
        itoa(d, buf, 10);
        for (s = buf; *s; s++) {
          USART_SendData(USARTx,*s);
          while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
        }
        Data++;
        break;
      default:
        Data++;
        break;
      }		 
    }
    else USART_SendData(USARTx, *Data++);
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
  }
}

/******************************************************
��������ת�ַ�������
char *itoa(int value, char *string, int radix)
radix=10 ��ʾ��10����	��ʮ���ƣ�ת�����Ϊ0;  

����d=-379;
ִ��	itoa(d, buf, 10); ��

buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
  int     i, d;
  int     flag = 0;
  char    *ptr = string;
  
  /* This implementation only works for decimal numbers. */
  if (radix != 10)
  {
    *ptr = 0;
    return string;
  }
  
  if (!value)
  {
    *ptr++ = 0x30;
    *ptr = 0;
    return string;
  }
  
  /* if this is a negative value insert the minus sign. */
  if (value < 0)
  {
    *ptr++ = '-';
    
    /* Make the value positive. */
    value *= -1;
  }
  
  for (i = 10000; i > 0; i /= 10)
  {
    d = value / i;
    
    if (d || flag)
    {
      *ptr++ = (char)(d + 0x30);
      value -= (d * i);
      flag = 1;
    }
  }
  
  /* Null terminate the string. */
  *ptr = 0;
  
  return string;
  
} 




