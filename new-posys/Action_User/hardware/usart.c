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

/**
* @brief  Retargets the C library printf function to the USART.
* @param  None
* @retval None
*/

void USART_SendDataToDMA(uint8_t data)
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

//PD8
void USART1_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹŜGPIOAʱד
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹŜUSART1ʱד
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
	
//	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
//	NVIC_Init(&NVIC_InitStructure);  
	
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
	
	//Ԯࠚ1הӦӽޅشԃӳʤ
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA9شԃΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_USART1); //GPIOA10شԃΪUSART1
	
	//USART1׋ࠚƤ׃
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
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ϞӲݾ˽ߝ·࠘׆
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//˕עģʽ
  USART_Init(USART1, &USART_InitStructure); //ԵʼۯԮࠚ1

	//Usart1 NVIC Ƥ׃
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//Ԯࠚ1א׏ͨր
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//ȀռԅЈܶ3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//ؓԅЈܶ3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨրʹŜ
	NVIC_Init(&NVIC_InitStructure);	//ٹߝָ֨քӎ˽ԵʼۯVIC݄զǷb
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//ߪǴРژא׏
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ClearFlag(USART1, USART_FLAG_TXE);
	
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART1, ENABLE);  //ʹŜԮࠚ3
  
}

void USART_OUT_F(float value)
{
  char s[10]={0};
  sprintf(s,"%f\t",value);
  USART_OUT(USART1,s);
}
void USART_Enter(void){
  USART_OUT(USART1,"\r\n");
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




