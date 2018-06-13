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
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
  
  //串口3对应引脚复用映射
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOD8复用为USART3
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11, GPIO_AF_USART3); //GPIOD9复用为USART3
  
  //USART3端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOD8与GPIOD9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PD8，PD9
  
  //USART3 初始化设置
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

void USART_SendDataToDMA_USART1(uint8_t data)
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
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使艤GPIOA时讚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使艤USART1时讚
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
	
	//援酄?讛应咏迏卮詢映胜
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA9卮詢为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_USART1); //GPIOA10卮詢为USART1
	
	//USART1讒酄毱ぷ?
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9; //GPIOA9垣GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//卮詢佴艤
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//虣讏50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //螁维卮詢摔远
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //蕪-
	GPIO_Init(GPIOA,&GPIO_InitStructure); //缘始郫PA9矛PA10

   //USART1 缘始郫狮變
	USART_InitStructure.USART_BaudRate = BaudRate;//舀蜆脢狮變
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//貣婴为8位私邼俦式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一俣停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//蠟铅偶校药位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//蠟硬菥私邼路酄樧?
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//藭注模式
  USART_Init(USART1, &USART_InitStructure); //缘始郫援酄?

	//Usart1 NVIC 皮變
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//援酄?讗讖通謤
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//葊占詤袌芏3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//負詤袌芏3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通謤使艤
	NVIC_Init(&NVIC_InitStructure);	//俟邼指吱謩訋私缘始郫VIC輨咋欠b
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//擢谴袪跇讗讖
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ClearFlag(USART1, USART_FLAG_TXE);
	
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART1, ENABLE);  //使艤援酄?
  
}


void USART6_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
  
  //串口3对应引脚复用映射
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOD8复用为USART6
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_USART6); //GPIOD9复用为USART6
  
  //USART6端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOD8与GPIOD9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PD8，PD9
  
  //USART6 初始化设置
  USART_InitStructure.USART_BaudRate = BaudRate;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART6, &USART_InitStructure); //初始化串口3
  
  USART_Cmd(USART6, ENABLE);  //使能串口3
  
  USART_ClearFlag(USART6, USART_FLAG_TC);
  
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断
  
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口3中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
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
		StartCount();
		while(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6) == RESET	&&	DMA_GetCmdStatus(DMA2_Stream6)	==	ENABLE)
		{
			if(getCount()>1000)
			{
				DeadWhileReport(35);
			}
		}
		EndCnt(); 
		DMA_ClearFlag(DMA2_Stream6,DMA_IT_TCIF6);  
		DMA_Cmd(DMA2_Stream6,DISABLE);  
		count=0;
		memcpy(dmaSendBuffer,tempBuffer,DMA_SEND_SIZE);
		DMA_SetCurrDataCounter(DMA2_Stream6,DMA_SEND_SIZE);
		DMA_Cmd(DMA2_Stream6,ENABLE);
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
				USART_SendDataToDMA_USATR6(*s);
     }
}

void USART_EnterByDMA(void){
	USART_SendDataToDMA_USATR6('\r');
	USART_SendDataToDMA_USATR6('\n');
}
void USART_OUT(USART_TypeDef* USARTx,const char *Data,...){ 
  const char *s;
  int d;
  char buf[16];
  va_list ap;
  va_start(ap, Data);
  
  while(*Data!=0){				                          //判断是否到达字符串结束符
    if(*Data==0x5c){									  //'\'
      switch (*++Data){
      case 'r':							          //回车符
        USART_SendData(USARTx, 0x0d);	   
        
        Data++;
        break;
      case 'n':							          //换行符
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
      case 's':										  //字符串
        s = va_arg(ap, const char *);
        for ( ; *s; s++) {
          USART_SendData(USARTx,*s);
          while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
        }
        Data++;
        break;
      case 'd':										  //十进制
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
整形数据转字符串函数
char *itoa(int value, char *string, int radix)
radix=10 标示是10进制	非十进制，转换结果为0;  

例：d=-379;
执行	itoa(d, buf, 10); 后

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




