#include "spi.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "config.h"

/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

void ICM_SPIInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  
  /* 使能SPI和GPIO的时钟------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* 引脚复用为SPI1----------------------------------------------*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  
  /* 硬件SPI1引脚配置---------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* SPI配置------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				/* 双线双向全双工									*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* 主SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI接收8位帧结构								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																/* 串行同步时钟的空闲状态为低电平	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;															/* 第一个跳变沿数据被采样					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;				/* 预分频  陀螺仪最大10MHz													*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  SPI_Cmd(SPI1, ENABLE);
}
void SPI1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  
  /* 使能SPI和GPIO的时钟------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* 引脚复用为SPI1----------------------------------------------*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  
  /* 硬件SPI1引脚配置---------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* SPI配置------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				/* 双线双向全双工									*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* 主SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;													/* SPI接收8位帧结构								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																/* 串行同步时钟的空闲状态为低电平	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* 第一个跳变沿数据被采样					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;				/* 预分频  陀螺仪最大10MHz													*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
  SPI_InitStructure.SPI_CRCPolynomial = 10;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  SPI_Cmd(SPI1, ENABLE);
}

void SPI2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  
  /* 使能SPI和GPIO的时钟------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* 引脚复用为SPI1----------------------------------------------*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
  
  /* 硬件SPI1引脚配置---------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
	/* SPI配置------------------------------------------------------*/
	SPI_I2S_DeInit(SPI2);
	#ifdef TLE5012_USED
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				/* 双线双向全双工									*/
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* 主SPI													*/
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;													/* SPI接收8位帧结构								*/
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																/* 串行同步时钟的空闲状态为低电平	*/
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* 第一个跳变沿数据被采样					*/
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;				/* 预分频  陀螺仪最大10MHz													*/
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
		SPI_InitStructure.SPI_CRCPolynomial = 10;
	#else
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;				    /* 双线双向全双工									*/
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* 主SPI													*/
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI接收8位帧结构								*/
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;																/* 串行同步时钟的空闲状态为低电平	*/
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;															/* 第二个跳变沿数据被采样					*/
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;				/* 预分频	168M/64	*/
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
		SPI_InitStructure.SPI_CRCPolynomial = 7;
	#endif
	SPI_Init(SPI2, &SPI_InitStructure);
	
  SPI_Cmd(SPI2, ENABLE);
}

void SPI3_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  
  /* 使能SPI和GPIO的时钟------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* 引脚复用为SPI1----------------------------------------------*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
  
  /* 硬件SPI1引脚配置---------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* SPI配置------------------------------------------------------*/
  SPI_I2S_DeInit(SPI3);
	#ifdef TLE5012_USED
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				/* 双线双向全双工									*/
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* 主SPI													*/
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;													/* SPI接收8位帧结构								*/
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																/* 串行同步时钟的空闲状态为低电平	*/
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* 第一个跳变沿数据被采样					*/
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;				/* 预分频  陀螺仪最大10MHz													*/
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
		SPI_InitStructure.SPI_CRCPolynomial = 10;
	#else
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;				    /* 双线双向全双工									*/
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* 主SPI													*/
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI接收8位帧结构								*/
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;																/* 串行同步时钟的空闲状态为低电平	*/
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;															/* 第二个跳变沿数据被采样					*/
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;				/* 预分频	168M/64	*/
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
		SPI_InitStructure.SPI_CRCPolynomial = 7;
	#endif
  SPI_Init(SPI3, &SPI_InitStructure);
  
  SPI_Cmd(SPI3, ENABLE);
}

/**
* @brief  SPI1的波特率预分频设置
*
* @param[in] 波特率预分频值
* @retval None
*/
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));					/* 判断有效性												*/
  SPI1->CR1 &= 0XFFC7;																										/* 位3-5清零，用来设置波特率*/
  SPI1->CR1 |= SPI_BaudRatePrescaler;																			
  SPI_Cmd(SPI1,ENABLE); 																									
} 

/**
* @brief     向SPI从机设备写入数据
* @param     SPI: SPIx
* @param     GPIOx,GPIO_Pin: 片选引脚
* @param     address: 要写入得寄存器的地址
* @param     value：要写入的值
* @retval    None
*/                            
void SPI_Write(SPI_TypeDef *SPI,
               GPIO_TypeDef *GPIOx,
               uint16_t GPIO_Pin,
               uint8_t address,
               uint8_t value)
{
//  SPI_Cmd(SPI,ENABLE);	
  GPIO_ResetBits(GPIOx,GPIO_Pin);
  
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET){}		//等待发送区空  
  
  SPI_I2S_SendData(SPI, address); 																		//通过外设SPIx发送一个byte  数据
  
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
  
  SPI_I2S_ReceiveData(SPI); 																	        //返回通过SPIx最近接收的数据
  
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET){}		//等待发送区空  
  
  SPI_I2S_SendData(SPI, value); 																		//通过外设SPIx发送一个byte  数据
  
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
  
  SPI_I2S_ReceiveData(SPI); 		
  
	/*待定，屏蔽部分是罗潇逸在研究ADI陀螺仪时写的，不写读不出  */
  // while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_BSY) == SET){}	
//  SPI_Cmd(SPI,DISABLE);		
  
	GPIO_SetBits(GPIOx,GPIO_Pin);	
}




/**
* @brief     向SPI从机设备读取数据
* @param     SPI: SPIx
* @param     GPIOx,GPIO_Pin: 片选引脚
* @param     address: 要读取的寄存器的地址
* @retval    value：寄存器的值
*/
uint8_t SPI_Read(SPI_TypeDef *SPIx,
                 GPIO_TypeDef* GPIOx,
                 uint16_t GPIO_Pin,
                 uint8_t address)
{
  
  uint8_t data;
  
  address |= (uint8_t)READWRITE_CMD;
//  SPI_Cmd(SPIx,ENABLE);
  
  GPIO_ResetBits(GPIOx,GPIO_Pin);
  
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
  
  SPI_I2S_SendData(SPIx, address);	 																		//通过外设SPIx发送一个byte  数据
  
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
  
  SPI_I2S_ReceiveData(SPIx);																		        //返回通过SPIx最近接收的数据
  
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
  
  SPI_I2S_SendData(SPIx, DUMMY_BYTE); 																	//通过外设SPIx发送一个byte  数据
  
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
  
  data = SPI_I2S_ReceiveData(SPIx); 		
  
//  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){}	
//  SPI_Cmd(SPIx,DISABLE);
  GPIO_SetBits(GPIOx,GPIO_Pin);
  
  return data;
}


void SPI_MultiRead(SPI_TypeDef *SPIx,
                   GPIO_TypeDef* GPIOx,
                   uint16_t GPIO_Pin,
                   uint8_t address,
                   uint8_t* data,
                   uint32_t 	len)
{  
  /*
  SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). 
  The first byte contains the SPI Address, and the following byte(s) contain(s) the SPI data. 
  The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation. 
  */
  address |= (uint8_t)READWRITE_CMD;
  
  SPI_Cmd(SPIx,ENABLE);	
  GPIO_ResetBits(GPIOx,GPIO_Pin);
	
	StartCount();
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
	{
		if(getCount()>1000)
		{
			DeadWhileReport(38);
			break;
		}
	}
	EndCnt(); 
  
  SPI_I2S_SendData(SPIx, address); 																		//通过外设SPIx发送一个byte  数据
  
	StartCount();
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
	{
		if(getCount()>1000)
		{
			DeadWhileReport(39);
			break;
		}
	}
	EndCnt(); 
  
  SPI_I2S_ReceiveData(SPIx); 																	        //返回通过SPIx最近接收的数据
  
  /* Receive the data that will be read from the device (MSB First) */
  for(uint32_t i=0;i<len;i++)
  {
    StartCount();
		while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
		{
			if(getCount()>1000)
			{
				DeadWhileReport(40);
				break;
			}
		}
		EndCnt();   
    
    SPI_I2S_SendData(SPIx, DUMMY_BYTE); 																	//通过外设SPIx发送一个byte  数据
    
    
		StartCount();
		while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
		{
			if(getCount()>1000)
			{
				DeadWhileReport(41);
				break;
			}
		}
		EndCnt();   
    
    data[i]=SPI_I2S_ReceiveData(SPIx); 																	        //返回通过SPIx最近接收的数据
  }
  StartCount();
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET)
	{
		if(getCount()>1000)
		{
			DeadWhileReport(42);
			break;
		}
	}
	EndCnt(); 
  SPI_Cmd(SPIx,DISABLE);		
  GPIO_SetBits(GPIOx,GPIO_Pin);	
}


/**
* @brief  SPI的片选引脚配置
*
* @param  None
* @retval None
*/
void CS_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
  
  /* 配置片选引脚------------------------- */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);                 //ICM20608G
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;            
  GPIO_Init(GPIOB, &GPIO_InitStructure);   
  
  /* Deselect : Chip Select high ---------*/
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  GPIO_SetBits(GPIOA, GPIO_Pin_15);
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}


uint16_t	SPIx_RdWr_HalfWord(SPI_TypeDef* SPIx, uint16_t hw)
{
  uint16_t u16Temp;
	
//	printf("\r\n @Sky [SPIx_RdWr_HalfWord] write：0x%4X \r\n", hw);
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
  
  if (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == SET)
  {
    SPI_I2S_ReceiveData(SPIx);	// 如果SPI_DR 不空，则清除
  }

  /* Send byte through the SPIx peripheral */
  SPI_I2S_SendData(SPIx, hw);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
 
  u16Temp = SPI_I2S_ReceiveData(SPIx);

  /* Wait to RXNE 建立 */
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == SET);
 
//	printf("\r\n @Sky [SPIx_RdWr_HalfWord] read：0x%4X \r\n", u16Temp);
	
  /* Return the byte read from the SPI bus */
  return (u16Temp);	
}

