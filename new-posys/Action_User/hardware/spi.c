#include "spi.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "config.h"

#ifdef SINGLESYSTEM
/*
设置详细请查看参考手册
*/
void SPI1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	/* 使能SPI和GPIOB的时钟------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//84MHz
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* 引脚复用为SPI2----------------------------------------------*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	
	/* 硬件SPI1引脚配置---------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* SPI配置------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;				    /* 双线双向全双工									*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* 主SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI接收8位帧结构								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;															/* 串行同步时钟的空闲状态为高电平	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* 第二个跳变沿数据被采样					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;				/* 预分频	84/128	*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_Cmd(SPI1, ENABLE);

}
/*
设置详细请查看参考手册
*/
void SPI2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	/* 使能SPI和GPIOB的时钟------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//42MHz
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* 引脚复用为SPI2----------------------------------------------*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
	/* 硬件SPI1引脚配置---------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI配置------------------------------------------------------*/
  SPI_I2S_DeInit(SPI2);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				    /* 双线双向全双工									*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* 主SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI接收8位帧结构								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;																/* 串行同步时钟的空闲状态为高电平	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* 第一个跳变沿数据被采样					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;				/* 预分频	42/32	*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  SPI_Cmd(SPI2, ENABLE);
}
#else
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
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI接收8位帧结构								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;																/* 串行同步时钟的空闲状态为高电平	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* 第一个跳变沿数据被采样					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;				/* 预分频													*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_Cmd(SPI1, ENABLE);
}


void SPI2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	/* 使能SPI和GPIOB的时钟------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* 引脚复用为SPI2----------------------------------------------*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	
	/* 硬件SPI1引脚配置---------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI配置------------------------------------------------------*/
  SPI_I2S_DeInit(SPI2);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;				    /* 双线双向全双工									*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* 主SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI接收8位帧结构								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																/* 串行同步时钟的空闲状态为低电平	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* 第二个跳变沿数据被采样					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;				/* 预分频	168M/64	*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  SPI_Cmd(SPI2, ENABLE);
}
#endif



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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);                 //ICM20608G
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_15;             
	GPIO_Init(GPIOB, &GPIO_InitStructure);   
	
	/* Deselect : Chip Select high ---------*/
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
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
	GPIO_ResetBits(GPIOx,GPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET){}		//等待发送区空  
	
	SPI_I2S_SendData(SPI, address); 																		//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	SPI_I2S_ReceiveData(SPI); 																	        //返回通过SPIx最近接收的数据
  
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET){}		//等待发送区空  
	
	SPI_I2S_SendData(SPI, value); 																		//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	SPI_I2S_ReceiveData(SPI); 		
		
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
	
	GPIO_ResetBits(GPIOx,GPIO_Pin);
	/*
	一个完整的传送周期是16位，即两个字节，
	因为，首先主机要发送命令过去，然后从机根据主机的命令准备数据，
	主机在下一个8位时钟周期才把数据读回来。
	*/
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPIx, address); 																		//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	SPI_I2S_ReceiveData(SPIx); 																	        //返回通过SPIx最近接收的数据
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPIx, DUMMY_BYTE); 																	//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	data = SPI_I2S_ReceiveData(SPIx); 		
		
	GPIO_SetBits(GPIOx,GPIO_Pin);
		
	return data;
}




	#ifdef SINGLESYSTEM
uint16_t SPI_ReadAS5045(uint8_t num)
{
	uint8_t  buf[3],i;
	uint32_t AS5045_Val;
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	
	Delay_us(1);  
	SPI_Cmd(SPI1,ENABLE);
	for(i=0;i<3;i++)
	{
		while((SPI1->SR & SPI_I2S_FLAG_RXNE) == (u16)RESET);
		buf[i] = SPI1->DR;
	}
	GPIO_SetBits(GPIOA,GPIO_Pin_4);

	SPI_Cmd(SPI1,DISABLE);
  AS5045_Val = (((uint32_t)buf[0]<<16) | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]));
	
	Delay_us(50);        
	
	return (AS5045_Val>>12) & 0xffff;
}

uint8_t SPI2_ReadWriteByte(uint8_t TxData)
{		 			 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}      	
	SPI_I2S_SendData(SPI2, TxData);                                     
		
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){}      
	return SPI_I2S_ReceiveData(SPI2);                                     
}

void mRead(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
	/*
	SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). 
	The first byte contains the SPI Address, and the following byte(s) contain(s) the SPI data. 
	The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation. 
	*/
  ReadAddr |= (uint8_t)READWRITE_CMD;
  /* Set chip select Low at the start of the transmission */
  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
  /* Send the Address of the indexed register */
  SPI2_ReadWriteByte(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
		/*
		pBuffer从低地址到高地址依次是
		X高八位,X第八位,Y高八位,Y第八位,Z高八位,Z第八位
		*/
    *pBuffer = SPI2_ReadWriteByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
  /* Set chip select High at the end of the transmission */ 
  GPIO_SetBits(GPIOB,GPIO_Pin_12);
}
  #else
uint16_t SPI_ReadAS5045(uint8_t num)
{
	uint8_t  buf[3],i;
	uint32_t AS5045_Val;
	
	if(num==0)
	  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	else if(num==1)
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
	
	Delay_us(1); 
	SPI_Cmd(SPI2,ENABLE);
	for(i=0;i<3;i++)
	{
		while((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
		buf[i] = SPI2->DR;
	}
	if(num==0)
	  GPIO_SetBits(GPIOB,GPIO_Pin_12);
	else if(num==1)
		GPIO_SetBits(GPIOB,GPIO_Pin_15);

	SPI_Cmd(SPI2,DISABLE);
  AS5045_Val = (((uint32_t)buf[0]<<16) | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]));
	
	Delay_us(50); 
	
	return (AS5045_Val>>12) & 0xffff;
}
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{		 			 
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}      	
	SPI_I2S_SendData(SPI1, TxData);                                     
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}      
	return SPI_I2S_ReceiveData(SPI1);                                     
}

void mRead(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
	/*
	SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). 
	The first byte contains the SPI Address, and the following byte(s) contain(s) the SPI data. 
	The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation. 
	*/
  ReadAddr |= (uint8_t)READWRITE_CMD;
  /* Set chip select Low at the start of the transmission */
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);
  
  /* Send the Address of the indexed register */
  SPI1_ReadWriteByte(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
    *pBuffer = SPI1_ReadWriteByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  GPIO_SetBits(GPIOA,GPIO_Pin_4);
}
	#endif


