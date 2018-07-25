#include "spi.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"


/**
  * @brief  Ӳ��SPI1��ʼ��
	*
  * @param  None
  * @retval None
  */
void SPI1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;			
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;											
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;														
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;													
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;															
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;			
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;											
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_Cmd(SPI1, ENABLE);
}

void SPI2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 /*| GPIO_Pin_15*/;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  SPI_I2S_DeInit(SPI2);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;							
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;														
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;									
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;											
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;									
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;		
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;			
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  SPI_Cmd(SPI2, DISABLE);
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
  * @brief  SPI��Ƭѡ��������
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
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
  GPIO_SetBits(GPIOA, GPIO_Pin_15);
}

        
void SPI_Write(SPI_TypeDef *SPI,
	            GPIO_TypeDef *GPIOx,
	                uint16_t GPIO_Pin,
                   uint8_t address,
                   uint8_t value)
{
	SPI_Cmd(SPI,ENABLE);	
	GPIO_ResetBits(GPIOx,GPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET){}		  
	
	SPI_I2S_SendData(SPI, address); 																		
		
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET){} 	  
 
	SPI_I2S_ReceiveData(SPI); 																	        
  
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET){}	
	
	SPI_I2S_SendData(SPI, value); 																		
		
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET){} 	 
 
	SPI_I2S_ReceiveData(SPI); 		
		
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_BSY) == SET){}	
	SPI_Cmd(SPI,DISABLE);		
		
	Delay_us(1);//min tcs.hd 63ns
	GPIO_SetBits(GPIOx,GPIO_Pin);	
	Delay_us(1);//tsdo.dis 20ns
}


uint8_t SPI_Read(SPI_TypeDef *SPIx,
								 GPIO_TypeDef* GPIOx,
	               uint16_t GPIO_Pin,
                 uint8_t address)
{
	
	uint8_t data;
	
  address |= (uint8_t)READWRITE_CMD;
	SPI_Cmd(SPIx,ENABLE);	
	
	GPIO_ResetBits(GPIOx,GPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  
	
	SPI_I2S_SendData(SPIx, address); 																		
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	 
 
	SPI_I2S_ReceiveData(SPIx); 																	     
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		 
	
	SPI_I2S_SendData(SPIx, DUMMY_BYTE); 															
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  
 
	data = SPI_I2S_ReceiveData(SPIx); 		
		
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){}	
	SPI_Cmd(SPIx,DISABLE);		
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
	/*SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). 
	The first byte contains the SPI Address, and the following byte(s) contain(s) the SPI data. 
	The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation. */
	address |= (uint8_t)READWRITE_CMD;
	
	SPI_Cmd(SPIx,ENABLE);	
	GPIO_ResetBits(GPIOx,GPIO_Pin);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  
	
	SPI_I2S_SendData(SPIx, address); 																	

	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	 
 
	SPI_I2S_ReceiveData(SPIx); 																	 

	/* Receive the data that will be read from the device (MSB First) */
	for(uint32_t i=0;i<len;i++)
	{
		while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		 
	
		SPI_I2S_SendData(SPIx, DUMMY_BYTE); 																

		while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	 
 
		data[i]=SPI_I2S_ReceiveData(SPIx); 																	
	}
		
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){}	
	SPI_Cmd(SPIx,DISABLE);		
	GPIO_SetBits(GPIOx,GPIO_Pin);	
}


uint16_t SPI16_Read(SPI_TypeDef *SPIx,
													 GPIO_TypeDef* GPIOx,
													 uint16_t GPIO_Pin,
													 uint8_t address)
{
	
	uint16_t data=0x0000;
  uint16_t data2=0x0000;
	uint16_t checkbit=0x0000;
	
	uint16_t cmd_mo=0x0000;
  
	cmd_mo=0x8000|((uint16_t)address<<1);
	
	for(uint8_t i=0;i<16;i++)
	{
		if((cmd_mo>>i)&0x0001)
			checkbit++;
	}
	checkbit=(uint16_t)(checkbit%2!=1);
	SPI_Cmd(SPIx,ENABLE);		
	GPIO_ResetBits(GPIOx,GPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
	
	SPI_I2S_SendData(SPIx, cmd_mo); 																		//ͨ������SPIx����һ��byte  ����
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
 
	data =SPI_I2S_ReceiveData(SPIx); 																	        //����ͨ��SPIx������յ�����
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
	
	SPI_I2S_SendData(SPIx, checkbit); 																	//ͨ������SPIx����һ��byte  ����
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
 
	data2 = SPI_I2S_ReceiveData(SPIx); 		
		
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){}		
	SPI_Cmd(SPIx,DISABLE);		
	GPIO_SetBits(GPIOx,GPIO_Pin);
	
	data=(data<<11)|(data2>>5);	
		
	return data;
}
 uint16_t SPI16_Write(SPI_TypeDef *SPIx,
									   GPIO_TypeDef* GPIOx,
									   uint16_t GPIO_Pin,
									   uint8_t address,
                     uint16_t sdata)
{
	
	uint16_t data=0x0000;
  uint16_t data2=0x0000;
	uint16_t checkbit=0x0000;
	
	uint16_t cmd_mo=0x0000;
  
	cmd_mo=0x4000|((uint16_t)address<<1);
	
	for(uint8_t i=0;i<16;i++)
	{
		if((cmd_mo>>i)&0x0001)
			checkbit++;
	}
	for(uint8_t i=0;i<16;i++)
	{
		if((sdata>>i)&0x0001)
			checkbit++;
	}
	checkbit=(uint16_t)(checkbit%2!=1);
	SPI_Cmd(SPIx,ENABLE);	
	
	GPIO_ResetBits(GPIOx,GPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
	
	SPI_I2S_SendData(SPIx, cmd_mo|(sdata>>15)); 																		//ͨ������SPIx����һ��byte  ����
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
 
	data =SPI_I2S_ReceiveData(SPIx); 																	        //����ͨ��SPIx������յ�����
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
	
	SPI_I2S_SendData(SPIx, ((uint16_t)checkbit)|((uint16_t)(sdata<<1))); 																	//ͨ������SPIx����һ��byte  ����
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
 
	data2 = SPI_I2S_ReceiveData(SPIx); 		
	
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){}	
	SPI_Cmd(SPIx,DISABLE);		
	GPIO_SetBits(GPIOx,GPIO_Pin);
			
	data=(data<<11)|(data2>>5);	
		
	return data;
}
void SPI_HalfDuplex_Write(SPI_TypeDef *SPIx,
												  GPIO_TypeDef* GPIOx,
												  uint16_t GPIO_Pin,
											  	uint8_t address,
												  uint16_t sdata)
{
	
	SPI_BiDirectionalLineConfig(SPIx,SPI_Direction_Tx);
	SPI_Cmd(SPIx,ENABLE);
	
	GPIO_ResetBits(GPIOx,GPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
	
	SPI_I2S_SendData(SPIx, address); 																		//ͨ������SPIx����һ��byte  ����
		 
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
	
	SPI_I2S_SendData(SPIx, sdata); 																	//ͨ������SPIx����һ��byte  ����
		
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){}
	SPI_Cmd(SPIx,DISABLE);	
	GPIO_SetBits(GPIOx,GPIO_Pin);
	
}

uint8_t SPI_HalfDuplex_Read(SPI_TypeDef *SPIx,
														GPIO_TypeDef* GPIOx,
														uint16_t GPIO_Pin,
														uint8_t address)
{
	uint8_t data;
  address |= (uint8_t)READWRITE_CMD;
	delay_us(10);
  SPI_Cmd(SPIx,DISABLE);	
	SPI_BiDirectionalLineConfig(SPIx,SPI_Direction_Tx);	
	SPI_Cmd(SPIx,ENABLE);
	
	GPIO_ResetBits(GPIOx,GPIO_Pin);

	SPIx->DR=address;
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}

  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){}
		
		
	SPI_Cmd(SPIx,DISABLE);
	SPI_BiDirectionalLineConfig(SPIx,SPI_Direction_Rx);
	SPI_Cmd(SPIx,ENABLE);			
		
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){}
	data = SPI_I2S_ReceiveData(SPIx);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){}
	SPI_Cmd(SPIx,DISABLE);
	GPIO_SetBits(GPIOx,GPIO_Pin);
	delay_us(10);
	return data;
}
void SPI_OnlyRead(SPI_TypeDef *SPIx,
												 GPIO_TypeDef* GPIOx,
												 uint16_t GPIO_Pin,
												 uint8_t* data,
												 uint8_t len)
{
	GPIO_ResetBits(GPIOx,GPIO_Pin);
	SPI_Cmd(SPIx,ENABLE);			
	for(uint8_t i=0;i<len;i++)
	{
		while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){}
		data[i] = SPI_I2S_ReceiveData(SPIx);
	}
	//while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){}
	SPI_Cmd(SPIx,DISABLE);
	GPIO_SetBits(GPIOx,GPIO_Pin);
	delay_us(10);
}




