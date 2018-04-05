#include "spi.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
/*
������ϸ��鿴�ο��ֲ�
*/
void SPI1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	/* ʹ��SPI��GPIOB��ʱ��------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//84MHz
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* ���Ÿ���ΪSPI2----------------------------------------------*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	
	/* Ӳ��SPI1��������---------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* SPI����------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;				    /* ˫��˫��ȫ˫��									*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* ��SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI����8λ֡�ṹ								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;															/* ����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* �ڶ������������ݱ�����					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS����������									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;				/* Ԥ��Ƶ	84/128	*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* ���ݴ�MSBλ��ʼ								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_Cmd(SPI1, ENABLE);

}

/*
������ϸ��鿴�ο��ֲ�
*/
void SPI2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	/* ʹ��SPI��GPIOB��ʱ��------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* ���Ÿ���ΪSPI2----------------------------------------------*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	
	/* Ӳ��SPI1��������---------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI����------------------------------------------------------*/
  SPI_I2S_DeInit(SPI2);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;				    /* ˫��˫��ȫ˫��									*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* ��SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI����8λ֡�ṹ								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																/* ����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* �ڶ������������ݱ�����					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS����������									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;				/* Ԥ��Ƶ	168M/64	*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* ���ݴ�MSBλ��ʼ								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  SPI_Cmd(SPI2, ENABLE);

}

void SPI3_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	/* ʹ��SPI��GPIO��ʱ��------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* ���Ÿ���ΪSPI1----------------------------------------------*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
	
	/* Ӳ��SPI1��������---------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* SPI����------------------------------------------------------*/
  SPI_I2S_DeInit(SPI3);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				/* ˫��˫��ȫ˫��									*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* ��SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI����8λ֡�ṹ								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																/* ����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;															/* ��һ�����������ݱ�����					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS����������									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;				/* Ԥ��Ƶ													*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* ���ݴ�MSBλ��ʼ								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
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

	/* ����Ƭѡ����------------------------- */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);                 //ICM20608G
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;             
	GPIO_Init(GPIOB, &GPIO_InitStructure);   
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;             
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	/* Deselect : Chip Select high ---------*/
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

/**
  * @brief  SPI1�Ĳ�����Ԥ��Ƶ����
	*
  * @param[in] ������Ԥ��Ƶֵ
  * @retval None
  */
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));					/* �ж���Ч��												*/
	SPI1->CR1 &= 0XFFC7;																										/* λ3-5���㣬�������ò�����*/
	SPI1->CR1 |= SPI_BaudRatePrescaler;																			
	SPI_Cmd(SPI1,ENABLE); 																									
} 

/**
  * @brief     ��SPI�ӻ��豸д������
  * @param     SPI: SPIx
  * @param     GPIOx,GPIO_Pin: Ƭѡ����
  * @param     address: Ҫд��üĴ����ĵ�ַ
  * @param     value��Ҫд���ֵ
  * @retval    None
  */
void SPI_Write(SPI_TypeDef *SPI,
	            GPIO_TypeDef *GPIOx,
	                uint16_t GPIO_Pin,
                   uint8_t address,
                   uint8_t value)
{
	GPIO_ResetBits(GPIOx,GPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET){}		//�ȴ���������  
	
	SPI_I2S_SendData(SPI, address); 																		//ͨ������SPIx����һ��byte  ����
		
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
 
	SPI_I2S_ReceiveData(SPI); 																	        //����ͨ��SPIx������յ�����
  
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET){}		//�ȴ���������  
	
	SPI_I2S_SendData(SPI, value); 																		//ͨ������SPIx����һ��byte  ����
		
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
 
	SPI_I2S_ReceiveData(SPI); 		
		
	GPIO_SetBits(GPIOx,GPIO_Pin);	
}




/**
  * @brief     ��SPI�ӻ��豸��ȡ����
  * @param     SPI: SPIx
  * @param     GPIOx,GPIO_Pin: Ƭѡ����
  * @param     address: Ҫ��ȡ�ļĴ����ĵ�ַ
  * @retval    value���Ĵ�����ֵ
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
	һ�������Ĵ���������16λ���������ֽڣ�
	��Ϊ����������Ҫ���������ȥ��Ȼ��ӻ���������������׼�����ݣ�
	��������һ��8λʱ�����ڲŰ����ݶ�������
	*/
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
	
	SPI_I2S_SendData(SPIx, address); 																		//ͨ������SPIx����һ��byte  ����
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
 
	SPI_I2S_ReceiveData(SPIx); 																	        //����ͨ��SPIx������յ�����
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
	
	SPI_I2S_SendData(SPIx, DUMMY_BYTE); 																	//ͨ������SPIx����һ��byte  ����
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
 
	data = SPI_I2S_ReceiveData(SPIx); 		
		
	GPIO_SetBits(GPIOx,GPIO_Pin);
		
	return data;
}


u8 SPI2_ReadWriteByte(u8 TxData)
{		 			 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}      	
	SPI_I2S_SendData(SPI2, TxData);                                     
		
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){}      
	return SPI_I2S_ReceiveData(SPI2);                                     
}
#ifdef SINGLE
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
#else 
uint16_t SPI_ReadAS5045(uint8_t num)
{
	uint8_t  buf[3],i;
	uint32_t AS5045_Val;
	
	if(num==1)
	  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	else if(num==0)
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
	
	Delay_us(1);
	SPI_Cmd(SPI2,ENABLE);
	for(i=0;i<3;i++)
	{
		while((SPI2->SR & SPI_I2S_FLAG_RXNE) == (u16)RESET);
		buf[i] = SPI2->DR;
	}
	if(num==1)
	  GPIO_SetBits(GPIOB,GPIO_Pin_12);
	else if(num==0)
		GPIO_SetBits(GPIOB,GPIO_Pin_15);

	SPI_Cmd(SPI2,DISABLE);
  AS5045_Val = (((uint32_t)buf[0]<<16) | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]));
	
	Delay_us(50);
	
	return (AS5045_Val>>12) & 0xffff;
}
#endif
void mRead(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
	/*
	SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). 
	The first byte contains the SPI Address, and the following byte(s) contain(s) the SPI data. 
	The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation. 
	*///??MULTIPLEBYTE_CMD��û������
//  if(NumByteToRead > 0x01){
//		/*
//		#define READWRITE_CMD              ((uint8_t)0x80) �ߵڰ�λ
//		#define MULTIPLEBYTE_CMD           ((uint8_t)0x40) �ߵ���λ
//		*/
//    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
//  }
//  else{
    ReadAddr |= (uint8_t)READWRITE_CMD;
 // }
  /* Set chip select Low at the start of the transmission */
  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
  
  /* Send the Address of the indexed register */
  SPI2_ReadWriteByte(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
		/*
		pBuffer�ӵ͵�ַ���ߵ�ַ������
		X�߰�λ,X�ڰ�λ,Y�߰�λ,Y�ڰ�λ,Z�߰�λ,Z�ڰ�λ
		*/
    *pBuffer = SPI2_ReadWriteByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  GPIO_SetBits(GPIOB,GPIO_Pin_12);
}