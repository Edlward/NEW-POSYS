#include "spi.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "stdlib.h"



/**
  * @brief  ”≤º˛SPI1≥ı ºªØ
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


/**
  * @brief  SPI3≥ı ºªØ
  * @retval None
  */
void SPI3_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	/*  πƒ‹SPI∫ÕGPIOµƒ ±÷”------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* “˝Ω≈∏¥”√Œ™SPI1----------------------------------------------*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
  //GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);  //∞ÎÀ´π§ƒ£ Ω ÷˜ª˙ œ¬£¨SPI÷ª”√CLK ∫Õ MOSI 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
	
	/* ”≤º˛SPI1“˝Ω≈≈‰÷√---------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 /*| GPIO_Pin_11*/ | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* SPI≈‰÷√------------------------------------------------------*/
  SPI_I2S_DeInit(SPI3);
  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;									/* À´œﬂÀ´œÚ»´À´π§									*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* ÷˜SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPIΩ” ’8Œª÷°Ω·ππ								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;																/* ¥Æ––Õ¨≤Ω ±÷”µƒø’œ–◊¥Ã¨Œ™µÕµÁ∆Ω	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* µ⁄“ª∏ˆÃ¯±‰—ÿ ˝æ›±ª≤…—˘					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS”…»Ìº˛øÿ÷∆									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;				/* ‘§∑÷∆µ													*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/*  ˝æ›¥”MSBŒªø™ º								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI3, &SPI_InitStructure);

  SPI_Cmd(SPI3, ENABLE);
}			


/**
  * @brief  SPIµƒ∆¨—°“˝Ω≈≈‰÷√
	*
  * @param  None
  * @retval None
  */
void CS_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
  
  /* ÈÖçÁΩÆÁâáÈÄâÂºïËÑö------------------------- */
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
  * @brief     œÚSPI¥”ª˙…Ë±∏–¥»Î ˝æ›
  * @param     SPI: SPIx
  * @param     GPIOx,GPIO_Pin: ∆¨—°“˝Ω≈
  * @param     address: “™–¥»Îµ√ºƒ¥Ê∆˜µƒµÿ÷∑
  * @param     value£∫“™–¥»Îµƒ÷µ
  * @retval    None
  */                            
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




/**
  * @brief     œÚSPI¥”ª˙…Ë±∏∂¡»° ˝æ›
  * @param     SPI: SPIx
  * @param     GPIOx,GPIO_Pin: ∆¨—°“˝Ω≈
  * @param     address: “™∂¡»°µƒºƒ¥Ê∆˜µƒµÿ÷∑
  * @retval    value£∫ºƒ¥Ê∆˜µƒ÷µ
  */
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
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //µ»¥˝∑¢ÀÕ«¯ø’  
	
	SPI_I2S_SendData(SPIx, cmd_mo); 																		//Õ®π˝Õ‚…ËSPIx∑¢ÀÕ“ª∏ˆbyte   ˝æ›
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //µ»¥˝Ω” ’ÕÍ“ª∏ˆbyte  
 
	data =SPI_I2S_ReceiveData(SPIx); 																	        //∑µªÿÕ®π˝SPIx◊ÓΩ¸Ω” ’µƒ ˝æ›
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //µ»¥˝∑¢ÀÕ«¯ø’  
	
	SPI_I2S_SendData(SPIx, checkbit); 																	//Õ®π˝Õ‚…ËSPIx∑¢ÀÕ“ª∏ˆbyte   ˝æ›
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //µ»¥˝Ω” ’ÕÍ“ª∏ˆbyte  
 
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
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //µ»¥˝∑¢ÀÕ«¯ø’  
	
	SPI_I2S_SendData(SPIx, cmd_mo|(sdata>>15)); 																		//Õ®π˝Õ‚…ËSPIx∑¢ÀÕ“ª∏ˆbyte   ˝æ›
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //µ»¥˝Ω” ’ÕÍ“ª∏ˆbyte  
 
	data =SPI_I2S_ReceiveData(SPIx); 																	        //∑µªÿÕ®π˝SPIx◊ÓΩ¸Ω” ’µƒ ˝æ›
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //µ»¥˝∑¢ÀÕ«¯ø’  
	
	SPI_I2S_SendData(SPIx, ((uint16_t)checkbit)|((uint16_t)(sdata<<1))); 																	//Õ®π˝Õ‚…ËSPIx∑¢ÀÕ“ª∏ˆbyte   ˝æ›
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //µ»¥˝Ω” ’ÕÍ“ª∏ˆbyte  
 
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
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //µ»¥˝∑¢ÀÕ«¯ø’  
	
	SPI_I2S_SendData(SPIx, address); 																		//Õ®π˝Õ‚…ËSPIx∑¢ÀÕ“ª∏ˆbyte   ˝æ›
		 
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //µ»¥˝∑¢ÀÕ«¯ø’  
	
	SPI_I2S_SendData(SPIx, sdata); 																	//Õ®π˝Õ‚…ËSPIx∑¢ÀÕ“ª∏ˆbyte   ˝æ›
		
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



uint32_t SPI_ReadAS5045All(uint8_t num)
{
	uint8_t  buf[3]={0,0,0},i=0;
	uint32_t AS5045_Val=0;
	
	if(num==1)
	  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	else if(num==0)
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);

	SPI_Cmd(SPI2,ENABLE);
	
	for(i=0;i<3;i++)
	{
		while((SPI2->SR & SPI_I2S_FLAG_RXNE) == (unsigned short)RESET){};
			buf[i] = SPI2->DR;
	}
	
	if(num==1)
	  GPIO_SetBits(GPIOB,GPIO_Pin_12);
	else if(num==0)
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
	
	SPI_Cmd(SPI2,DISABLE);
	
	delay_us(15);
	
  AS5045_Val = (((uint32_t)buf[0]<<16) | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]));
	
	return AS5045_Val;
}

uint16_t SPI_ReadAS5045_Parity(uint8_t num)
{
	uint32_t  AbsEncData  = SPI_ReadAS5045All(num); //SPI∂¡µΩµƒ±‡¬Î∆˜µƒ ˝æ›
	/*±‡¬Î∆˜ ˝æ›*/
	/*±‡¬Î∆˜ ˝æ›*/
	int16_t   tmpAbs      = (AbsEncData >> 12) & 0X0FFF;
	/*∑¢ÀÕ¿¥µƒ∆Ê≈º–£—ÈŒª*/
	uint8_t evebParity = (AbsEncData >> 6) & 0x01;
	/*±ª–£—È ˝*/
	uint32_t parity = (AbsEncData>>7) & 0x1FFFF;
	/*º∆À„¿¥µƒ∆Ê≈º–£—ÈŒª*/
	uint8_t evebParityCal = 0;
	/*ªÒµ√±Í÷æŒª*/
	uint16_t MagSta = (AbsEncData >> 7) & 0x1F;
	static int count=0;
	while(1)
	{
		/*º∆À„∆Ê≈º–£—ÈΩ·π˚*/
		while (parity)
		{
			evebParityCal =!evebParityCal;
			parity = parity & (parity - 1);
		}
		/*»Áπ˚∆Ê≈º–£—È≥…π¶£¨≤¢«“±Í÷æŒª’˝»∑*/
		if(evebParityCal==evebParity&&(MagSta==0x10||MagSta==0x13))
		{
			//Ã¯≥ˆ—≠ª∑
			count=0;
			break;
		}
		//÷ÿ–¬∂¡»°
		else
		{
			count++;
			/*‘Ÿ¥ŒªÒ»°24Œª ˝æ›*/
			AbsEncData  = SPI_ReadAS5045All(num);
			/*»°≥ˆ ˝æ›Œª*/
			tmpAbs      = (AbsEncData >> 12) & 0X0FFF;
			/*ªÒµ√∏¯≥ˆµƒ∆Ê≈º–£—ÈŒª*/
			evebParity = (AbsEncData >> 6) & 0x01;
			/*ªÒµ√±ª–£—È ˝*/
			parity = (AbsEncData>>7) & 0x1FFFF;
			/*ªÒµ√±Í÷æŒª*/
			MagSta = (AbsEncData >> 7) & 0x1F;
			/*≥ı ºªØ∆Ê≈º–£—ÈΩ·π˚*/
			evebParityCal = 0;
			/*∑¿÷π∂‡¥Œ∂¡≤ª≥ˆ¿¥*/
			if(count>3)
			{
				count=0;
				break;
			}
		}
	}
	return tmpAbs;
}

#define READ_NUM	3
/*’“µΩ ˝◊È◊Ó–°µƒ÷µ*/
uint16_t FindMin2(int codes[READ_NUM])
{
	uint16_t Min=codes[0]; 
	uint16_t index = 0;
  for(int i=1;i<READ_NUM;i++)
	{
		if(codes[i]<Min) 
		{
			Min=codes[i];
			index=i;
		}
	}
	return index;
}


uint16_t SPI_ReadAS5045(uint8_t num)
{
	/*¡¨∂¡¥Ê¥¢«¯*/
	uint16_t value[READ_NUM]={0};
	/*”Î…œ“ªøÃ÷µµƒ≤Ó÷µ*/
	int delValue[READ_NUM]={0};
	/*◊Ó÷’Ω·π˚*/
	uint16_t endValue=0;
	static uint16_t endValueLast[2]={0,0};
	
	/*¡¨–¯∂¡»˝¥Œ*/
	for(int i=0;i<READ_NUM;i++)
		value[i]=SPI_ReadAS5045_Parity(num);
	
	/*»˝¥Œ”Î…œ“ª¥Œµƒ≤Ó÷µ*/
	for(int i=0;i<READ_NUM;i++)
	{
		if(num==0)
			delValue[i]=(value[i]-endValueLast[0]);
		else if(num==1)
			delValue[i]=(value[i]-endValueLast[1]);
		if(delValue[i]>2048)
			delValue[i]-=4096;
		if(delValue[i]<-2048)
			delValue[i]+=4096;
		delValue[i]=abs(delValue[i]);
	}
	
	/*’“µΩ”Î…œ“ª¥Œ≤Ó÷µ◊Ó–°µƒ÷µµƒ–Ú¡–∫≈£¨≤¢¥´»Î*/
	endValue= value[FindMin2(delValue)];
	
	if(num==0)
		endValueLast[0]=endValue;
	else if(num==1)
		endValueLast[1]=endValue;
	return endValue;
	
}


