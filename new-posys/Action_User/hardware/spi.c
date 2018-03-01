#include "spi.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "config.h"

void ICM_SPIInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  
  /* ʹ��SPI��GPIO��ʱ��------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* ���Ÿ���ΪSPI1----------------------------------------------*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  
  /* Ӳ��SPI1��������---------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* SPI����------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				/* ˫��˫��ȫ˫��									*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* ��SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													/* SPI����8λ֡�ṹ								*/
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																/* ����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ	*/
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;															/* ��һ�����������ݱ�����					*/
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS���������									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;				/* Ԥ��Ƶ  ���������10MHz													*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* ���ݴ�MSBλ��ʼ								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  SPI_Cmd(SPI1, ENABLE);
}


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
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS���������									*/
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;				/* Ԥ��Ƶ	168M/64	*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* ���ݴ�MSBλ��ʼ								*/
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  SPI_Cmd(SPI2, ENABLE);
}


/**
* @brief  SPI1�Ĳ�����Ԥ��Ƶ����
*
* @param[in] ������Ԥ��Ƶֵ
* @retval None
*/
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
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
  SPI_Cmd(SPI,ENABLE);	
  GPIO_ResetBits(GPIOx,GPIO_Pin);
  
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET){}		//�ȴ���������  
  
  SPI_I2S_SendData(SPI, address); 																		//ͨ������SPIx����һ��byte  ����
  
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
  
  SPI_I2S_ReceiveData(SPI); 																	        //����ͨ��SPIx������յ�����
  
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET){}		//�ȴ���������  
  
  SPI_I2S_SendData(SPI, value); 																		//ͨ������SPIx����һ��byte  ����
  
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
  
  SPI_I2S_ReceiveData(SPI); 		
  
  while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_BSY) == SET){}	
  SPI_Cmd(SPI,DISABLE);		
  
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
  SPI_Cmd(SPIx,ENABLE);
  
  GPIO_ResetBits(GPIOx,GPIO_Pin);
  
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
  
  SPI_I2S_SendData(SPIx, address);	 																		//ͨ������SPIx����һ��byte  ����
  
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
  
  SPI_I2S_ReceiveData(SPIx);																		        //����ͨ��SPIx������յ�����
  
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
  
  SPI_I2S_SendData(SPIx, DUMMY_BYTE); 																	//ͨ������SPIx����һ��byte  ����
  
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
  
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
  /*
  SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). 
  The first byte contains the SPI Address, and the following byte(s) contain(s) the SPI data. 
  The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation. 
  */
  address |= (uint8_t)READWRITE_CMD;
  
  SPI_Cmd(SPIx,ENABLE);	
  GPIO_ResetBits(GPIOx,GPIO_Pin);
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
  
  SPI_I2S_SendData(SPIx, address); 																		//ͨ������SPIx����һ��byte  ����
  
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
  
  SPI_I2S_ReceiveData(SPIx); 																	        //����ͨ��SPIx������յ�����
  
  /* Receive the data that will be read from the device (MSB First) */
  for(uint32_t i=0;i<len;i++)
  {
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //�ȴ���������  
    
    SPI_I2S_SendData(SPIx, DUMMY_BYTE); 																	//ͨ������SPIx����һ��byte  ����
    
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //�ȴ�������һ��byte  
    
    data[i]=SPI_I2S_ReceiveData(SPIx); 																	        //����ͨ��SPIx������յ�����
  }
  
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){}	
  SPI_Cmd(SPIx,DISABLE);		
  GPIO_SetBits(GPIOx,GPIO_Pin);	
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
		while((SPI2->SR & SPI_I2S_FLAG_RXNE) == (u16)RESET);
		buf[i] = SPI2->DR;
	}
	if(num==1)
	  GPIO_SetBits(GPIOB,GPIO_Pin_12);
	else if(num==0)
		GPIO_SetBits(GPIOB,GPIO_Pin_15);

	SPI_Cmd(SPI2,DISABLE);
  AS5045_Val = (((uint32_t)buf[0]<<16) | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]));
	
	return AS5045_Val;
}

uint16_t SPI_ReadAS5045_Parity(uint8_t num)
{
	uint32_t  AbsEncData  = SPI_ReadAS5045All(num); //SPI�����ı�����������
	/*����������*/
	int16_t   tmpAbs      = (AbsEncData >> 12) & 0X0FFF;
	/*����������żУ��λ*/
	uint8_t evebParity = (AbsEncData >> 6) & 0x01;
	/*��У����*/
	uint32_t parity = (AbsEncData>>7) & 0x1FFFF;
	/*����������żУ��λ*/
	uint8_t evebParityCal = 0;
	/*��ñ�־λ*/
	uint16_t MagSta = (AbsEncData >> 7) & 0x1F;
	static int count=0;
	while(1)
	{
		/*������żУ����*/
		while (parity)
		{
			evebParityCal =!evebParityCal;
			parity = parity & (parity - 1);
		}
		/*�����żУ��ɹ������ұ�־λ��ȷ*/
		if(evebParityCal==evebParity&&(MagSta==0x10||MagSta==0x13))
		{
			//����ѭ��
			count=0;
			break;
		}
		//���¶�ȡ
		else
		{
			count++;
			/*�ٴλ�ȡ24λ����*/
			AbsEncData  = SPI_ReadAS5045All(num);
			/*ȡ������λ*/
			tmpAbs      = (AbsEncData >> 12) & 0X0FFF;
			/*��ø�������żУ��λ*/
			evebParity = (AbsEncData >> 6) & 0x01;
			/*��ñ�У����*/
			parity = (AbsEncData>>7) & 0x1FFFF;
			/*��ñ�־λ*/
			MagSta = (AbsEncData >> 7) & 0x1F;
			/*��ʼ����żУ����*/
			evebParityCal = 0;
			/*��ֹ��ζ�������*/
			if(count>5)
			{
				count=0;
				break;
			}
		}
	}
	return tmpAbs;
}

#define READ_NUM	3
/*�ҵ�������С��ֵ*/
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
	/*�����洢��*/
	uint16_t value[READ_NUM]={0};
	/*����һ��ֵ�Ĳ�ֵ*/
	int delValue[READ_NUM]={0};
	/*���ս��*/
	uint16_t endValue=0;
	static uint16_t endValueLast[2]={0,0};
	
	/*����������*/
	for(int i=0;i<READ_NUM;i++)
		value[i]=SPI_ReadAS5045_Parity(num);
	
	/*��������һ�εĲ�ֵ*/
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
	
	/*�ҵ�����һ�β�ֵ��С��ֵ�����кţ�������*/
	endValue= value[FindMin2(delValue)];
	
	if(num==0)
		endValueLast[0]=endValue;
	else if(num==1)
		endValueLast[1]=endValue;
	return endValue;
	
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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
  
  /* ����Ƭѡ����------------------------- */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);                 //ICM20608G
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;             
  GPIO_Init(GPIOC, &GPIO_InitStructure);   
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_15;             
  GPIO_Init(GPIOB, &GPIO_InitStructure);   
  
  /* Deselect : Chip Select high ---------*/
  GPIO_SetBits(GPIOA, GPIO_Pin_1);
  GPIO_SetBits(GPIOA, GPIO_Pin_2);
  GPIO_SetBits(GPIOC, GPIO_Pin_6);
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
  GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

