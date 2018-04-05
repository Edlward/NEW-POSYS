/**
  ******************************************************************************
  * @file    ADXRS453.cpp
  * @author  Luo Xiaoyi 
  * @version V1.0
  * @date    2017.3.13
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "ADXRS453.h"
#include "stm32f4xx_spi.h"
#include "spi.h"
#include "timer.h"
#include "usart.h"
#include "signalProcess.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static deviceADXRS453 ADXRS453(SPI1,GPIOA,GPIO_Pin_4);
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
static void  changeSPIconfigure(SPI_TypeDef* SPIx)
{
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_Cmd(SPIx,DISABLE);
	SPI_I2S_DeInit(SPIx);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				/* 双线双向全双工									*/
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															/* 主SPI													*/
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;											    /* SPI接收8位帧结构								*/
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																/* 串行同步时钟的空闲状态为低电平	*/
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;															/* 第一个跳变沿数据被采样					*/
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;				/* 预分频													*/
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPIx, &SPI_InitStructure);
	SPI_Cmd(SPIx, ENABLE);
}
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
deviceADXRS453& getADXRS453(void)
{
	return ADXRS453;
}
/* Class functions ------------------------------------------------------------*/
uint16_t deviceADXRS453::rawDataRead(uint8_t address)
{
	changeSPIconfigure(SPIx);
	return SPI16_Read(SPIx,csGPIOx,csGPIO_Pin,address);
}
void 	deviceADXRS453::rawDataWrite(uint8_t address,uint16_t value)
{
	changeSPIconfigure(SPIx);
	SPI16_Write(SPIx,csGPIOx,csGPIO_Pin,address,value);
}
void 	deviceADXRS453::init(void)
{
	uint16_t data=0x00;
	
	changeSPIconfigure(SPIx);
	GPIO_ResetBits(csGPIOx,csGPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPIx, 0X2000); 																		//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	data=SPI_I2S_ReceiveData(SPIx);	
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPIx, 0x0003); 																	//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	SPI_I2S_ReceiveData(SPIx);	
		
	GPIO_SetBits(csGPIOx,csGPIO_Pin);
	
	delay_ms(50);//第二个命令
	
	GPIO_ResetBits(csGPIOx,csGPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPIx, 0X2000); 																		//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
  data=SPI_I2S_ReceiveData(SPIx);       
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPIx, 0x0000); 																//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	data=SPI_I2S_ReceiveData(SPIx);		
		
	GPIO_SetBits(csGPIOx,csGPIO_Pin);
		
	delay_ms(50);//第三个命令
	
	GPIO_ResetBits(csGPIOx,csGPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPIx, 0X2000); 																		//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
  SPI_I2S_ReceiveData(SPIx);	        
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPIx, 0x0000); 																	//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
  
	data=SPI_I2S_ReceiveData(SPIx);
		
	GPIO_SetBits(csGPIOx,csGPIO_Pin);
	
	delay_ms(50);//第四个命令
		
	GPIO_ResetBits(csGPIOx,csGPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPIx, 0X2000); 																		//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	data=SPI_I2S_ReceiveData(SPIx);        
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPIx, 0x0000); 																	//通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	data=SPI_I2S_ReceiveData(SPIx);		
		
	GPIO_SetBits(csGPIOx,csGPIO_Pin);
		
	//rawDataRead(0x00);	
}
void     		deviceADXRS453::updateData(void)
{
	static float rateList[ADXRS453_UPDATE_FREQ]={0,0,0,0,0};
	static float tempList[ADXRS453_UPDATE_FREQ]={0,0,0,0,0};
	
	shiftRightData(rateList,ADXRS453_UPDATE_FREQ);
	shiftRightData(tempList,ADXRS453_UPDATE_FREQ);
	
	rawDataRead(0x00);
	rateList[0]=static_cast<int16_t>(rawDataRead(0x02))/80.0f;
	tempList[0]=static_cast<int16_t>(rawDataRead(0x0C))/64;
	static uint32_t countOut=0;
	countOut++;
	
	if(countOut>=ADXRS453_UPDATE_FREQ)
	{
		countOut=0;
		val=meanData(rateList,ADXRS453_UPDATE_FREQ);
		temp=meanData(tempList,ADXRS453_UPDATE_FREQ);
		//cout<<endl;
	}

	//cout<<rateList[0]<<'\t'<<(int)tempList[0]<<endl;
	
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
