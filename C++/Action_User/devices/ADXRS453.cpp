/**
  ******************************************************************************
  * @file    ADXRS453.cpp
  * @author  Luo Xiaoyi and Qiao Zhijian 
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
#ifdef ADXRS453_USED
static deviceADXRS453 ADXRS453(SPI1,GPIOA,GPIO_Pin_4);
#endif
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
//change SPI configures 
//this function can only be called at ADXRS453.cpp
static void  changeSPIconfigure(SPI_TypeDef* SPIx)
{
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_Cmd(SPIx,DISABLE);
	SPI_I2S_DeInit(SPIx);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;			
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;													
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;								
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;														
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;											
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																							
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;				
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;											
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPIx, &SPI_InitStructure);
	SPI_Cmd(SPIx, ENABLE);
}
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
deviceADXRS453& getADXRS453(void)
{
	#ifdef ADXRS453_USED
		return ADXRS453;
	#else
		deviceADXRS453* b;
		return (deviceADXRS453&)b;
	#endif
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
	//change SPI configures according to the reference mannual
	changeSPIconfigure(SPIx);
	
	//reset CS to initiate the transmission
	GPIO_ResetBits(csGPIOx,csGPIO_Pin);
	
	/*configure in a way that the mannual indicates*/
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		  
	
	SPI_I2S_SendData(SPIx, 0X2000); 																	
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 
 
	SPI_I2S_ReceiveData(SPIx);	
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}	
	
	SPI_I2S_SendData(SPIx, 0x0003); 															
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 
 
	SPI_I2S_ReceiveData(SPIx);	
		
	GPIO_SetBits(csGPIOx,csGPIO_Pin);
	
	delay_ms(50);
	
	GPIO_ResetBits(csGPIOx,csGPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}	
	
	SPI_I2S_SendData(SPIx, 0X2000); 																
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	 
 
  SPI_I2S_ReceiveData(SPIx);       
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		
	
	SPI_I2S_SendData(SPIx, 0x0000); 														
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	
 
	SPI_I2S_ReceiveData(SPIx);		
		
	GPIO_SetBits(csGPIOx,csGPIO_Pin);
		
	delay_ms(50);
	
	GPIO_ResetBits(csGPIOx,csGPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		
	
	SPI_I2S_SendData(SPIx, 0X2000); 																
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	 
 
  SPI_I2S_ReceiveData(SPIx);	        
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		
	
	SPI_I2S_SendData(SPIx, 0x0000); 																
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	  
  
	SPI_I2S_ReceiveData(SPIx);
		
	GPIO_SetBits(csGPIOx,csGPIO_Pin);
	
	delay_ms(50);
		
	GPIO_ResetBits(csGPIOx,csGPIO_Pin);
	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		
	
	SPI_I2S_SendData(SPIx, 0X2000); 															
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	
 
	SPI_I2S_ReceiveData(SPIx);        
  
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}		
	
	SPI_I2S_SendData(SPIx, 0x0000); 																	
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 	
 
	SPI_I2S_ReceiveData(SPIx);		
		
	GPIO_SetBits(csGPIOx,csGPIO_Pin);
		
	//rawDataRead(0x00);	
}
void deviceADXRS453::UpdateData(void)
{
	static float rateList[ADXRS453_UPDATE_FREQ]={0,0,0,0,0};
	static float tempList[ADXRS453_UPDATE_FREQ]={0,0,0,0,0};
	
	//shift data right one unit
	shiftRightData(rateList,ADXRS453_UPDATE_FREQ);
	shiftRightData(tempList,ADXRS453_UPDATE_FREQ);
	
	//send read command
	rawDataRead(0x00);
	//cast to int16_t
	rateList[0]=static_cast<int16_t>(rawDataRead(0x02))/80.0f;
	tempList[0]=static_cast<int16_t>(rawDataRead(0x0C))/64;
	
	//count for the num to make updating period 5 seconds
	static uint32_t countOut=0;
	countOut++;
	
	if(countOut>=ADXRS453_UPDATE_FREQ)
	{
		countOut=0;
		//mean the angular velocity
		val=meanData(rateList,ADXRS453_UPDATE_FREQ);
		temp=meanData(tempList,ADXRS453_UPDATE_FREQ);
		//cout<<endl;
	}

	//cout<<rateList[0]<<'\t'<<(int)tempList[0]<<endl;
	
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
