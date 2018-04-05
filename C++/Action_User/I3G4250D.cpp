/**
  ******************************************************************************
  * @file    I3G4250D.cpp
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
#include "I3G4250D.h"
#include "spi.h"
#include "stm32f4xx_spi.h"
#include "map"
#include "timer.h"
#include "signalProcess.h"
#include "usart.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/

#define HARDWARE_TEST_REG 0x0F
#define HARDWARE_TEST_VAL 0xD3

#define GYRO_DATA_REG_BEGIN 0x28

#define DATA_AVAIABLE ((rawDataRead(0x27)&0x08)>>3)

/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static deviceI3G4250D I3G4250D(SPI1,GPIOC,GPIO_Pin_4);
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
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;											    /* SPI接收8位帧结构								*/
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;																/* 串行同步时钟的空闲状态为低电平	*/
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;															/* 第一个跳变沿数据被采样					*/
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	/* NSS由软件控制									*/
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;				/* 预分频													*/
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												/* 数据从MSB位开始								*/
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPIx, &SPI_InitStructure);
	SPI_Cmd(SPIx, ENABLE);
	
}
void deviceI3G4250D::multiRead(uint8_t address,uint8_t *data,uint32_t len)
{
	changeSPIconfigure(SPIx);
	SPI_MultiRead(SPIx,csGPIOx,csGPIO_Pin,address,data,len);
}
/* Exported function prototypes -----------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/
deviceI3G4250D& getI3G4250D(void)
{	
	return I3G4250D;
}
uint8_t 	deviceI3G4250D::rawDataRead(uint8_t address)
{
	changeSPIconfigure(SPIx);
	return SPI_Read(SPIx,csGPIOx,csGPIO_Pin,address);
}
void 		deviceI3G4250D::rawDataWrite(uint8_t address,uint8_t value)
{
	changeSPIconfigure(SPIx);
	SPI_Write(SPIx,csGPIOx,csGPIO_Pin,address,value);
}
void 		deviceI3G4250D::init(void)
{
	std::map<uint8_t,uint8_t> regWriteCheck;
	
	regWriteCheck[0x20]=0xFF;
	regWriteCheck[0x21]=0x09;
	regWriteCheck[0x22]=0x00;
	regWriteCheck[0x23]=0x20;
	regWriteCheck[0x24]=0x40;
	regWriteCheck[0x2E]=0x44;
	regWriteCheck[0x30]=0x00;
	regWriteCheck[0x32]=0x00;
 	regWriteCheck[0x33]=0x00;
	regWriteCheck[0x34]=0x00;
	regWriteCheck[0x35]=0x00;
	regWriteCheck[0x36]=0x00;
	regWriteCheck[0x37]=0x00;
	regWriteCheck[0x38]=0x00;
	
	for(std::map<uint8_t,uint8_t>::iterator iter=regWriteCheck.begin();iter!=regWriteCheck.end();iter++)
	{
		rawDataWrite(iter->first,iter->second);
	}
	for(std::map<uint8_t,uint8_t>::iterator iter=regWriteCheck.begin();iter!=regWriteCheck.end();iter++)
	{
		if(rawDataRead(iter->first)!=iter->second)
		{
			if(rawDataRead(HARDWARE_TEST_REG)!=HARDWARE_TEST_VAL)
				while(1);
			else
				init();
		}
	}
}
void    deviceI3G4250D::updateData(void)
{
	temp=rawDataRead(0x26);
	
	uint8_t dataLen=0;
	uint8_t data[6];
	int16_t tempVal;
	dataLen=rawDataRead(0x2f)&0x1f;
	
	float *dataX=new float[dataLen];
	float *dataY=new float[dataLen];
	float *dataZ=new float[dataLen];
	
	for(uint32_t i=0;i<dataLen;i++)
	{
		rawDataRead(GYRO_DATA_REG_BEGIN);
		multiRead(GYRO_DATA_REG_BEGIN+2,data,6);
		
		tempVal=(static_cast<uint16_t>(data[5])<<8)|(static_cast<uint16_t>(data[4]));
		dataY[i]=tempVal*0.07f;
		tempVal=(static_cast<uint16_t>(data[1])<<8)|(static_cast<uint16_t>(data[0]));
		dataX[i]=tempVal*0.07f;
		tempVal=(static_cast<uint16_t>(data[3])<<8)|(static_cast<uint16_t>(data[2]));
		dataZ[i]=-tempVal*0.07f;
	}
	
	val.x=averageWithRemoveErr(dataX,dataLen);
	val.y=averageWithRemoveErr(dataY,dataLen);
	val.z=averageWithRemoveErr(dataZ,dataLen);
		
	delete[] dataX;
	delete[] dataY;
	delete[] dataZ;
	//cout<<val.z<<'\t'<<(int)temp<<endl;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
