/**
  ******************************************************************************
  * @file    LSM303AGR.cpp
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
#include "LSM303AGR.h"
#include "map"
#include "timer.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/

#define HARDWARE_TEST_Acc_REG 0x0F
#define HARDWARE_TEST_Acc_VAL 0x33

#define HARDWARE_TEST_Mag_REG 0x4F
#define HARDWARE_TEST_Mag_VAL 0x40

#define ACC_DATA_REG_BEGIN 0x28
#define MAG_DATA_REG_BEGIN 0x68

#define ACC_AVAIABLE ((rawDataRead(0x27)&0x08)>>3)
#define MAG_AVAIABLE ((rawDataRead(0x67)&0x08)>>3)

/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static deviceLSM303AGR_Acc LSM303AGR_Acc(SPI3,GPIOB,GPIO_Pin_5);
static deviceLSM303AGR_Mag LSM303AGR_Mag(SPI3,GPIOB,GPIO_Pin_6);
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
deviceLSM303AGR_Acc& getLSM303AGR_Acc(void)
{
	return LSM303AGR_Acc;
}
deviceLSM303AGR_Mag& getLSM303AGR_Mag(void)
{
	return LSM303AGR_Mag;
}
uint8_t 	deviceLSM303AGR_Acc::rawDataRead(uint8_t address)
{
	return SPI_HalfDuplex_Read(SPIx,csGPIOx,csGPIO_Pin,address);
}
void 		deviceLSM303AGR_Acc::rawDataWrite(uint8_t address,uint8_t value)
{
	SPI_HalfDuplex_Write(SPIx,csGPIOx,csGPIO_Pin,address,value);
}
void 		deviceLSM303AGR_Acc::init(void)
{
	std::map<uint8_t,uint8_t> regWriteCheck;
	
	regWriteCheck[0x1F]=0x00;
	regWriteCheck[0x20]=0x57;
	regWriteCheck[0x21]=0x00;
	regWriteCheck[0x22]=0x00;
	regWriteCheck[0x23]=0x89;
	regWriteCheck[0x24]=0x00;
	regWriteCheck[0x25]=0x00;
	regWriteCheck[0x2E]=0x00;
	
	for(std::map<uint8_t,uint8_t>::iterator iter=regWriteCheck.begin();iter!=regWriteCheck.end();iter++)
	{
		rawDataWrite(iter->first,iter->second);
	}
	for(std::map<uint8_t,uint8_t>::iterator iter=regWriteCheck.begin();iter!=regWriteCheck.end();iter++)
	{
		if(rawDataRead(iter->first)!=iter->second)
		{
			if(rawDataRead(HARDWARE_TEST_Acc_REG)!=HARDWARE_TEST_Acc_VAL)
				while(1);
			else
				init();
		}
	}
}
void    deviceLSM303AGR_Acc::updateData(void)
{	
	int16_t temp;
	temp=(static_cast<uint16_t>(rawDataRead(ACC_DATA_REG_BEGIN+1))<<8)|(static_cast<uint16_t>(rawDataRead(ACC_DATA_REG_BEGIN)));
	temp>>=4;  
	val.x=-temp*0.98f;
	temp=(static_cast<uint16_t>(rawDataRead(ACC_DATA_REG_BEGIN+3))<<8)|(static_cast<uint16_t>(rawDataRead(ACC_DATA_REG_BEGIN+2)));
	temp>>=4;
	val.y=-temp*0.98f;
	temp=(static_cast<uint16_t>(rawDataRead(ACC_DATA_REG_BEGIN+5))<<8)|(static_cast<uint16_t>(rawDataRead(ACC_DATA_REG_BEGIN+4)));
	temp>>=4;
	val.z=-temp*0.98f;
}
void 		deviceLSM303AGR_Mag::init(void)
{      
	std::map<uint8_t,uint8_t> regWriteCheck;
	
	regWriteCheck[0x60]=0x0C;
	regWriteCheck[0x61]=0x03;
	regWriteCheck[0x62]=0x30;
	regWriteCheck[0x63]=0x00;
	
	for(std::map<uint8_t,uint8_t>::iterator iter=regWriteCheck.begin();iter!=regWriteCheck.end();iter++)
	{
		rawDataWrite(iter->first,iter->second);
	}
	for(std::map<uint8_t,uint8_t>::iterator iter=regWriteCheck.begin();iter!=regWriteCheck.end();iter++)
	{
		if(rawDataRead(iter->first)!=iter->second)
		{
			if(rawDataRead(HARDWARE_TEST_Mag_REG)!=HARDWARE_TEST_Mag_VAL)
				while(1);
			else
				init();
		}
	}
}
void    deviceLSM303AGR_Mag::updateData(void)
{
	int16_t temp;
	temp=(static_cast<uint16_t>(rawDataRead(MAG_DATA_REG_BEGIN+1))<<8)|(static_cast<uint16_t>(rawDataRead(MAG_DATA_REG_BEGIN)));
	val.x=-temp*1.5f;
	temp=(static_cast<uint16_t>(rawDataRead(MAG_DATA_REG_BEGIN+3))<<8)|(static_cast<uint16_t>(rawDataRead(MAG_DATA_REG_BEGIN+2)));
	val.y=-temp*1.5f;
	temp=(static_cast<uint16_t>(rawDataRead(MAG_DATA_REG_BEGIN+5))<<8)|(static_cast<uint16_t>(rawDataRead(MAG_DATA_REG_BEGIN+4)));
	val.z=-temp*1.5f;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
