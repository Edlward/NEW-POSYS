#ifndef __SPI_H
#define __SPI_H
#include "config.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"



/*  �ӻ���д��������-------------------------------*/
/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)

#define ICM_ReadByte(address)            	SPI_Read(SPI1,GPIOA,GPIO_Pin_4,address)
#define ICM_WriteByte(address,value)     	SPI_Write(SPI1,GPIOA,GPIO_Pin_4,address,value)

/* spi���ú���-----------------------------------*/
void ICM_SPIInit(void);	
void SPI1_Init(void);
void SPI2_Init(void);
void SPI3_Init(void);
void CS_Config(void);	

void SPI_Write(SPI_TypeDef *SPI,
	           GPIO_TypeDef* GPIOx,
	                uint16_t GPIO_Pin,
                   uint8_t address,
                   uint8_t value);
uint8_t SPI_Read(SPI_TypeDef *SPI,
	              GPIO_TypeDef* GPIOx,
	                   uint16_t GPIO_Pin,
                      uint8_t address);

void SPI_MultiRead(SPI_TypeDef *SPIx,
									 GPIO_TypeDef* GPIOx,
									 uint16_t GPIO_Pin,
									 uint8_t address,
									 uint8_t* data,
									 uint32_t 	len);

uint16_t	SPIx_RdWr_HalfWord(SPI_TypeDef* SPIx, uint16_t hw);


#endif

