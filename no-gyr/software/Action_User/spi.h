#ifndef __SPI_H
#define __SPI_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/*  ¥”ª˙∂¡–¥øÿ÷∆√¸¡Ó-------------------------------*/
/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

/* spi≈‰÷√∫Ø ˝-----------------------------------*/
void SPI1_Init(void);	
void SPI2_Init(void);
void SPI3_Init(void);
void CS_Config(void);
void SPI1_SetSpeed(uint8_t SpeedSet); 				

void SPI_Write(SPI_TypeDef *SPI,
	           GPIO_TypeDef* GPIOx,
	                uint16_t GPIO_Pin,
                   uint8_t address,
                   uint8_t value);
uint8_t SPI_Read(SPI_TypeDef *SPI,
	              GPIO_TypeDef* GPIOx,
	                   uint16_t GPIO_Pin,
                      uint8_t address);

uint8_t SPI2_ReadWriteByte(uint8_t TxData);
uint16_t SPI_ReadAS5045(uint8_t num);
void mRead(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
#endif


