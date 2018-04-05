#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C"
{
#endif

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
void CS_Config(void);
void SPI3_Init(void);

void SPI_Write(	 SPI_TypeDef *SPI,
				 GPIO_TypeDef *GPIOx,
				 uint16_t GPIO_Pin,
				 uint8_t address,
				 uint8_t value);

uint8_t SPI_Read(	SPI_TypeDef *SPIx,
					GPIO_TypeDef* GPIOx,
					uint16_t GPIO_Pin,
					uint8_t address);


void SPI_MultiRead(	 SPI_TypeDef *SPIx,
					 GPIO_TypeDef* GPIOx,
					 uint16_t GPIO_Pin,
					 uint8_t address,
					 uint8_t* data,
					 uint32_t 	len);

uint16_t SPI16_Read(  SPI_TypeDef *SPIx,
					  GPIO_TypeDef* GPIOx,
					  uint16_t GPIO_Pin,
					  uint8_t address);

uint16_t SPI16_Write(	SPI_TypeDef *SPIx,
						GPIO_TypeDef* GPIOx,
						uint16_t GPIO_Pin,
						uint8_t address,
						uint16_t sdata);

void SPI_HalfDuplex_Write(	SPI_TypeDef *SPIx,
							GPIO_TypeDef* GPIOx,
							uint16_t GPIO_Pin,
							uint8_t address,
							uint16_t sdata);

uint8_t SPI_HalfDuplex_Read(SPI_TypeDef *SPIx,
														GPIO_TypeDef* GPIOx,
														uint16_t GPIO_Pin,
														uint8_t address);
void SPI_OnlyRead(SPI_TypeDef *SPIx,
									GPIO_TypeDef* GPIOx,
									uint16_t GPIO_Pin,
									uint8_t* data,
									uint8_t len);
#ifdef __cplusplus
}
#endif

#endif


