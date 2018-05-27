/**
  ******************************************************************************
  * @file    
  * @author Billy
  * @version V2.0
  * @date    01-May-2018
  * @brief   This file contains all the functions prototypes for orthogonal odom 
  *          information provided by two megnetic encoders AS5045B
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 Action Robot</center></h2>
  *
  * @note
  ******************************************************************************
  */ 

#include <stdint.h>
#include <stdlib.h>
#include "timer.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "odom.h"
#include "spi.h"
/* Private define ------------------------------------------------------------*/

#define   DUMMY_BYTE          0xFFFF      //哑元字节 SPI 读数据用

/* Private variables ---------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

static void	TLE5012_A_CS_ENABLE(void);
static void	TLE5012_A_CS_DISABLE(void);
static void	TLE5012_B_CS_ENABLE(void);
static void	TLE5012_B_CS_DISABLE(void);
static void	SPI2_TX_OFF(void);
static void	SPI2_TX_ON(void);
static void	SPI3_TX_OFF(void);
static void	SPI3_TX_ON(void);
uint16_t	TLE5012WriteReg(SPI_TypeDef* SPIx, uint16_t u16Command, uint16_t* pu16Data);

/**
 * @brief  读取TLE5012的寄存器
 * @param  commond：命令字
 * @param  reData：数据指针
 * @retval safety word
 */
 
uint16_t TLE5012ReadAbsPos_A(void)
{
	uint16_t u16Command = 0xFFFF;
	uint16_t u16Data = 0x0000;
	u16Command = READ_ANGLE_VALUE;
	TLE5012ReadReg(SPI2, u16Command, &u16Data);//返回safe
	u16Data = u16Data & 0x7FFF;
	return ((int16_t)(u16Data << 1)) >> 1;
}
uint16_t TLE5012ReadAbsPos_B(void)
{
	uint16_t u16Command = 0xFFFF;
	uint16_t u16Data = 0x0000;
	u16Command = READ_ANGLE_VALUE;
	TLE5012ReadReg(SPI3, u16Command, &u16Data);//返回safe
	u16Data = u16Data & 0x7FFF;
	return ((int16_t)(u16Data << 1)) >> 1;
}

uint16_t	TLE5012ReadReg(SPI_TypeDef* SPIx, uint16_t u16Command, uint16_t* pu16Data)
{
	uint16_t u16Safe = 0;
	
	if(SPIx==SPI2)
	{
		SPI2_TX_ON();
		TLE5012_A_CS_ENABLE();
	}else if(SPIx==SPI3){
		SPI3_TX_ON();
		TLE5012_B_CS_ENABLE();
	}
	
	*pu16Data = SPIx_RdWr_HalfWord(SPIx, u16Command);	
	
	if(SPIx==SPI2)
	{
		SPI2_TX_OFF();
	}else if(SPIx==SPI3){
		SPI3_TX_OFF();
	}
	
	*pu16Data = SPIx_RdWr_HalfWord(SPIx, DUMMY_BYTE);	
	u16Safe = SPIx_RdWr_HalfWord(SPIx, DUMMY_BYTE);
	
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
	
	if(SPIx==SPI2)
	{
		TLE5012_A_CS_DISABLE();											//禁止模块的片选信号
	}else if(SPIx==SPI3){
		TLE5012_B_CS_DISABLE();											//禁止模块的片选信号
	}

	return u16Safe;
}

uint16_t	TLE5012WriteReg(SPI_TypeDef* SPIx, uint16_t u16Command, uint16_t* pu16Data)
{
	uint16_t	u16Safe = 0;

	if(SPIx==SPI2)
	{
		SPI2_TX_ON();
		TLE5012_A_CS_ENABLE();
	}else if(SPIx==SPI3){
		SPI3_TX_ON();
		TLE5012_B_CS_ENABLE();
	}

	u16Safe = SPIx_RdWr_HalfWord(SPIx, u16Command);	
	u16Safe = SPIx_RdWr_HalfWord(SPIx, *pu16Data);
	
	if(SPIx==SPI2)
	{
		SPI2_TX_OFF();
	}else if(SPIx==SPI3){
		SPI3_TX_OFF();
	}
	
	u16Safe = SPIx_RdWr_HalfWord(SPIx, DUMMY_BYTE);
	/* Wait until SPIx TXD RXD Complete */
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
	
	if(SPIx==SPI2)
	{
		TLE5012_A_CS_DISABLE();											//禁止模块的片选信号
	}else if(SPIx==SPI3){
		TLE5012_B_CS_DISABLE();											//禁止模块的片选信号
	}

//	printf("\r\n @Sky [TLE5012ReadReg] RegData：0x%04X; SafeWord：0x%04X\r\n", *pu16Data, u16Safe);
	return u16Safe;
}


static void	TLE5012_A_CS_ENABLE(void)
{
//	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
}

static void	TLE5012_A_CS_DISABLE(void)
{
//	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}
static void	TLE5012_B_CS_ENABLE(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
}

static void	TLE5012_B_CS_DISABLE(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
}
static void	SPI2_TX_OFF(void)
{         
	//PB15--MOSI输入模式                   
	GPIOB->MODER &= 0x3FFFFFFF; 
	GPIOB->MODER |= 0x00000000; 
}
static void	SPI2_TX_ON(void)
{
	//PB15--MOSI复用                            
	GPIOB->MODER &= 0x3FFFFFFF; 
	GPIOB->MODER |= 0x80000000; 
}
static void	SPI3_TX_OFF(void)
{         
	//PC12--MOSI输入模式                   
	GPIOC->MODER &= 0xFCFFFFFF; 
	GPIOC->MODER |= 0x00000000; 
}
static void	SPI3_TX_ON(void)
{
	//PC12--MOSI复用                            
	GPIOC->MODER &= 0xFCFFFFFF; 
	GPIOC->MODER |= 0x02000000; 
}


