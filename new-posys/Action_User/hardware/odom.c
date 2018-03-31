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


#define ODOM_STATE_POWER_ON 0
#define ODOM_STATE_SETUP 1
#define ODOM_STATE_CONFIGURE 2
#define ODOM_STATE_DEINIT 3


#define ODOM_MAX_READ_RETRY_TIMES	(10)
#define ODOM_READ_RETRY_INTERVAL	(10)
#define ODOM_MAX_GAP (4096)

#define ODOM_CHECK_TIMES (200)
#define ODOM_FATIGUE_TEST_TIMES (100000)

//min cs high time is 500us, we set 1us here
#define ODOM_MIN_CS_HIGH_TIME (1)

//time between CS falling edge and first falling edge of CLK
//is 500ns, we set 1us here
#define CS_TO_CLK_TIME (1)

typedef struct 
{
	uint32_t value;
	uint8_t ocf;
  uint8_t	cof;
  uint8_t	lin;
  uint8_t	magInc;
	uint8_t magDec;
	uint8_t even;
	uint8_t dum[2];
}AS5405BRawData_t;

static uint32_t gOdomState = ODOM_STATE_POWER_ON;

/**
  * @brief  Setup two AS5405B communication.
  * @note   #1 AS5405B and #2 AS5405B function as a SPI slave with id of 0 and 1.
  *		    STM32 use SPI2 to communicate with them.
  *	@note   PinAF:
  *         PB12 -- #1 CS
  *         PB13 -- SCK
  *         PB14 -- MISO
  *         PB15 -- #2 CS
  * @param  none
  * @param  none
  * @retval none
  */
void ODOM_Setup(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;             
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);   
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;             
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;				     							 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;															 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;														 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																	 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;				 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;											 
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_Cmd(SPI2, ENABLE);
	
	gOdomState = ODOM_STATE_SETUP;
}

/**
  * @brief  Configure ICM20608-G registers.
  * @note   configure none now.
  *	@note   
  * @param  none
  * @param  none
  * @retval none
  */
void ODOM_Configure(void)
{
	
	gOdomState = ODOM_STATE_CONFIGURE;
}

/**
  * @brief  A
  * @note  A
  * @note  A
  * @param  A
  * @param  A
  * @retval  A
  */
static int ODOM_ReadRawData(Odom_t* rawData)
{
	uint32_t retryCount = 0;
	uint8_t  buf[3] = {0}, i = 0;
	int shift = 24, sum = 0;

	AS5405BRawData_t data[2] = {0};
	
	if(rawData == (void *)0) return ODOM_INVALID_PARAM_ERROR;

	if(gOdomState != ODOM_STATE_CONFIGURE) return ODOM_NOT_READY_ERROR;
	
	while(1)
	{

		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		
		delay_us(CS_TO_CLK_TIME);
		SPI_Cmd(SPI2,ENABLE);
		
		for(i=0;i<3;i++)
		{
			while((SPI2->SR & SPI_I2S_FLAG_RXNE) == (u16)RESET);
			buf[i] = SPI2->DR;
		}
		

		GPIO_SetBits(GPIOB,GPIO_Pin_15);

		SPI_Cmd(SPI2,DISABLE);

		delay_us(ODOM_MIN_CS_HIGH_TIME);
		data[0].value = (((uint32_t)buf[0]<<16) | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]));

		data[0].ocf = (data[0].value>>11 &0x01);
		data[0].cof = (data[0].value>>10 &0x01);
		data[0].lin = (data[0].value>>9 &0x01);
		data[0].magInc = (data[0].value>>8 &0x01);
		data[0].magDec = (data[0].value>>7 &0x01);
		
		while(shift >= 6)
		{
			shift--;
			sum += ((data[0].value>>shift)&0x01);
		}
		if(sum %2 != 0) 
		{
			delay_us(ODOM_READ_RETRY_INTERVAL);
			retryCount++;
			if(retryCount > ODOM_MAX_READ_RETRY_TIMES) break;
			continue;
		}
		
		if(!(data[0].ocf == 1 &&data[0].cof == 0 && data[0].lin ==0 /* && data[0].magInc == 0 && data[0].magDec == 0*/)) 
		{
			delay_us(ODOM_READ_RETRY_INTERVAL);
			retryCount++;
			if(retryCount > ODOM_MAX_READ_RETRY_TIMES) break;
			continue;
		}
		break;
	}
	
	if(retryCount > ODOM_MAX_READ_RETRY_TIMES) return ODOM_READ_TIMEOUT_ERROR1;
	retryCount = 0;
	
	while(1)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);

		delay_us(CS_TO_CLK_TIME);		
		SPI_Cmd(SPI2,ENABLE);
		for(i=0;i<3;i++)
		{
			while((SPI2->SR & SPI_I2S_FLAG_RXNE) == (u16)RESET);
			buf[i] = SPI2->DR;
		}
		
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
		SPI_Cmd(SPI2,DISABLE);

		delay_us(ODOM_MIN_CS_HIGH_TIME);
		
		data[1].value = (((uint32_t)buf[0]<<16) | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]));

		data[1].ocf = (data[1].value>>11 &0x01);
		data[1].cof = (data[1].value>>10 &0x01);
		data[1].lin = (data[1].value>>9 &0x01);
		data[1].magInc = (data[1].value>>8 &0x01);
		data[1].magDec = (data[1].value>>7 &0x01);
		
		while(shift >= 6)
		{
			shift--;
			sum += ((data[1].value>>shift)&0x01);
		}
		if(sum %2 != 0) 
		{
			delay_us(ODOM_READ_RETRY_INTERVAL);
			retryCount++;
			if(retryCount > ODOM_MAX_READ_RETRY_TIMES) break;
			continue;
		}
		
		if(!(data[1].ocf == 1 && data[1].cof == 0 && data[1].lin ==0 /* && data[1].magInc == 0 && data[1].magDec == 0*/)) 
		{
			delay_us(ODOM_READ_RETRY_INTERVAL);
			retryCount++;
			if(retryCount > ODOM_MAX_READ_RETRY_TIMES) break;
			continue;
		}

		break;
	}
	
	if(retryCount > ODOM_MAX_READ_RETRY_TIMES) return ODOM_READ_TIMEOUT_ERROR2;
	
	rawData->odom2 = (data[0].value>>12) & 0xffff;
	rawData->odom1 = (data[1].value>>12) & 0xffff;

	return ODOM_READ_OK;
}

/**
  * @brief  A
  * @note  A
  * @note  A
  * @param  A
  * @param  A
  * @retval  A
  */
int ODOM_Read(Odom_t* odomData, int mode)
{
	int retVal = 0;
	static Odom_t lastData = {0.0f, 0.0f};
	
	if(odomData == (void *)0) return ODOM_INVALID_PARAM_ERROR;

	
	if(mode == ODOM_INCREMENTAL_MODE)
	{
		retVal = ODOM_ReadRawData(odomData);
		if(retVal == ODOM_READ_OK)
		{
			double diff = odomData->odom1 - lastData.odom1;
			if(diff > ODOM_MAX_GAP/2)
			{
				diff -= ODOM_MAX_GAP;
			}
			else if(diff < -ODOM_MAX_GAP/2)
			{
				diff += ODOM_MAX_GAP;
			}
			
			odomData->odom1 =  diff;
			
			
			diff = odomData->odom2 - lastData.odom2;
			if(diff > ODOM_MAX_GAP/2)
			{
				diff -= ODOM_MAX_GAP;
			}
			else if(diff < -ODOM_MAX_GAP/2)
			{
				diff += ODOM_MAX_GAP;
			}
			
			odomData->odom2 =  diff;

			lastData.odom1 = odomData->odom1;
			lastData.odom2 = odomData->odom2;

			return ODOM_READ_OK;
		}
		else
		{
			return retVal;
		}
		
	}
	else
	{
		return ODOM_INVALID_PARAM_ERROR;
	}
}

/**
  * @brief  Check ODOM by reading operation for ODOM_TEST_TIMES times.
  * @note  A
  * @note  A
  * @param  A
  * @param  A
  * @retval  A
  */
int ODOM_Check(void)
{
	Odom_t odomData;
	int retVal = 0, i = 0;
	
	
	if(gOdomState != ODOM_STATE_CONFIGURE) return ODOM_NOT_READY_ERROR;
	
	for(i = 0; i < ODOM_CHECK_TIMES; i++)
	{
		//random delay 4~6ms for test
		delay_ms(rand()%3 + 4);
		
		retVal = ODOM_ReadRawData(&odomData);
		if(retVal == ODOM_READ_OK) continue;
		return retVal;
	}
	
	return ODOM_READ_OK;
}

/**
  * @brief  Test ODOM read operation for ODOM_TEST_TIMES times.
  * @note  A
  * @note  A
  * @param  A
  * @param  A
  * @retval  A
  */
int ODOM_FatigureTest(void)
{
	Odom_t odomData;
	int retVal = 0, i = 0;
	
	
	if(gOdomState != ODOM_STATE_CONFIGURE) return ODOM_NOT_READY_ERROR;
	
	for(i = 0; i < ODOM_FATIGUE_TEST_TIMES; i++)
	{
		//random delay 4~6ms for test
		delay_ms(rand()%3 + 4);
		
		retVal = ODOM_ReadRawData(&odomData);
		if(retVal == ODOM_READ_OK) continue;
		return retVal;
	}
	
	return ODOM_READ_OK;
}

