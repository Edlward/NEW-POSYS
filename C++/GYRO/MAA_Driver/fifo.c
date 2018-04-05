/**
  ******************************************************************************
  * @file    fifo.c
  * @author  lxy zlq
  * @version 
  * @date    
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
#include "fifo.h"
#include "maa_config.h"
#include "spi.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/**
  * @brief   
  * @none
  * @retval  
  */
void FIFO_Read(FIFO_Data *data)
{	
	uint8_t SRC;
	
	uint8_t i;
	uint8_t buf[2]={0,0};
	uint16_t buf16;
	
	SRC=L3GD20H_Read(L3GD20H_FIFO_SRC_REG);
	(*data).FIFO_FTH=SRC>>7;
	(*data).FIFO_Full=(SRC&0X7F)>>6;
	(*data).FIFO_Empty=(SRC&0X3F)>>5;		
	(*data).FIFO_len=(SRC&0X1F);
	for(i=0;i<(*data).FIFO_len;i++)
	{
		buf[0]=L3GD20H_Read(L3GD20H_OUT_X_L);
		buf[1]=L3GD20H_Read(L3GD20H_OUT_X_H);	
		buf16=(buf[0])|((buf[1])<<8);
		(*data).x[i]=((short)buf16)*0.00875;
		
		buf[0]=L3GD20H_Read(L3GD20H_OUT_Y_L);
		buf[1]=L3GD20H_Read(L3GD20H_OUT_Y_H);
		buf16=(buf[0])|((buf[1])<<8);
		(*data).y[i]=((short)buf16)*0.00875;
		
		buf[0]=L3GD20H_Read(L3GD20H_OUT_Z_L);
		buf[1]=L3GD20H_Read(L3GD20H_OUT_Z_H);	
		buf16=(buf[0])|((buf[1])<<8);
		(*data).z[i]=((short)buf16)*0.00875;
	}
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
