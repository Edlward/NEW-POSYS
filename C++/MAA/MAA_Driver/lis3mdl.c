/**
  ******************************************************************************
  * @file     
  * @author  lxy
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
#include "stdint.h"
#include "lis3mdl.h"
#include "spi.h"
#include "math.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static three_axis mag={0,0,0};
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
void LIS3MDL_Init(void)
{
	LIS3_Write(LIS3MDL_CTRL_REG1,0x7e);
	LIS3_Write(LIS3MDL_CTRL_REG2,0x00);
	LIS3_Write(LIS3MDL_CTRL_REG3,0x00);
	LIS3_Write(LIS3MDL_CTRL_REG4,0x0C);
	LIS3_Write(LIS3MDL_CTRL_REG5,0x00);
}
void LIS3MDL_UpdateMag(void)
{
	static float vector_min=9999999;
	static float vector_max=0;
	
	static three_axis mag_max = {0,0,0};
	static three_axis mag_min = {0,0,0};
	
	static float sum; 
	
	uint8_t data[2];
	data[1]=LIS3_Read(LIS3MDL_OUT_X_H);
	data[0]=LIS3_Read(LIS3MDL_OUT_X_L);
	mag.x=  ((int16_t)(data[1]<<8|data[0]))/6.842f;
	
	data[1]=LIS3_Read(LIS3MDL_OUT_Y_H);
	data[0]=LIS3_Read(LIS3MDL_OUT_Y_L);
	mag.y=  ((int16_t)(data[1]<<8|data[0]))/6.842f;
	
	data[1]=LIS3_Read(LIS3MDL_OUT_Z_H);
	data[0]=LIS3_Read(LIS3MDL_OUT_Z_L);
	mag.z=  ((int16_t)(data[1]<<8|data[0]))/6.842f;
	
	
	sum=mag.x*mag.x+mag.y*mag.y+mag.z*mag.z;
	
	if(sum>vector_max)
	{
		mag_max.x=mag.x;
		mag_max.y=mag.y;
		mag_max.z=mag.z;
		vector_max=sum;
	}
	if(sum<vector_min)
	{
		mag_min.x=mag.x;
		mag_min.y=mag.y;
		mag_min.z=mag.z;
		vector_min=sum;
	}
	
}
void LIS_readMag(three_axis *val)
{
	val->x=-mag.x;
	val->y=-mag.y;
	val->z= mag.z;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/




