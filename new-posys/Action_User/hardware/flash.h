/**
  ******************************************************************************
  * @file    *.h
  * @author  Qzj Action
  * @version 
  * @date   
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  *
  * 
  * 
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H
#define __FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define TempTable_Num  	315		//这个就不宏定义了要好好算一下		
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Flash_Write(uint8_t *data,uint32_t len);
void Flash_Zero(uint32_t len);
void Flash_Read(uint8_t *data,uint32_t len);
void Flash_Init(void);
uint8_t  *GetFlashArr(void);
void Flash_Return(void);
/*
chartW排序方式  
陀螺仪1（X轴（TEMP_SAMPLE_NUMBER个结果）Y轴（TEMP_SAMPLE_NUMBER个结果）Z轴（TEMP_SAMPLE_NUMBER个结果））
陀螺仪2（X轴（TEMP_SAMPLE_NUMBER个结果）Y轴（TEMP_SAMPLE_NUMBER个结果）Z轴（TEMP_SAMPLE_NUMBER个结果））
陀螺仪3（X轴（TEMP_SAMPLE_NUMBER个结果）Y轴（TEMP_SAMPLE_NUMBER个结果）Z轴（TEMP_SAMPLE_NUMBER个结果））

chart+陀螺仪序号（0-(GYRO_NUMBER-1）*AXIS_NUMBER*+轴号（0-(AXIS_NUMBER-1）*TEMP_SAMPLE_NUMBER+结果号（0-(TEMP_SAMPLE_NUMBER-1)）
*/


/*
chartMode方式  
陀螺仪1（X轴  Y轴  Z轴）
陀螺仪2（X轴  Y轴  Z轴）
陀螺仪3（X轴  Y轴  Z轴）

chart+陀螺仪序号（0-(GYRO_NUMBER-1)）*AXIS_NUMBER+轴号（0-(AXIS_NUMBER-1））
*/
typedef struct{
	/*三个陀螺仪的三个轴的五种结果 GYRO_NUMBER*AXIS_NUMBER*TEMP_SAMPLE_NUMBER个float数*/
	float     *chartW;
	/*最小角速度阈值 共 GYRO_NUMBER*AXIS_NUMBER个*/
	float  		*minValue;
	/*三个陀螺仪的三轴角速度方差 共 GYRO_NUMBER*AXIS_NUMBER个*/
	float     *varXYZ;
	/*是否利用之前推断的标准数据 3个陀螺仪三个轴   GYRO_NUMBER*AXIS_NUMBER个*/
	uint8_t 	*chartMode;
	/*利用上一次哪些数据 GYRO_NUMBER*AXIS_NUMBER*TEMP_SAMPLE_NUMBER个bool值*/
	uint8_t 	*chartSelect;
	/*测量角速度的范围 共 GYRO_NUMBER*AXIS_NUMBER个*/
	uint8_t   *scaleMode;
}flashData_t;


#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
