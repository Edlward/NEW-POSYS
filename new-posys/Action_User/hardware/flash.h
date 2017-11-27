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
#define TempTable_Num  	261		//����Ͳ��궨����Ҫ�ú���һ��		
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Flash_Write(uint8_t *data,uint32_t len);
void Flash_Zero(uint32_t len);
void Flash_Read(uint8_t *data,uint32_t len);
void Flash_Init(void);
uint8_t  *GetFlashArr(void);
void Flash_Return(void);
/*
chartW����ʽ  
������1��X�ᣨTEMP_SAMPLE_NUMBER�������Y�ᣨTEMP_SAMPLE_NUMBER�������Z�ᣨTEMP_SAMPLE_NUMBER���������
������2��X�ᣨTEMP_SAMPLE_NUMBER�������Y�ᣨTEMP_SAMPLE_NUMBER�������Z�ᣨTEMP_SAMPLE_NUMBER���������
������3��X�ᣨTEMP_SAMPLE_NUMBER�������Y�ᣨTEMP_SAMPLE_NUMBER�������Z�ᣨTEMP_SAMPLE_NUMBER���������

chart+��������ţ�0-(GYRO_NUMBER-1��*AXIS_NUMBER*+��ţ�0-(AXIS_NUMBER-1��*TEMP_SAMPLE_NUMBER+����ţ�0-(TEMP_SAMPLE_NUMBER-1)��
*/


/*
chartMode��ʽ  
������1��X��  Y��  Z�ᣩ
������2��X��  Y��  Z�ᣩ
������3��X��  Y��  Z�ᣩ

chart+��������ţ�0-(GYRO_NUMBER-1)��*AXIS_NUMBER+��ţ�0-(AXIS_NUMBER-1����
*/
typedef struct{
	/*���������ǵ�����������ֽ�� GYRO_NUMBER*AXIS_NUMBER*TEMP_SAMPLE_NUMBER��float��*/
	float     *chartW;
	/*��С���ٶ���ֵ �� GYRO_NUMBER��*/
	float  		*minValue;
	/*���������ǵ�������ٶȷ��� �� GYRO_NUMBER��*/
	float     *varXYZ;
	/*�Ƿ�����֮ǰ�ƶϵı�׼���� 3��������������   GYRO_NUMBER*AXIS_NUMBER��*/
	uint8_t 	*chartMode;
	/*������һ����Щ���� GYRO_NUMBER*AXIS_NUMBER*TEMP_SAMPLE_NUMBER��boolֵ*/
	uint8_t 	*chartSelect;
	/*�������ٶȵķ�Χ �� GYRO_NUMBER��*/
	uint8_t   *scaleMode;
}flashData_t;


#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
