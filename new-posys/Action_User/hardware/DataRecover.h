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
#ifndef __DATA_RECOVERY_H
#define __DATA_RECOVERY_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "config.h"

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 					0x08000000 	//STM32 FLASH����ʼ��ַ
#define FLASH_SAVE_ADDR 					0x08080000 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.
#define MAX_SIZE									4




void STMFLASH_ERASE(void);
void WriteFlashData(AllPara_t robot,unsigned int resetTime);
void SoftWareReset(void);
void STMFLASH_Read(DataSave_t* flashdata,uint32_t resetTime) ;
void FindResetTime(void);
#endif



