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

//FLASH起始地址
#define STM32_FLASH_BASE 					0x08000000 	//STM32 FLASH的起始地址
#define FLASH_SAVE_ADDR 					0x08080000 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
#define MAX_SIZE									4




void STMFLASH_ERASE(void);
void WriteFlashData(AllPara_t robot,unsigned int resetTime);
void SoftWareReset(void);
void STMFLASH_Read(DataSave_t* flashdata,uint32_t resetTime) ;
void FindResetTime(void);
#endif



