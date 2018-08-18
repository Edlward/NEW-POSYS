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
*`
* 
******************************************************************************
*/ 
/* Includes -------------------------------------------------------------------*/
#include "flash.h"
#include "stm32f4xx_flash.h"
#include "string.h"
#include "config.h"
#include "math.h"
#include "usart.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/



uint16_t STMFLASH_GetFlashSector(unsigned int addr)
{
  if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
  else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
  else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
  else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
  else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
  else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
  else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
  else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
  else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
  else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
  else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10;
  return FLASH_Sector_11;	
}

/**
* @brief  FLASH����
*
* @param  None
* @retval None
*/
//static void Flash_Encryp(void)
//{
//  /* �ж�FLASH�Ƿ񱻱��� */
//  if(SET != FLASH_OB_GetRDP())
//  {
//    /* ���������� */
//    FLASH_Unlock();
//    FLASH_OB_Unlock();
//    /* �ȼ�0���������ȼ�1������������ȼ�2����������� */
//    FLASH_OB_RDPConfig(OB_RDP_Level_1);
//    FLASH_OB_Launch();
//    FLASH_OB_Lock();
//    FLASH_Lock();
//  }
//}



/**
* @brief  ��ʼ��FLASH
*
* @param  None
* @retval None
*/
void Flash_Init(void)
{
	// HCLLKΪ168MHz��������ѹΪ3.3V
	FLASH_SetLatency(5);
	

}




/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/

