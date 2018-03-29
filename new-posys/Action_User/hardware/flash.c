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
#define FLASH_USER_ADDRESS 0x08040000   //FLASH起始地址
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

FlashData_t flashData={0};


/**
* @brief  FLASH加密
*
* @param  None
* @retval None
*/
//static void Flash_Encryp(void)
//{
//  /* 判断FLASH是否被保护 */
//  if(SET != FLASH_OB_GetRDP())
//  {
//    /* 若不被保护 */
//    FLASH_Unlock();
//    FLASH_OB_Unlock();
//    /* 等级0不保护，等级1可逆读保护，等级2不可逆读保护 */
//    FLASH_OB_RDPConfig(OB_RDP_Level_1);
//    FLASH_OB_Launch();
//    FLASH_OB_Lock();
//    FLASH_Lock();
//  }
//}



/**
* @brief  初始化FLASH
*
* @param  None
* @retval None
*/
//void Flash_Init(void)
//{
	

//}

//void STMFLASH_Write(FlashData_t *pBuffer,unsigned int resetTime)	
//{ 
//  unsigned int* address=(unsigned int*)pBuffer;
//  unsigned int WriteAddr=FLASH_USER_ADDRESS+resetTime*sizeof(FlashData_t);
//  unsigned int endaddr=WriteAddr+sizeof(FlashData_t);
//  
//  FLASH_Unlock();									//解锁 
//  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
//  
//  while(WriteAddr<endaddr)//写数据
//  {
//    if(FLASH_ProgramWord(WriteAddr,*address)!=FLASH_COMPLETE)//写入数据
//    { 
//      address=address;	//写入异常
//    }
//		/*WriteAddr是整数，要加4，address是指针只用加一*/
//    WriteAddr+=MAX_SIZE;
//    address++;
//  }
//  
//  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
//  FLASH_Lock();//上锁
//} 

//void copyFlashData_tFromOther(FlashData_t* copy,FlashData_t* const reality)
//{
//	unsigned int* address=(unsigned int*)reality;
//  unsigned int* WriteAddr=(unsigned int*)copy;
//  unsigned int* endaddr=WriteAddr+sizeof(FlashData_t)/MAX_SIZE;
//	
//  while(WriteAddr<endaddr)//写数据
//  {
//    *WriteAddr=*address;
//    WriteAddr++;
//    address++;
//  }
//}

//void copyFlashData_tAlongAddress(FlashData_t* copy,uint32_t address)
//{
//  unsigned int* WriteAddr=(unsigned int*)copy;
//  unsigned int* endaddr=WriteAddr+sizeof(FlashData_t)/MAX_SIZE;
//	
//  while(WriteAddr<endaddr)//写数据
//  {
//    *WriteAddr=STMFLASH_ReadWord(address);
//    WriteAddr++;
//    address+=MAX_SIZE;
//  }
//}

///*把传入的机器人结构体的参数保存到“flash写入结构体”里*/
//void WriteFlashData(AllPara_t robot,unsigned int resetTime)
//{
//	copyFlashData_tFromOther(&dataSave,&robot.sDta);
//  
//  STMFLASH_Write(&dataSave,allPara.resetTime);
//}


///*读出第resetTime个结构体的值，resetTime取0 1 2 3*/
//void STMFLASH_Read(FlashData_t* temp,uint32_t resetTime)   	
//{
//  uint32_t baseAdd=FLASH_USER_ADDRESS+resetTime*sizeof(FlashData_t);
//  
//	copyFlashData_tAlongAddress(temp,baseAdd);
//}



/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/

