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
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
#define FLASH_USER_ADDRESS 0x08040000   //FLASH起始地址
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static uint8_t  flashdata[(4+4)*(int)(1.f/TempStep)*(TempTable_max-TempTable_min)];  //从flash中取出的数据

/**
  * @brief  从FLASH里得到的数据分两个部分，第一部分为数据区，第二部分为
  *         计数区，该函数用于获得数据区的指针
  * @param  None
  * @retval Result : 指向flash数据区的指针
  */
float    *chartW;
/**
  * @brief  从FLASH里得到的数据分两个部分，第一部分为数据区，第二部分为
  *         计数区，该函数用于获得计数区的指针
  * @param  None
  * @retval Result : 指向flash计数区的指针
  */
uint32_t *chartNum;
static uint8_t  flag=0;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/**
  * @brief  FLASH加密
	*
  * @param  None
  * @retval None
  */
static void Flash_Encryp(void)
{
	#ifdef FLASH_ENCRYP
	
	/* 判断FLASH是否被保护 */
	if(SET != FLASH_OB_GetRDP())
  {
		    /* 若不被保护 */
        FLASH_Unlock();
        FLASH_OB_Unlock();
		    /* 等级0不保护，等级1可逆读保护，等级2不可逆读保护 */
        FLASH_OB_RDPConfig(OB_RDP_Level_1);
        FLASH_OB_Launch();
        FLASH_OB_Lock();
        FLASH_Lock();
  }
	#endif
}
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/**
  * @brief  向FLASH中写入数据
	*
  * @param  data :  存储数据的指针
  * @param  len  :  写入的数据量，单位：byte
  * @retval None
  */
void Flash_Write(uint8_t *data,uint32_t len)
{
  uint32_t count;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3);
	FLASH_WaitForLastOperation();
	for(count=0;count<len;count++)
	{
		FLASH_ProgramByte(FLASH_USER_ADDRESS+count,*(data+count));
	}
	FLASH_Lock();
}
/**
  * @brief  将FLASH中的数据改成0
	*
  * @param  len  :  改写的数据量，单位：byte
  * @retval None
  */
void Flash_Zero(uint32_t len)
{
  uint32_t count;
	FLASH_Unlock();
	/*将标志位都写1*/
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	/*我们的405芯片用的是3.3V,所以选择VoltageRange_3*/
	FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3);
	FLASH_WaitForLastOperation();
	for(count=0;count<len;count++)
	{
		FLASH_ProgramByte(FLASH_USER_ADDRESS+count,0x00);
	}
	FLASH_Lock();
}

/**
  * @brief  从FLASH中读取数据
	*
  * @param  data :  存储数据的指针
  * @param  len  :  读取的数据量，单位：byte
  * @retval None
  */
void Flash_Read(uint8_t *data,uint32_t len)
{
	uint32_t i;
	for(i=0;i<len;i++)
	//一个字节一个字节地进行赋值
	  *(data+i)= *((uint8_t *)(FLASH_USER_ADDRESS+i));
}
/**
  * @brief  初始化FLASH
	*
  * @param  None
  * @retval None
  */
void Flash_Init(void)
{
	/* 读取FLASH中保存的数据，并将其存到内存(RAM)里 */
	//static uint8_t  flashdata[160*(TempTable_max-TempTable_min)];  //从flash中取出的数据
	Flash_Read(flashdata,(4+4)*(int)(1.f/TempStep)*(TempTable_max-TempTable_min));  //50-30
	/* 分割数据段，将零漂值与计数值分开 */
	chartW=(float *)flashdata;
	//是因为已经转换成32位
	chartNum=(uint32_t *)(flashdata)+(int)(1.f/TempStep)*(TempTable_max-TempTable_min);
	
	/* 保护Flash数据 */
	Flash_Encryp();
}
/**
  * @brief  获取从FLASH里得到的数据的指针
	*
  * @param  None
  * @retval flashdata : 指向flash数据的指针
  */
uint8_t *GetFlashArr(void)
{
	return flashdata;
}

/**
  * @brief  获得FLASH是否需要更新的标志位
  *         
  * @param  None
  * @retval flag : flash更新的标志位
  */
uint8_t GetFlashUpdataFlag(void)
{
	return flag;
}
/**
  * @brief  设置FLASH更新标志位的值
  *         
  * @param  val  : 赋给更新标志位的值
  * @retval None
  */
void  SetFlashUpdateFlag(uint8_t val)
{
	flag=val;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/

