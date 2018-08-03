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
#define FLASH_USER_ADDRESS 0x08040000   //FLASH��ʼ��ַ
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

FlashData_t flashData={0};


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
//void Flash_Init(void)
//{
	

//}

//void STMFLASH_Write(FlashData_t *pBuffer,unsigned int resetTime)	
//{ 
//  unsigned int* address=(unsigned int*)pBuffer;
//  unsigned int WriteAddr=FLASH_USER_ADDRESS+resetTime*sizeof(FlashData_t);
//  unsigned int endaddr=WriteAddr+sizeof(FlashData_t);
//  
//  FLASH_Unlock();									//���� 
//  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
//  
//  while(WriteAddr<endaddr)//д����
//  {
//    if(FLASH_ProgramWord(WriteAddr,*address)!=FLASH_COMPLETE)//д������
//    { 
//      address=address;	//д���쳣
//    }
//		/*WriteAddr��������Ҫ��4��address��ָ��ֻ�ü�һ*/
//    WriteAddr+=MAX_SIZE;
//    address++;
//  }
//  
//  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
//  FLASH_Lock();//����
//} 

//void copyFlashData_tFromOther(FlashData_t* copy,FlashData_t* const reality)
//{
//	unsigned int* address=(unsigned int*)reality;
//  unsigned int* WriteAddr=(unsigned int*)copy;
//  unsigned int* endaddr=WriteAddr+sizeof(FlashData_t)/MAX_SIZE;
//	
//  while(WriteAddr<endaddr)//д����
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
//  while(WriteAddr<endaddr)//д����
//  {
//    *WriteAddr=STMFLASH_ReadWord(address);
//    WriteAddr++;
//    address+=MAX_SIZE;
//  }
//}

///*�Ѵ���Ļ����˽ṹ��Ĳ������浽��flashд��ṹ�塱��*/
//void WriteFlashData(AllPara_t robot,unsigned int resetTime)
//{
//	copyFlashData_tFromOther(&dataSave,&robot.sDta);
//  
//  STMFLASH_Write(&dataSave,allPara.resetTime);
//}


///*������resetTime���ṹ���ֵ��resetTimeȡ0 1 2 3*/
//void STMFLASH_Read(FlashData_t* temp,uint32_t resetTime)   	
//{
//  uint32_t baseAdd=FLASH_USER_ADDRESS+resetTime*sizeof(FlashData_t);
//  
//	copyFlashData_tAlongAddress(temp,baseAdd);
//}



/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/

