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
#define FLASH_USER_ADDRESS 0x08040000   //FLASH��ʼ��ַ
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static uint8_t  flashdata[160*(TempTable_max-TempTable_min)];  //��flash��ȡ��������
static float    *Result;
static uint32_t *countnum;
static uint8_t  flag=0;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/**
  * @brief  FLASH����
	*
  * @param  None
  * @retval None
  */
static void Flash_Encryp(void)
{
	#ifdef FLASH_ENCRYP
	
	/* �ж�FLASH�Ƿ񱻱��� */
	if(SET != FLASH_OB_GetRDP())
  {
		    /* ���������� */
        FLASH_Unlock();
        FLASH_OB_Unlock();
		    /* �ȼ�0���������ȼ�1������������ȼ�2����������� */
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
  * @brief  ��FLASH��д������
	*
  * @param  data :  �洢���ݵ�ָ��
  * @param  len  :  д�������������λ��byte
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
  * @brief  ��FLASH�е����ݸĳ�0
	*
  * @param  len  :  ��д������������λ��byte
  * @retval None
  */
void Flash_Zero(uint32_t len)
{
  uint32_t count;
	FLASH_Unlock();
	/*����־λ��д1*/
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	/*���ǵ�405оƬ�õ���3.3V,����ѡ��VoltageRange_3*/
	FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3);
	FLASH_WaitForLastOperation();
	for(count=0;count<len;count++)
	{
		FLASH_ProgramByte(FLASH_USER_ADDRESS+count,0x00);
	}
	FLASH_Lock();
}

/**
  * @brief  ��FLASH�ж�ȡ����
	*
  * @param  data :  �洢���ݵ�ָ��
  * @param  len  :  ��ȡ������������λ��byte
  * @retval None
  */
void Flash_Read(uint8_t *data,uint32_t len)
{
	uint32_t i;
	for(i=0;i<len;i++)
	//һ���ֽ�һ���ֽڵؽ��и�ֵ
	  *(data+i)= *((uint8_t *)(FLASH_USER_ADDRESS+i));
}
/**
  * @brief  ��ʼ��FLASH
	*
  * @param  None
  * @retval None
  */
void Flash_Init(void)
{
	/* ��ȡFLASH�б�������ݣ�������浽�ڴ�(RAM)�� */
	//static uint8_t  flashdata[160*(TempTable_max-TempTable_min)];  //��flash��ȡ��������
	Flash_Read(flashdata,16*10*(TempTable_max-TempTable_min));  //55-10 
	/* һ���¶ȿ��Էֳ�10�ȷ֣�ÿ0.1�ȶ�Ӧ����оƬ��
	  3�����ٶȣ�1������ֵ��һ��16�ֽ� */
	/* �ָ����ݶΣ�����Ưֵ�����ֵ�ֿ� */
	Result=(float *)flashdata;
	//һ��160*45���ֽ�,ǰ120*45�ǽ��ٶ�ֵ,�˴�*30,
	//����Ϊ�Ѿ�ת����32λ
	countnum=(uint32_t *)(flashdata)+30*(TempTable_max-TempTable_min);
	
	/* ����Flash���� */
	Flash_Encryp();
}
/**
  * @brief  ��ȡ��FLASH��õ������ݵ�ָ��
	*
  * @param  None
  * @retval flashdata : ָ��flash���ݵ�ָ��
  */
uint8_t *GetFlashArr(void)
{
	return flashdata;
}
/**
  * @brief  ��FLASH��õ������ݷ��������֣���һ����Ϊ���������ڶ�����Ϊ
  *         ���������ú������ڻ����������ָ��
  * @param  None
  * @retval Result : ָ��flash��������ָ��
  */
float *GetResultArr(void)
{
	return Result;
}
/**
  * @brief  ��FLASH��õ������ݷ��������֣���һ����Ϊ���������ڶ�����Ϊ
  *         ���������ú������ڻ�ü�������ָ��
  * @param  None
  * @retval Result : ָ��flash��������ָ��
  */
uint32_t *GetCountArr(void)
{
	return countnum;
}
/**
  * @brief  ���FLASH�Ƿ���Ҫ���µı�־λ
  *         
  * @param  None
  * @retval flag : flash���µı�־λ
  */
uint8_t GetFlashUpdataFlag(void)
{
	return flag;
}
/**
  * @brief  ����FLASH���±�־λ��ֵ
  *         
  * @param  val  : �������±�־λ��ֵ
  * @retval None
  */
void  SetFlashUpdateFlag(uint8_t val)
{
	flag=val;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/

