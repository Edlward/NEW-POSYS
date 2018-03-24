/**
******************************************************************************
* @file     
* @author  qiaozhijian
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
#include "DataRecover.h"
#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "string.h"
#include "usart.h"
#include "config.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

/*�ýṹ����Զ�����¸��µĽṹ��*/
DataSave_t dataSave;
extern AllPara_t allPara;

//FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes  

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


//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
unsigned int STMFLASH_ReadWord(unsigned int faddr)
{
  return *(volatile unsigned int*)faddr; 
}  

float STMFLASH_ReadFloat(unsigned int faddr)
{
  return *(volatile float*)faddr; 
}  


void STMFLASH_ERASE(void)	
{ 
  FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
  
  if(FLASH_EraseSector(STMFLASH_GetFlashSector(FLASH_SAVE_ADDR),VoltageRange_3)!=FLASH_COMPLETE) 
  {
    
  }
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
  FLASH_Lock();//����
} 


void STMFLASH_Write(DataSave_t *pBuffer,unsigned int resetTime)	
{ 
  unsigned int* address=(unsigned int*)pBuffer;
  unsigned int WriteAddr=FLASH_SAVE_ADDR+resetTime*sizeof(DataSave_t);
  unsigned int endaddr=WriteAddr+sizeof(DataSave_t);
  
  FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
  
  while(WriteAddr<endaddr)//д����
  {
    if(FLASH_ProgramWord(WriteAddr,*address)!=FLASH_COMPLETE)//д������
    { 
      address=address;	//д���쳣
    }
		/*WriteAddr��������Ҫ��4��address��ָ��ֻ�ü�һ*/
    WriteAddr+=MAX_SIZE;
    address++;
  }
  
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
  FLASH_Lock();//����
} 

void copyDataSave_tFromOther(DataSave_t* copy,DataSave_t* const reality)
{
	unsigned int* address=(unsigned int*)reality;
  unsigned int* WriteAddr=(unsigned int*)copy;
  unsigned int* endaddr=WriteAddr+sizeof(DataSave_t)/MAX_SIZE;
	
  while(WriteAddr<endaddr)//д����
  {
    *WriteAddr=*address;
    WriteAddr++;
    address++;
  }
}

void copyDataSave_tAlongAddress(DataSave_t* copy,uint32_t address)
{
  unsigned int* WriteAddr=(unsigned int*)copy;
  unsigned int* endaddr=WriteAddr+sizeof(DataSave_t)/MAX_SIZE;
	
  while(WriteAddr<endaddr)//д����
  {
    *WriteAddr=STMFLASH_ReadWord(address);
    WriteAddr++;
    address+=MAX_SIZE;
  }
}

/*�Ѵ���Ļ����˽ṹ��Ĳ������浽��flashд��ṹ�塱��*/
void WriteFlashData(AllPara_t robot,unsigned int resetTime)
{
	copyDataSave_tFromOther(&dataSave,&robot.sDta);
  
  STMFLASH_Write(&dataSave,allPara.resetTime);
}


/*������resetTime���ṹ���ֵ��resetTimeȡ0 1 2 3*/
void STMFLASH_Read(DataSave_t* temp,uint32_t resetTime)   	
{
  uint32_t baseAdd=FLASH_SAVE_ADDR+resetTime*sizeof(DataSave_t);
  
	copyDataSave_tAlongAddress(temp,baseAdd);
}

/*  
**��ȡδ�����ռ��ǰһ���ṹ������к�
**resetTime ȡ 0 1 2...  
*/
void FindResetTime(void)
{
  allPara.resetTime=0;
  
  DataSave_t flashdata;
	
  STMFLASH_Read(&flashdata,allPara.resetTime);
  
  while(flashdata.isReset!=0xffffffff)
  {
    allPara.resetTime++;
    STMFLASH_Read(&flashdata,allPara.resetTime);
    if(allPara.resetTime>=500)
    {
      //USART_OUT(USART1,"extend!\r\n");
      return;
    }
    
    //printf("%f\t%f\r\n",flashdata.posx,flashdata.posy);
  }
}
/*************
softReset

��һ���ϵ��flashΪ�ա�ˢ��flash��allPara.resetTimeΪ0���������ϡ���hardfaultдflash��һ���ṹ�壨allPara.resetTimeΪ0��flashdata.sDta.isReset=1��
��������allPara.resetTimeΪ1���õ���0���ṹ���ֵ���浽flashdata����Զ���£�����flashdata�����ݿ�����allPara��flashdata��isResetΪ0����flashд����allPara.resetTime���ṹ�壬Ϊ�´ο�����׼�����´ο���ʶ��flashdata��isResetΪλ0��
������ٷ����жϣ���allPara.resetTimeд�������ݵĽṹ�壨��ʱallPara.resetTime�Ѽ�һ��
************/


void SoftWareReset(void)
{
	/*������������*/
  FindResetTime();
  
  //���flash��д����������������
  if(allPara.resetTime>0)
  {
    //�õ����һ���ṹ�壨����dataSave��
    STMFLASH_Read(&dataSave,allPara.resetTime-1);
  }
  
  /*�ϵ�״̬  �ֱ��Ӧ��һ���³�����Ժ����еó��Ľ��*/
  if(allPara.resetTime==0||dataSave.isReset==0||allPara.resetTime>=500)
  {
    STMFLASH_ERASE();
  }
  /*����Ӳ���жϺ�����*/
  else
  {
    //����Ӳ���ж�ǰ�����ݻָ�
		copyDataSave_tFromOther(&allPara.sDta,&dataSave);
		
    /*дһ��reset=set��*/
    dataSave.isReset=0;
    //������һ����λдһ���ṹ�壬�����´ο���ʶ��
    STMFLASH_Write(&dataSave,allPara.resetTime);
		
		/*��ʾ�������������*/
		allPara.resetFlag=1;
  }
}


