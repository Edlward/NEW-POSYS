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
DataSave_t dataSave={0};
extern AllPara_t allPara;

#define FLASH_SAVE_ADDR 					0x08080000 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.

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
  delay_ms(1);
  if(FLASH_EraseSector(STMFLASH_GetFlashSector(FLASH_SAVE_ADDR),VoltageRange_4)!=FLASH_COMPLETE) 
  {
    delay_ms(1);
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
      //USART_OUT(USART_USED,"extend!\r\n");
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
//		ReadCharacters();
    STMFLASH_ERASE();
//		writeCharacters();
		LED1_OFF;
		LED2_OFF;
		AllParaInit();
  }
  /*����Ӳ���жϺ�����*/
  else
  {
		LED1_ON;
		LED2_ON;
    //����Ӳ���ж�ǰ�����ݻָ�
		copyDataSave_tFromOther(&allPara.sDta,&dataSave);
		
    /*дһ��reset=set��*/
    dataSave.isReset=0;
    //������һ����λдһ���ṹ�壬�����´ο���ʶ��
    STMFLASH_Write(&dataSave,allPara.resetTime);
//		writeCharacters();
		/*��ʾ�������������*/
		allPara.resetFlag=1;
  }
}


