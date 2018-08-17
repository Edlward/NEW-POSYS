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

/*该结构体永远是最新更新的结构体*/
DataSave_t dataSave={0};
extern AllPara_t allPara;

#define FLASH_SAVE_ADDR 					0x08080000 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.

//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
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
  FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
  
  if(FLASH_EraseSector(STMFLASH_GetFlashSector(FLASH_SAVE_ADDR),VoltageRange_3)!=FLASH_COMPLETE) 
  {
    
  }
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
  FLASH_Lock();//上锁
} 


void STMFLASH_Write(DataSave_t *pBuffer,unsigned int resetTime)	
{ 
  unsigned int* address=(unsigned int*)pBuffer;
  unsigned int WriteAddr=FLASH_SAVE_ADDR+resetTime*sizeof(DataSave_t);
  unsigned int endaddr=WriteAddr+sizeof(DataSave_t);
  
  FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
  
  while(WriteAddr<endaddr)//写数据
  {
    if(FLASH_ProgramWord(WriteAddr,*address)!=FLASH_COMPLETE)//写入数据
    { 
      address=address;	//写入异常
    }
		/*WriteAddr是整数，要加4，address是指针只用加一*/
    WriteAddr+=MAX_SIZE;
    address++;
  }
  
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
  FLASH_Lock();//上锁
} 

void copyDataSave_tFromOther(DataSave_t* copy,DataSave_t* const reality)
{
	unsigned int* address=(unsigned int*)reality;
  unsigned int* WriteAddr=(unsigned int*)copy;
  unsigned int* endaddr=WriteAddr+sizeof(DataSave_t)/MAX_SIZE;
	
  while(WriteAddr<endaddr)//写数据
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
	
  while(WriteAddr<endaddr)//写数据
  {
    *WriteAddr=STMFLASH_ReadWord(address);
    WriteAddr++;
    address+=MAX_SIZE;
  }
}

/*把传入的机器人结构体的参数保存到“flash写入结构体”里*/
void WriteFlashData(AllPara_t robot,unsigned int resetTime)
{
	copyDataSave_tFromOther(&dataSave,&robot.sDta);
  
  STMFLASH_Write(&dataSave,allPara.resetTime);
}


/*读出第resetTime个结构体的值，resetTime取0 1 2 3*/
void STMFLASH_Read(DataSave_t* temp,uint32_t resetTime)   	
{
  uint32_t baseAdd=FLASH_SAVE_ADDR+resetTime*sizeof(DataSave_t);
  
	copyDataSave_tAlongAddress(temp,baseAdd);
}

/*  
**获取未开发空间的前一个结构体的序列号
**resetTime 取 0 1 2...  
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

第一次上电→flash为空→刷新flash（allPara.resetTime为0）→出故障→在hardfault写flash第一个结构体（allPara.resetTime为0，flashdata.sDta.isReset=1）
→重启→allPara.resetTime为1→得到第0个结构体的值保存到flashdata（永远最新）→把flashdata的内容拷贝给allPara→flashdata的isReset为0，把flash写给第allPara.resetTime个结构体，为下次开电做准备（下次可以识别到flashdata的isReset为位0）
→如果再发生中断，在allPara.resetTime写保存数据的结构体（此时allPara.resetTime已加一）
************/


void SoftWareReset(void)
{
	/*计算重启次数*/
  FindResetTime();
  
  //如果flash被写过（曾经重启过）
  if(allPara.resetTime>0)
  {
    //得到最后一个结构体（更新dataSave）
    STMFLASH_Read(&dataSave,allPara.resetTime-1);
  }
  
  /*上电状态  分别对应第一次下程序和以后运行得出的结果*/
  if(allPara.resetTime==0||dataSave.isReset==0||allPara.resetTime>=500)
  {
    STMFLASH_ERASE();
		LED1_OFF;
		LED2_OFF;
		AllParaInit();
		ReadCharacters();
  }
  /*进了硬件中断后重启*/
  else
  {
		LED1_ON;
		LED2_ON;
    //将进硬件中断前的数据恢复
		copyDataSave_tFromOther(&allPara.sDta,&dataSave);
		
    /*写一个reset=set的*/
    dataSave.isReset=0;
    //再往下一个空位写一个结构体，便于下次开电识别
    STMFLASH_Write(&dataSave,allPara.resetTime);
		
		/*表示这次是重启程序*/
		allPara.resetFlag=1;
  }
}


