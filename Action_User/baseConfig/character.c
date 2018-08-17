#include "character.h"
#include "flash.h"
#include "stm32f4xx_flash.h"

//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.
//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
#define FLASH_SAVE_ADDR 					0x080d0000 	

extern AllPara_t allPara;

void CheckDafault(void)
{
	//如果flash里没有存储信息(或偏离正常值)，则使用默认信息
	
	if(allPara.sDta.para.rWheelNo1>25.6||allPara.sDta.para.rWheelNo1<25.0||isnan(allPara.sDta.para.rWheelNo1))
		allPara.sDta.para.rWheelNo1=25.4;
	
	if(allPara.sDta.para.rWheelNo2>25.6||allPara.sDta.para.rWheelNo2<25.0||isnan(allPara.sDta.para.rWheelNo2))
		allPara.sDta.para.rWheelNo2=25.4;
	
	if(((allPara.sDta.para.calibrationFactor>132.0||allPara.sDta.para.calibrationFactor<130.0)&&(allPara.sDta.para.gyroScale==250))||isnan(allPara.sDta.para.calibrationFactor))
	{
		allPara.sDta.para.calibrationFactor=131.0;
		allPara.sDta.para.gyroScale=250;
	}
	else if(((allPara.sDta.para.calibrationFactor>66.0||allPara.sDta.para.calibrationFactor<65.0)&&(allPara.sDta.para.gyroScale==500))||isnan(allPara.sDta.para.calibrationFactor))
	{
		allPara.sDta.para.calibrationFactor=65.5;
		allPara.sDta.para.gyroScale=500;
	}
	else if(((allPara.sDta.para.calibrationFactor>33.3||allPara.sDta.para.calibrationFactor<32.3)&&(allPara.sDta.para.gyroScale==1000))||isnan(allPara.sDta.para.calibrationFactor))
	{
		allPara.sDta.para.calibrationFactor=32.8;
		allPara.sDta.para.gyroScale=1000;
	}
	else
		allPara.sDta.para.calibrationFactor=131.0;
	
	if(allPara.sDta.para.gyroScale!=250&&allPara.sDta.para.gyroScale!=500&&allPara.sDta.para.gyroScale!=1000)
		allPara.sDta.para.gyroScale=250;
	
	if(allPara.sDta.para.angleWheelError>1.0||allPara.sDta.para.angleWheelError<-1.0||isnan(allPara.sDta.para.angleWheelError))
		allPara.sDta.para.angleWheelError=0.0;
	
}


//上电从flash里读取最新的信息
void ReadCharacters(void)
{
	uint32_t baseAdd=FLASH_SAVE_ADDR+sizeof(character_t);
	
	unsigned int* WriteAddr=(unsigned int*)(&allPara.sDta.para);
  unsigned int* endaddr=WriteAddr+sizeof(character_t)/4;
	
  while(WriteAddr<endaddr)//写数据
  {
    *WriteAddr=*(volatile unsigned int*)(baseAdd);
    WriteAddr++;
    baseAdd+=4;
  }
	
	CheckDafault();
}

//存储到flash里
void writeCharacters(void)
{
	character_t *pBuffer=&allPara.sDta.para;

  unsigned int* address=(unsigned int*)pBuffer;
  unsigned int WriteAddr=FLASH_SAVE_ADDR;
  unsigned int endaddr=WriteAddr+sizeof(character_t);
  
  FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
  
  while(WriteAddr<endaddr)//写数据
  {
    if(FLASH_ProgramWord(WriteAddr,*address)!=FLASH_COMPLETE)//写入数据
    { 
      address=address;	//写入异常
    }
		/*WriteAddr是整数，要加4，address是指针只用加一*/
    WriteAddr+=4;
    address++;
  }
  
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
  FLASH_Lock();//上锁
}



