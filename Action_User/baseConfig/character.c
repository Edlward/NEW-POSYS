#include "character.h"
#include "flash.h"
#include "stm32f4xx_flash.h"

//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.
//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.
#define FLASH_SAVE_ADDR 					0x080A0000 	

extern AllPara_t allPara;

void CheckDafault(void)
{
	//���flash��û�д洢��Ϣ(��ƫ������ֵ)����ʹ��Ĭ����Ϣ
	
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


//�ϵ��flash���ȡ���µ���Ϣ
void ReadCharacters(void)
{
	uint32_t baseAdd=FLASH_SAVE_ADDR+sizeof(character_t);
	
	unsigned int* WriteAddr=(unsigned int*)(&allPara.sDta.para);
  unsigned int* endaddr=WriteAddr+sizeof(character_t)/4;
	
  while(WriteAddr<endaddr)//д����
  {
    *WriteAddr=*(volatile unsigned int*)(baseAdd);
    WriteAddr++;
    baseAdd+=4;
  }
	
	CheckDafault();
}

//�洢��flash��
void writeCharacters(void)
{
	character_t *pBuffer=&allPara.sDta.para;

  unsigned int* address=(unsigned int*)pBuffer;
  unsigned int WriteAddr=FLASH_SAVE_ADDR+sizeof(character_t);
  unsigned int endaddr=WriteAddr+sizeof(character_t);
  
  FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
  
  while(WriteAddr<endaddr)//д����
  {
    if(FLASH_ProgramWord(WriteAddr,*address)!=FLASH_COMPLETE)//д������
    { 
      address=address;	//д���쳣
    }
		/*WriteAddr��������Ҫ��4��address��ָ��ֻ�ü�һ*/
    WriteAddr+=4;
    address++;
  }
  
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
  FLASH_Lock();//����
}



