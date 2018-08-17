#include "config.h"
#include "self_math.h"
#include "usart.h"

extern AllPara_t allPara;
float safe_asin(float v)
{
	int error=0;
	if (isnan(v)) 
		error=1;
	else if (v >= 1.0f) 
		error=2;
	else if (v <= -1.0f) 
		error=3;
	
	if(error!=0)
	{
		uint32_t r_sp ;
			/*�жϷ����쳣ʱʹ��MSP����PSP*/		
		if(__get_PSP()!=0x00) //��ȡSP��ֵ
			r_sp = __get_PSP(); 
		else
			r_sp = __get_MSP(); 
		/*��Ϊ�����жϺ�����ջ֮�󣬶�ջָ����С0x10������ƽ�ƻ��������ܲ������ձ��ԣ�*/
		r_sp = r_sp+0x10;
		/*���ڷ���֪ͨ*/
		USART_OUT(USART_USED,"sinFault %d",error);
		char sPoint[2]={0};
		USART_OUT(USART_USED,"%s","0x");
		/*��ȡ�����쳣ʱ����ĵ�ַ*/
		for(int i=3;i>=-28;i--){
			Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
			USART_OUT(USART_USED,"%s",sPoint);
			if(i%4==0)
				USART_Enter();
		}
		/*���ͻس���*/
		USART_Enter();
		switch(error)
		{
			case 1:
				return 0.0;
			case 2:
				return 3.1415926/2;
			case 3:
				return -3.1415926/2;
		}
	}else
			return asin(v);
	return asin(v);
}

/**
  * @brief  �Ż���ķ����Ǻ���
  * @param  x: tan=x/y
  * @param  y:
  * @retval �õ������е�ֵ
  */
double safe_atan2(double x,double y)
{	
	int error=0;

		if (isnan(y)) 
	{ 
		error=1;
  }else if(isnan(x/y))
	{
		if(x>0)
			error=2;
		else if(x<0)
			error=3;
		else 
			error=4;
	}
	
	if(error!=0)
	{
		uint32_t r_sp ;
			/*�жϷ����쳣ʱʹ��MSP����PSP*/		
		if(__get_PSP()!=0x00) //��ȡSP��ֵ
			r_sp = __get_PSP(); 
		else
			r_sp = __get_MSP(); 
		/*��Ϊ�����жϺ�����ջ֮�󣬶�ջָ����С0x10������ƽ�ƻ��������ܲ������ձ��ԣ�*/
		r_sp = r_sp+0x10;
		/*���ڷ���֪ͨ*/
		USART_OUT(USART_USED,"tanFault %d",error);
		char sPoint[2]={0};
		USART_OUT(USART_USED,"%s","0x");
		/*��ȡ�����쳣ʱ����ĵ�ַ*/
		for(int i=3;i>=-28;i--){
			Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
			USART_OUT(USART_USED,"%s",sPoint);
			if(i%4==0)
				USART_Enter();
		}
		/*���ͻس���*/
		USART_Enter();
		switch(error)
		{
			case 1:
				return 0.0f;
			case 2:
				return  3.1415926/2.0; 
			case 3:
				return -3.1415926/2.0;
			case 4:
				return 0.0;
		}
	}else
		return atan2(x,y);
	return atan2(x,y);
}




int CalculateRealCrAndMean(float stdCr[AXIS_NUMBER],float mean[AXIS_NUMBER]){
  static float IAE_st[AXIS_NUMBER][200]={0.f};  
  static float data[AXIS_NUMBER][200]={0.f};    
  static int ignore=0;
  ignore++;
  
  int axis=0;
  
    for(axis=0;axis<AXIS_NUMBER;axis++)
    {
      mean[axis]=mean[axis]-data[axis][0]/200;
      memcpy(data[axis],data[axis]+1,796);
      data[axis][199]=allPara.sDta.GYRO_Aver[axis];
      memcpy(IAE_st[axis],IAE_st[axis]+1,796);
      mean[axis]=mean[axis]+data[axis][199]/200;
      IAE_st[axis][199]=allPara.sDta.GYRO_Aver[axis]-mean[axis];
    }
  
  if(ignore<400)
    return 0;
  else
    ignore=400;
  
    for(axis=0;axis<AXIS_NUMBER;axis++){
      stdCr[axis]=0;
      for(int i=0;i<200;i++)
      {
        stdCr[axis]=stdCr[axis]+IAE_st[axis][i]*IAE_st[axis][i];
      }
      stdCr[axis]=__sqrtf(stdCr[axis]/200.0f);
    }
  
  return 1;
}



float FilterVell(float newValue)
{
	static float datas[10]={0.f};
	float temp=0.f;
	
	for(int i=0;i<10-1;i++)
		datas[i]=datas[i+1];
	
	datas[10-1]=newValue;
	
	for(int i=0;i<10;i++)
		temp+=datas[i]/10;
	
	return temp;
}



#define STATIC_ARRAY_NUM	5

uint16_t FindMax(uint16_t codes[STATIC_ARRAY_NUM])
{
	uint16_t Max=codes[0]; 
  for(int i=1;i<STATIC_ARRAY_NUM;i++)
	{
		if(codes[i]>Max) Max=codes[i];
	}
	return Max;
}

uint16_t FindMin(uint16_t codes[STATIC_ARRAY_NUM])
{
	uint16_t Min=codes[0]; 
  for(int i=1;i<STATIC_ARRAY_NUM;i++)
	{
		if(codes[i]<Min) Min=codes[i];
	}
	return Min;
}

void JudgeStatic(void)
{
	static uint16_t codes0[STATIC_ARRAY_NUM];
	static uint16_t codes1[STATIC_ARRAY_NUM];
	int difCode[2]={0,0};
	static uint32_t count=0;
	
//	static int staticCount=0;
	for(int i=0;i<STATIC_ARRAY_NUM-1;i++)
	{
		codes0[i]=codes0[i+1];
		codes1[i]=codes1[i+1];
	}
	codes0[STATIC_ARRAY_NUM-1]=allPara.sDta.codeData[0];
	codes1[STATIC_ARRAY_NUM-1]=allPara.sDta.codeData[1];

	difCode[0]=FindMax(codes0)-FindMin(codes0);
	difCode[1]=FindMax(codes1)-FindMin(codes1);
	
	#ifdef TLE5012_USED
		difCode[0]=difCode[0]-(difCode[0]>16384)*32768;
		difCode[0]=difCode[0]+(difCode[0]<-16384)*32768;
		
		difCode[1]=difCode[1]-(difCode[1]>16384)*32768;
		difCode[1]=difCode[1]+(difCode[1]<-16384)*32768;
		
		if(abs(difCode[0])<5&&abs(difCode[1])<5&&fabs(allPara.GYRO_Real[2])<0.20)
			count++;
		else
			count=0;
	#else
		difCode[0]=difCode[0]-(difCode[0]>2048)*4096;
		difCode[0]=difCode[0]+(difCode[0]<-2048)*4096;
		
		difCode[1]=difCode[1]-(difCode[1]>2048)*4096;
		difCode[1]=difCode[1]+(difCode[1]<-2048)*4096;
		
		if(abs(difCode[0])<3&&abs(difCode[1])<3&&fabs(allPara.GYRO_Real[2])<0.20)
			count++;
		else
			count=0;
	#endif
	
	if(count>20)
		SetFlag(STATIC_FORCE);
	else
		SetFlag(~STATIC_FORCE);
	
}
