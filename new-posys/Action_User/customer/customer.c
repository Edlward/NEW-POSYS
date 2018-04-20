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
*
* 
******************************************************************************
*/ 
/* Includes -------------------------------------------------------------------*/

#include "config.h"

extern AllPara_t allPara;

extern double lowpass;
void AT_CMD_Judge(void);
void SetParaDefault(void);
void DataSend(void)
{
	int i;
	uint8_t tdata[DMA_SEND_SIZE];
  union{
		float   val;
		uint8_t data[4];
	}valSend;
	
  tdata[0]=0x0d;
  tdata[1]=0x0a;
  tdata[DMA_SEND_SIZE-2]=0x0a;
  tdata[DMA_SEND_SIZE-1]=0x0d;
	#ifdef AUTOCAR 
	
	valSend.val=(float)allPara.sDta.Result_Angle[2];
  memcpy(tdata+2,valSend.data,4);
	
	valSend.val=(float)allPara.sDta.vellx;
  memcpy(tdata+6,valSend.data,4);
	
	valSend.val=(float)allPara.sDta.velly;
  memcpy(tdata+10,valSend.data,4);
	
	valSend.val=(float)allPara.sDta.posx;
  memcpy(tdata+14,valSend.data,4);
	 
	valSend.val=(float)allPara.sDta.posy;
  memcpy(tdata+18,valSend.data,4);
	 
	valSend.val=(float)allPara.GYRO_Real[2];
  memcpy(tdata+22,valSend.data,4);
	
	#else
	valSend.val=(float)allPara.sDta.Result_Angle[2];
  memcpy(tdata+2,valSend.data,4);
	
	valSend.val=(float)allPara.sDta.vellx;
  memcpy(tdata+6,valSend.data,4);
	
	valSend.val=(float)allPara.sDta.velly;
  memcpy(tdata+10,valSend.data,4);
	
//	valSend.val=(float)allPara.sDta.GYRO_Bais[2];
//  memcpy(tdata+10,valSend.data,4);

	valSend.val=(float)allPara.sDta.posx;
  memcpy(tdata+14,valSend.data,4);
	 
	valSend.val=(float)allPara.sDta.posy;
  memcpy(tdata+18,valSend.data,4);
	 
	valSend.val=(float)allPara.GYRO_Real[2];
  memcpy(tdata+22,valSend.data,4);
	
//	valSend.val=(float)allPara.sDta.GYRO_Aver[1];
//  memcpy(tdata+26,valSend.data,4);
//	 
//	valSend.val=(float)allPara.sDta.GYRO_Aver[2];
//  memcpy(tdata+30,valSend.data,4);
	valSend.val=(float)allPara.sDta.codeData[0];
  memcpy(tdata+26,valSend.data,4);
	 
	valSend.val=(float)allPara.sDta.codeData[1];
  memcpy(tdata+30,valSend.data,4);
	#endif

//	
	#ifdef TEST_SUMMER
	i=i;
//	USART_OUT_F((allPara.GYRO_Temperature[0]+allPara.GYRO_Temperature[1]+allPara.GYRO_Temperature[2])/3.f);
//	USART_OUT_F(allPara.GYROWithoutRemoveDrift[0][2]);
//	USART_OUT_F(allPara.GYROWithoutRemoveDrift[1][2]);
//	USART_OUT_F(allPara.GYROWithoutRemoveDrift[2][2]);
//	for(int i=0;i<3;i++)
//		USART_OUT_F(allPara.GYROWithoutRemoveDrift[i][2]);

//	for(int i=0;i<3;i++)
//	{
//		USART_OUT_F(allPara.sDta.GYRO_TemperatureAim[i]);
//		USART_OUT_F(allPara.GYRO_Temperature[i]);
//	}
//	USART_OUT_F(allPara.sDta.GYRO_Aver[2]);
//	USART_OUT_F(lowpass);
	USART_OUT_F(allPara.sDta.Result_Angle[2]);
	USART_OUT_F(allPara.sDta.GYRO_Bais[2]);
//	USART_OUT_F(allPara.GYRO_Real[2]);
	USART_OUT_F(allPara.sDta.posx);
	USART_OUT_F(allPara.sDta.posy);
//	USART_OUT_F(allPara.sDta.vell[0]);
//	USART_OUT_F(allPara.sDta.vell[1]);
//	USART_OUT_F(allPara.isStatic);
	//USART_OUT(SEND_USART,"%d\t%d\t%d",allPara.sDta.codeData[0],allPara.sDta.codeData[1],allPara.cpuUsage);
	USART_Enter();
	#else
	
	for(i=0;i<DMA_SEND_SIZE;i++)
   USART_SendDataToDMA(tdata[i]);

	#endif
}
void debugsend2(float a,float b,float c,float d,float e)
{
	USART_OUT_F(a);
	USART_OUT_F(b);
	USART_OUT_F(c);
//	USART_OUT_F(d);
//	USART_OUT_F(e);
	USART_Enter();
}
void debugsend(float a,float b,float c,float d,float e,float f)
{
	int i;
	uint8_t tdata[28];
  union{
		float   val;
		uint8_t data[4];
	}valSend;
	
  tdata[0]=0x0d;
  tdata[1]=0x0a;
  tdata[26]=0x0a;
  tdata[27]=0x0d;
	
	valSend.val=a;
  memcpy(tdata+2,valSend.data,4);
	
	valSend.val=b;
  memcpy(tdata+6,valSend.data,4);
	
	valSend.val=c;
  memcpy(tdata+10,valSend.data,4);
	
	valSend.val=d;
  memcpy(tdata+14,valSend.data,4);
	 
	valSend.val=e;
  memcpy(tdata+18,valSend.data,4);
	 
	valSend.val=f;
  memcpy(tdata+22,valSend.data,4);
	
	for(i=0;i<28;i++)
   USART_SendData(SEND_USART,tdata[i]);

}

void ReportHardFault(void)
{
	USART_OUT(SEND_USART,"HF");
}

static char buffer[20];
static int bufferI=0;

static int atCommand=0;
void bufferInit(void){
  bufferI=0;
  for(int i=0;i<20;i++)
    buffer[i]=0;
}
#ifdef 0
void USART3_IRQHandler(void)
#else
void USART1_IRQHandler(void)
#endif
{
  uint8_t data;
  if(USART_GetITStatus(SEND_USART,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit(SEND_USART,USART_IT_RXNE);
    data=USART_ReceiveData(SEND_USART);
    buffer[bufferI]=data;
    bufferI++;
		if(bufferI>=20)
			bufferI=0;
//		if(data=='P')
//		{
//		atCommand=20;
//		}
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_CMD_Judge();
    }else{
      if(buffer[0]!='A'){
        bufferInit();
        //USART_OUT(SEND_USART,"NO A\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(SEND_USART);
  }
}
void AT_CMD_Judge(void){
	
	union{
		uint8_t data[4];
		float value;
	}convert_u;
	
  if((bufferI == 4) && strncmp(buffer, "AT\r\n", 4)==0)//AT    
	{
    bufferInit();
		SetFlag(START_COMPETE);
		USART_OUT(SEND_USART,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AS\r\n", 4)==0)//AT    
	{
    bufferInit();
		SetFlag(STATIC_FORCE);
		USART_OUT(SEND_USART,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AB\r\n", 4)==0)//AT    
	{
    bufferInit();
		SetFlag(~STATIC_FORCE);
		USART_OUT(SEND_USART,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AR\r\n", 4)==0)//AT    
	{
    bufferInit();
		allPara.sDta.Result_Angle[2]=0.0;
		USART_OUT(SEND_USART,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AQ\r\n", 4)==0)//AT    
	{
    bufferInit();
		allPara.sDta.Result_Angle[2]=0.0;
		allPara.sDta.posx=0.0;
		USART_OUT(SEND_USART,"OK");
	}
	else if((bufferI == 8) && strncmp(buffer, "AX", 2)==0)//AT    
	{
		convert_u.data[0]=*(buffer+2);
		convert_u.data[1]=*(buffer+3);
		convert_u.data[2]=*(buffer+4);
		convert_u.data[3]=*(buffer+5);
		allPara.sDta.posx=convert_u.value;
    bufferInit();
		USART_OUT(SEND_USART,"OK");
	}
	else if((bufferI == 8) && strncmp(buffer, "AY", 2)==0)//AT    
	{
		convert_u.data[0]=*(buffer+2);
		convert_u.data[1]=*(buffer+3);
		convert_u.data[2]=*(buffer+4);
		convert_u.data[3]=*(buffer+5);
		allPara.sDta.posy=convert_u.value;
    bufferInit();
		USART_OUT(SEND_USART,"OK");
	}
	else if((bufferI == 8) && strncmp(buffer, "AA", 2)==0)//AT    
	{
		convert_u.data[0]=*(buffer+2);
		convert_u.data[1]=*(buffer+3);
		convert_u.data[2]=*(buffer+4);
		convert_u.data[3]=*(buffer+5);
		allPara.sDta.Result_Angle[2]=convert_u.value;
    bufferInit();
		USART_OUT(SEND_USART,"OK");
	}
	else if((bufferI == 14) && strncmp(buffer, "AT+printData\r\n", 14)==0)
    atCommand=5;
  else if((bufferI == 11) && strncmp(buffer, "AT+noheat\r\n", 11)==0)//AT    
	{
    bufferInit();
		SetFlag(~HEATING);
		USART_OUT(SEND_USART,"OK");
	}
  else if((bufferI == 7) && strncmp(buffer, "AT+hf\r\n", 11)==0)//AT    
	{
    bufferInit();
		int a[2];
		for(int i=10000;i<80000;i++)
		{
			a[i]=100;
			a[i]=a[i];
		}
		USART_OUT(SEND_USART,"OK");
	}
  else 
	{
    atCommand=666;
    bufferInit();
	}
  
}

void AT_CMD_Handle(void){
	
	switch(atCommand)
	{
		case 0:
			break;

		default:
			USART_OUT(SEND_USART,"error\r\n");
			break;
	}
	atCommand=0;
	bufferInit();
}

void SetFlag(int val){
  switch(val){
  case START_COMPETE:
    allPara.sDta.flag|=START_COMPETE;
    break;
  case ~START_COMPETE:
    allPara.sDta.flag&=~START_COMPETE;
    break;
	case HEATING:
    allPara.sDta.flag|=HEATING;
		break;
  case ~HEATING:
    allPara.sDta.flag&=~HEATING;
    break;
	case STATIC_FORCE:
    allPara.sDta.flag|=STATIC_FORCE;
		break;
  case ~STATIC_FORCE:
    allPara.sDta.flag&=~STATIC_FORCE;
    break;
  }
}

void SetParaDefault(void)
{

}

