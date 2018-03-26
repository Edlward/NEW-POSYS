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
extern flashData_t flashData;

void AT_CMD_Judge(void);
void SetParaDefault(void);
void DataSend(void)
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


//	
	#ifdef TEST_SUMMER
	i=i;
//	USART_OUT_F((allPara.GYRO_Temperature[0]+allPara.GYRO_Temperature[1]+allPara.GYRO_Temperature[2])/3.f);
//	USART_OUT_F(allPara.GYROWithoutRemoveDrift[0][2]);
//	USART_OUT_F(allPara.GYROWithoutRemoveDrift[1][2]);
//	USART_OUT_F(allPara.GYROWithoutRemoveDrift[2][2]);
	USART_OUT_F(allPara.GYRO_Real[2]);
	USART_OUT_F(allPara.kalmanZ);
	USART_OUT_F(allPara.sDta.Result_Angle[2]);
	USART_OUT_F(allPara.sDta.GYRO_Bais[2]);
	USART_OUT_F(allPara.sDta.posx);
	USART_OUT_F(allPara.sDta.posy);
	USART_OUT_F(allPara.sDta.vell[0]);
	USART_OUT_F(allPara.sDta.vell[1]);
//	USART_OUT_F(allPara.isStatic);
	//USART_OUT(USART1,"%d\t%d\t%d",allPara.sDta.codeData[0],allPara.sDta.codeData[1],allPara.cpuUsage);
	USART_Enter();
	#else
	for(i=0;i<28;i++)
   USART_SendData(USART1,tdata[i]);
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
   USART_SendData(USART1,tdata[i]);

}

static char buffer[20];
static int bufferI=0;
extern flashData_t flashData;
static int atCommand=0;
void bufferInit(void){
  bufferI=0;
  for(int i=0;i<20;i++)
    buffer[i]=0;
}
void USART1_IRQHandler(void)
{
  uint8_t data;
  if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit( USART1,USART_IT_RXNE);
    data=USART_ReceiveData(USART1);
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
        //USART_OUT(USART1,"NO A\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(USART1);
  }
}

void AT_CMD_Judge(void){
  if((bufferI == 4) && strncmp(buffer, "AT\r\n", 4)==0)//AT    
	{
    bufferInit();
		SetFlag(START_COMPETE);
		USART_OUT(USART1,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AS\r\n", 4)==0)//AT    
	{
    bufferInit();
		SetFlag(STATIC_FORCE);
		USART_OUT(USART1,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AB\r\n", 4)==0)//AT    
	{
    bufferInit();
		SetFlag(~STATIC_FORCE);
		USART_OUT(USART1,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AR\r\n", 4)==0)//AT    
	{
    bufferInit();
		allPara.sDta.Result_Angle[2]=0.0;
		USART_OUT(USART1,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AQ\r\n", 4)==0)//AT    
	{
    bufferInit();
		allPara.sDta.Result_Angle[2]=0.0;
		allPara.sDta.posx=0.0;
		USART_OUT(USART1,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AH\r\n", 4)==0)//AT    
	{
		#ifdef TEST_SUMMER
    bufferInit();
		int i[2]={0};
		for(int j=60000;j<80000;j++)
		{
			i[j]=100;
			i[j]=i[j];
		}
		USART_OUT(USART1,"OK");
		#endif
	}
	else if((bufferI == 14) && strncmp(buffer, "AT+printData\r\n", 14)==0)
    atCommand=5;
  else if((bufferI == 10) && strncmp(buffer, "AT+reset\r\n", 10)==0)//AT    
    atCommand=7;
  else if((bufferI == 8) && strncmp(buffer, "AT+set\r\n", 8)==0)//AT    
    atCommand=8;
  else if((bufferI == 10) && strncmp(buffer, "AT+print\r\n", 10)==0)//AT 
    atCommand=9;
  else if((bufferI == 11) && strncmp(buffer, "AT+static\r\n", 11)==0)//AT  
    atCommand=10;
  else if((bufferI == 14) && strncmp(buffer, "AT+set", 6)==0)//AT    
    atCommand=11;
  else if((bufferI >= 10) && strncmp(buffer, "AT+setmin", 9)==0)//AT    
    atCommand=12;
	else if((bufferI >= 10) && strncmp(buffer, "AT+setvar", 9)==0)//设置方差
    atCommand=13;
  else if((bufferI == 10) && strncmp(buffer, "AT+G", 4)==0)//设置陀螺仪是否用标准的温飘数据 如AT+G1A11
    atCommand=14;
  else if((bufferI == 8) && strncmp(buffer, "AT+G", 4)==0)//设置陀螺仪测量范围 如AT+G10    
    atCommand=15;
  else if((bufferI == 14) && strncmp(buffer, "AT+G", 4)==0)//设置温度系数选择范围 如AT+G1A111111
    atCommand=16;
  else if((bufferI == 12) && strncmp(buffer, "AT+default\r\n", 12)==0)//AT    
    atCommand=17;
  else if((bufferI == 9) && strncmp(buffer, "AT+heat\r\n", 9)==0)//AT    
    atCommand=18;
  else if((bufferI == 11) && strncmp(buffer, "AT+noheat\r\n", 11)==0)//AT    
    atCommand=19;
//  else if((bufferI == 10) && strncmp(buffer, "AT+fault\r\n", 10)==0)//AT    
//    atCommand=20;
  else 
	{
    atCommand=666;
    bufferInit();
	}
  
}

void AT_CMD_Handle(void){
	float value=0.0f;
	int hardFaultMaker[2];
	switch(atCommand)
	{
		case 0:
			break;
		case 5:
			USART_OUT_F(allPara.GYRO_Temperature[0]);
			USART_OUT_F(allPara.GYRO_Temperature[1]);
			USART_OUT_F(allPara.GYRO_Temperature[0]);
			USART_OUT_F(allPara.sDta.GYRO_Aver[0]);
			USART_OUT_F(allPara.sDta.GYRO_Aver[1]);
			USART_OUT_F(allPara.sDta.GYRO_Aver[2]);
			USART_Enter();
			break;
		case 9:
			USART_OUT(USART1,"OK");
			TempTablePrintf();
			break;
		case 11:
			USART_OUT(USART1,"OK");
			union{
			float   val;
			uint8_t data[4];
			}temp_float;
			temp_float.data[0]=buffer[8];
			temp_float.data[1]=buffer[9];
			temp_float.data[2]=buffer[10];
			temp_float.data[3]=buffer[11];
			switch(buffer[6]){
				case 'x':
					SetPosX(temp_float.val);
					break;
				case 'y':
					SetPosY(temp_float.val);
					break;
				case 'a':
					SetAngle(temp_float.val);
					break;
			}
			break;
		//设置最小阈值
		case 12:
		  value = atof(buffer+10);
			switch(buffer[9]){
				case 'x':
					*(flashData.minValue)=value;
					break;
				case 'y':
					*((flashData.minValue)+1)=value;
					break;
				case 'z':
					*((flashData.minValue)+2)=value;
					break;
			}		
			USART_OUT(USART1,"writing\r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			PrintMinValue();
			break;
		//设置方差
		case 13:
		  value = atof(buffer+10);
			switch(buffer[9]){
				case 'x':
					*(flashData.varXYZ)=value;
					break;
				case 'y':
					*((flashData.varXYZ)+1)=value;
					break;
				case 'z':
					*((flashData.varXYZ)+2)=value;
					break;
			}		
			USART_OUT(USART1,"writing\r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			PrintVarXYZ();
			break;
			//设置陀螺仪是否用标准的温飘数据 如AT+G1A11
		case 14:
			USART_OUT(USART1,"OK");
			/*不结合之前的数据*/
			if(buffer[7]=='1')
				*(flashData.chartMode+(buffer[4]-'1')*AXIS_NUMBER+(buffer[6]-'1'))=1;
			/*结合之前的数据*/
			else if(buffer[7]=='0')
				*(flashData.chartMode+(buffer[4]-'1')*AXIS_NUMBER+(buffer[6]-'1'))=0;
			else
				USART_OUT(USART1,"mode allPara.sDta.flag error\r\n");
			USART_OUT(USART1,"writing \r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			PrintChartMode();
			break;
			//设置陀螺仪测量范围 如AT+G10
		case 15:
			USART_OUT(USART1,"OK");
			if(buffer[5]=='1')
				*(flashData.scaleMode+buffer[4]-'1')=1;
			else if(buffer[5]=='0')
				*(flashData.scaleMode+buffer[4]-'1')=0;
			else
				USART_OUT(USART1,"(flashData.scaleMode) allPara.sDta.flag error\r\n");
			USART_OUT(USART1,"writing\r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			PrintScaleMode();
			break;
		//	AT+G =  设置温度系数选择范围
		case 16:
			USART_OUT(USART1,"OK");
			for(int i=7;i<12;i++){
				if(buffer[i]=='1')
					*(flashData.chartSelect+(buffer[4]-'1')*AXIS_NUMBER*TEMP_SAMPLE_NUMBER+(buffer[6]-'1')*TEMP_SAMPLE_NUMBER+i-7)=1;
				else if(buffer[i]=='0')
					*(flashData.chartSelect+(buffer[4]-'1')*AXIS_NUMBER*TEMP_SAMPLE_NUMBER+(buffer[6]-'1')*TEMP_SAMPLE_NUMBER+i-7)=0;
				else
					USART_OUT(USART1,"select %d error\r\n",i-10);
			}
			USART_OUT(USART1,"writing\r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			PrintchartSelect();
			break;
		case 17:
			USART_OUT(USART1,"OK");
			SetParaDefault();
			break;
		case 18:
			USART_OUT(USART1,"OK");
			SetFlag(HEATING);
			break;
		case 19:
			USART_OUT(USART1,"OK");
			SetFlag(~HEATING);
			break;
		case 20:
			SetFlag(~HEATING);
			for(int i=80000;i<100000;i++)
				hardFaultMaker[i]=100;
		hardFaultMaker[0]=hardFaultMaker[0];
			break;
		default:
			USART_OUT(USART1,"error\r\n");
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
	int gyro=0;
  int axis=0;
  
  for(gyro=0;gyro<GYRO_NUMBER;gyro++)
  {
    for(axis=0;axis<AXIS_NUMBER;axis++)
    {
      *(flashData.chartMode+gyro*AXIS_NUMBER+axis)=1;
      for(int sample=0;sample<TEMP_SAMPLE_NUMBER;sample++)
      {
        *(flashData.chartSelect+gyro*AXIS_NUMBER*TEMP_SAMPLE_NUMBER+axis*TEMP_SAMPLE_NUMBER+sample)=1;
      }
    }
  }
  for(gyro=0;gyro<GYRO_NUMBER;gyro++)
  {
    *(flashData.scaleMode+gyro)=0;
  }
  
  for(int axis=0;axis<AXIS_NUMBER;axis++)
  {
    *(flashData.minValue+axis)=0.1;
  }
  
  for(int axis=0;axis<AXIS_NUMBER;axis++)
  {
    *(flashData.varXYZ+axis)=0.003;
  }
	Flash_Write(GetFlashArr(),TempTable_Num);
	TempTablePrintf();
}

