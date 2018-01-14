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
	
	valSend.val=(float)allPara.Result_Angle[2];
  memcpy(tdata+2,valSend.data,4);
	
	valSend.val=(float)allPara.Result_Angle[0];
  memcpy(tdata+6,valSend.data,4);
	
	valSend.val=(float)allPara.Result_Angle[1];
  memcpy(tdata+10,valSend.data,4);
	
	valSend.val=(float)allPara.posx;
  memcpy(tdata+14,valSend.data,4);
	 
	valSend.val=(float)allPara.posy;
  memcpy(tdata+18,valSend.data,4);
	 
	valSend.val=(float)allPara.GYRO_Real[2];
  memcpy(tdata+22,valSend.data,4);
	
	//debugsend2(allPara.Result_Angle[2],allPara.posx,allPara.posy,0,0);
	for(i=0;i<28;i++)
   USART_SendData(USART1,tdata[i]);
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
	#define ADAD
	#ifdef ADAD
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
	#else
	USART_OUT_F(a);
	USART_OUT_F(b);
	USART_OUT_F(c);
	USART_OUT_F(d);
	USART_OUT_F(e);
	USART_OUT_F(f);
	USART_Enter();
	#endif
}

static char buffer[20];
static int bufferI=0;
extern flashData_t flashData;
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
		if(bufferI==20)
			bufferI=0;
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_CMD_Judge();
    }else{
      if(buffer[0]!='A'){
        bufferInit();
        USART_OUT(USART1,"NO A\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(USART1);
  }
}

static int atCommand=0;
void AT_CMD_Judge(void){
  if((bufferI == 4) && strncmp(buffer, "AT\r\n", 4)==0)//AT    
    atCommand=2;
  else if((bufferI == 10) && strncmp(buffer, "AT+begin\r\n", 10)==0)//AT    
    atCommand=2;
	else if((bufferI == 13) && strncmp(buffer, "AT+stopSend\r\n", 13)==0)//none
    atCommand=3;
	else if((bufferI == 14) && strncmp(buffer, "AT+beginSend\r\n", 14)==0)//none
    atCommand=4;
	else if((bufferI == 14) && strncmp(buffer, "AT+printData\r\n", 14)==0)
    atCommand=5;
  else if((bufferI == 9) && strncmp(buffer, "AT+stop\r\n", 9)==0)//AT    
    atCommand=6;
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
  else if((bufferI == 9) && strncmp(buffer, "AT+heat\r\n", 12)==0)//AT    
    atCommand=18;
  else if((bufferI == 11) && strncmp(buffer, "AT+noheat\r\n", 12)==0)//AT    
    atCommand=19;
  else 
    atCommand=666;
  
}

void AT_CMD_Handle(void){
	float value=0.0f;
	switch(atCommand)
	{
		case 0:
			break;
		case 1:
			USART_OUT(USART1,"OK\r\n");
			break;
		case 2:
			SetCommand(ACCUMULATE);
			USART_OUT(USART1,"OK\r\n");
			break;
		case 3:
			USART_OUT(USART1,"OK\r\n");
			break;
		case 4:
			USART_OUT(USART1,"OK\r\n");
			break;
		case 5:
			USART_OUT_F(allPara.GYRO_Temperature[0]);
			USART_OUT_F(allPara.GYRO_Temperature[1]);
			USART_OUT_F(allPara.GYRO_Temperature[0]);
			USART_OUT_F(allPara.GYRO_Aver[0]);
			USART_OUT_F(allPara.GYRO_Aver[1]);
			USART_OUT_F(allPara.GYRO_Aver[2]);
			USART_Enter();
			break;
		case 6:
			SetCommand(~ACCUMULATE);
			USART_OUT(USART1,"OK\r\n");
			break;
		case 7:
			USART_OUT(USART1,"OK\r\n");
			SetCommand(CORRECT);
			break;
		case 8:
			SetCommand(~CORRECT);
			USART_OUT(USART1,"OK\r\n");
			break;
		case 9:
			USART_OUT(USART1,"OK\r\n");
			TempTablePrintf();
			break;
		case 10:
			USART_OUT(USART1,"OK\r\n");
			SetCommand(STATIC);
			break;
		case 11:
			USART_OUT(USART1,"OK\r\n");
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
			USART_OUT(USART1,"OK\r\n");
			/*不结合之前的数据*/
			if(buffer[7]=='1')
				*(flashData.chartMode+(buffer[4]-'1')*AXIS_NUMBER+(buffer[6]-'1'))=1;
			/*结合之前的数据*/
			else if(buffer[7]=='0')
				*(flashData.chartMode+(buffer[4]-'1')*AXIS_NUMBER+(buffer[6]-'1'))=0;
			else
				USART_OUT(USART1,"mode command error\r\n");
			USART_OUT(USART1,"writing \r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			PrintChartMode();
			break;
			//设置陀螺仪测量范围 如AT+G10
		case 15:
			USART_OUT(USART1,"OK\r\n");
			if(buffer[5]=='1')
				*(flashData.scaleMode+buffer[4]-'1')=1;
			else if(buffer[5]=='0')
				*(flashData.scaleMode+buffer[4]-'1')=0;
			else
				USART_OUT(USART1,"(flashData.scaleMode) command error\r\n");
			USART_OUT(USART1,"writing\r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			PrintScaleMode();
			break;
		//	AT+G =  设置温度系数选择范围
		case 16:
			USART_OUT(USART1,"OK\r\n");
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
			USART_OUT(USART1,"OK\r\n");
			SetParaDefault();
			break;
		case 18:
			USART_OUT(USART1,"OK\r\n");
			SetCommand(HEATING);
			break;
		case 19:
			USART_OUT(USART1,"OK\r\n");
			SetCommand(~HEATING);
			break;
		default:
			USART_OUT(USART1,"error\r\n");
			break;
	}
	atCommand=0;
	bufferInit();
}

static uint8_t command=0;
void SetCommand(int val){
  switch(val){
  case CORRECT:
    command|=CORRECT;
    break;
  case ACCUMULATE:
    command|=ACCUMULATE;
    break;
  case STATIC:
    command|=STATIC;
    break;
	case HEATING:
    command|=HEATING;
		break;
  case ~CORRECT:
    command&=~CORRECT;
    break;
  case ~ACCUMULATE:
    command&=~ACCUMULATE;
    break;
  case ~STATIC:
    command&=~STATIC;
    break;
  case ~HEATING:
    command&=~HEATING;
    break;
  }
}
uint8_t GetCommand(void){
  return command;
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

