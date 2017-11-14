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

void DataSend(void)
{
	 	int i;
	uint8_t 
	tdata[28];
	float angle[3];
  union{
		float   val;
		uint8_t data[4];
	}valSend;
	getAngle(angle);
	
  tdata[0]=0x0d;
  tdata[1]=0x0a;
  tdata[26]=0x0a;
  tdata[27]=0x0d;
	
	valSend.val=angle[2];
  memcpy(tdata+2,valSend.data,4);
	
	valSend.val=angle[0];
  memcpy(tdata+6,valSend.data,4);
	
	valSend.val=angle[1];
  memcpy(tdata+10,valSend.data,4);
	
	valSend.val=allPara.GYRO_Real[0];
  memcpy(tdata+14,valSend.data,4);
	 
	valSend.val=allPara.GYRO_Real[1];
  memcpy(tdata+18,valSend.data,4);
	 
	valSend.val=allPara.GYRO_Real[2];
  memcpy(tdata+22,valSend.data,4);
	
	for(i=0;i<28;i++)
   USART_SendData(USART1,tdata[i]);
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
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_CMD_Judge();
    }else{
      if(buffer[0]!='A'){
        bufferInit();
        USART_OUT(USART1,"NOT START WITH 'A'\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(USART1);
  }
}
static int heatPower=0;
extern uint8_t sendPermit;
static int atCommand=0;
void AT_CMD_Judge(void){
  if((bufferI == 4) && strncmp(buffer, "AT\r\n", 4)==0)//AT    
    atCommand=1;
  else if((bufferI == 10) && strncmp(buffer, "AT+begin\r\n", 10)==0)//AT    
    atCommand=2;
	else if((bufferI == 13) && strncmp(buffer, "AT+stopSend\r\n", 13)==0)
    atCommand=3;
	else if((bufferI == 14) && strncmp(buffer, "AT+beginSend\r\n", 14)==0)
    atCommand=4;
	else if((bufferI == 14) && strncmp(buffer, "AT+printData\r\n", 14)==0)
    atCommand=5;
  else if((bufferI == 9) && strncmp(buffer, "AT+stop\r\n", 9)==0)//AT    
    atCommand=6;
  else if((bufferI == 13) && strncmp(buffer, "AT+reset=\r\n", 9)==0)//AT    
    atCommand=7;
  else if((bufferI == 8) && strncmp(buffer, "AT+set\r\n", 8)==0)//AT    
    atCommand=8;
  else if((bufferI == 10) && strncmp(buffer, "AT+print\r\n", 10)==0)//AT 
    atCommand=9;
  else if((bufferI == 11) && strncmp(buffer, "AT+static\r\n", 11)==0)//AT  
    atCommand=10;
  else if((bufferI == 14) && strncmp(buffer, "AT+set", 6)==0)//AT    
    atCommand=11;
  else if((bufferI >= 10) && strncmp(buffer, "AT+setmin=", 10)==0)//AT    
    atCommand=12;
	else if((bufferI >= 10) && strncmp(buffer, "AT+setvar", 9)==0)//AT    
    atCommand=13;
  else if((bufferI == 11) && strncmp(buffer, "AT+mode=", 8)==0)//AT    
    atCommand=14;
  else if((bufferI == 12) && strncmp(buffer, "AT+scale=", 9)==0)//AT    
    atCommand=15;
  else if((bufferI == 17) && strncmp(buffer, "AT+select=", 10)==0)//AT    
    atCommand=16;
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
			sendPermit=0;
			USART_OUT(USART1,"OK\r\n");
			break;
		case 4:
			sendPermit=1;
			USART_OUT(USART1,"OK\r\n");
			break;
		case 5:
			USART_OUT_F(allPara.GYRO_Temperature);
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
			heatPower=0;
			USART_OUT(USART1,"OK\r\n");
			if(buffer[9]<='9'&&buffer[9]>='0')
				heatPower+=(buffer[9]-'0')*10;
			if(buffer[10]<='9'&&buffer[10]>='0')
				heatPower+=(buffer[10]-'0');
				
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
		case 12:
			USART_OUT(USART1,"OK\r\n");
			*(flashData.minValue) = atof(buffer+10);
			USART_OUT(USART1,"writing\r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			TempTablePrintf();
			break;
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
			TempTablePrintf();
			break;
		case 14:
			USART_OUT(USART1,"OK\r\n");
			/*不结合之前的数据*/
			if(buffer[8]=='1')
				*(flashData.chartMode)=1;
			/*结合之前的数据*/
			else if(buffer[8]=='0')
				*(flashData.chartMode)=0;
			else
				USART_OUT(USART1,"mode command error\r\n");
			USART_OUT(USART1,"writing\r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			TempTablePrintf();
			break;
		case 15:
			USART_OUT(USART1,"OK\r\n");
			if(buffer[9]=='1')
				*(flashData.scaleMode)=1;
			else if(buffer[9]=='0')
				*(flashData.scaleMode)=0;
			else
				USART_OUT(USART1,"(flashData.scaleMode) command error\r\n");
			USART_OUT(USART1,"writing\r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			break;
		case 16:
			USART_OUT(USART1,"OK\r\n");
			for(int i=10;i<15;i++){
				if(buffer[i]=='1')
					(flashData.chartSelect)[i-10]=1;
				else if(buffer[i]=='0')
					(flashData.chartSelect)[i-10]=0;
				else
					USART_OUT(USART1,"select %d error\r\n",i-10);
			}
			USART_OUT(USART1,"writing\r\n");
			Flash_Write(GetFlashArr(),TempTable_Num);
			USART_OUT(USART1,"write finished\r\n");
			break;
		case 17:
			break;
		case 18:
			break;
		default:
			USART_OUT(USART1,"error\r\n");
			break;
	}
	atCommand=0;
	bufferInit();
}

int getHeatPower(void){
	return heatPower;
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
  case ~CORRECT:
    command&=~CORRECT;
    break;
  case ~ACCUMULATE:
    command&=~ACCUMULATE;
    break;
  case ~STATIC:
    command&=~STATIC;
    break;
  }
}
uint8_t GetCommand(void){
  return command;
}
