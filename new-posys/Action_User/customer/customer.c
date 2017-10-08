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
#include "customer.h"
#include "figureAngle.h"
#include "figurePos.h"
#include "string.h"
#include "stm32f4xx_usart.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "buildExcel.h"
#include "flash.h"
void AT_CMD_Handle(void);

void DataSend(void)
{
	 	int i;
	uint8_t 
	tdata[28];
	three_axis angle;
	three_axis w_icm;
  union{
		float   val;
		uint8_t data[4];
	}valSend;
	angle=getAngle();
	
  tdata[0]=0x0d;
  tdata[1]=0x0a;
  tdata[26]=0x0a;
  tdata[27]=0x0d;
	
	valSend.val=angle.z;
  memcpy(tdata+2,valSend.data,4);
	
	valSend.val=angle.x;
  memcpy(tdata+6,valSend.data,4);
	
	valSend.val=angle.y;
  memcpy(tdata+10,valSend.data,4);
	
	valSend.val=getPosX();
  memcpy(tdata+14,valSend.data,4);
	 
	valSend.val=getPosY();
  memcpy(tdata+18,valSend.data,4);
	 
	valSend.val=getActIcm();
  memcpy(tdata+22,valSend.data,4);
	
	for(i=0;i<28;i++)
   USART_SendData(USART1,tdata[i]);	
}

static char buffer[20];
static int bufferI=0;
extern uint8_t* 	chartMode;
extern uint8_t 	*chartSelect; 
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
      AT_CMD_Handle();
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

void AT_CMD_Handle(void){
  if((bufferI == 4) && strncmp(buffer, "AT\r\n", 4)==0)//AT    
  {
    USART_OUT(USART1,"OK\r\n");
  }
  else if((bufferI == 10) && strncmp(buffer, "AT+begin\r\n", 10)==0)//AT    
  {
    SetCommand(ACCUMULATE);
    USART_OUT(USART1,"OK\r\n");
  }
  else if((bufferI == 9) && strncmp(buffer, "AT+stop\r\n", 9)==0)//AT    
  {
    SetCommand(~ACCUMULATE);
    USART_OUT(USART1,"OK\r\n");
  }
  else if((bufferI == 10) && strncmp(buffer, "AT+reset\r\n", 10)==0)//AT    
  {
    SetCommand(CORRECT);
    USART_OUT(USART1,"OK\r\n");
  }
  else if((bufferI == 8) && strncmp(buffer, "AT+set\r\n", 8)==0)//AT    
  {
    SetCommand(~CORRECT);
    USART_OUT(USART1,"OK\r\n");
  }
  else if((bufferI == 10) && strncmp(buffer, "AT+print\r\n", 10)==0)//AT    
  {
    USART_OUT(USART1,"OK\r\n");
    TempTablePrintf();
  }
  else if((bufferI == 11) && strncmp(buffer, "AT+static\r\n", 11)==0)//AT    
  {
    USART_OUT(USART1,"OK\r\n");
    SetCommand(STATIC);
  }
  else if((bufferI == 14) && strncmp(buffer, "AT+setx=\r\n", 8)==0)//AT    
  {
    USART_OUT(USART1,"OK\r\n");
	  union{
		float   val;
		uint8_t data[4];
		}temp_float;
		temp_float.data[0]=buffer[8];
		temp_float.data[1]=buffer[9];
		temp_float.data[2]=buffer[10];
		temp_float.data[3]=buffer[11];
		SetPosX(temp_float.val);
  }
  else if((bufferI == 14) && strncmp(buffer, "AT+sety=\r\n", 8)==0)//AT    
  {
    USART_OUT(USART1,"OK\r\n");
	  union{
		float   val;
		uint8_t data[4];
		}temp_float;
		temp_float.data[0]=buffer[8];
		temp_float.data[1]=buffer[9];
		temp_float.data[2]=buffer[10];
		temp_float.data[3]=buffer[11];
		SetPosY(temp_float.val);
  }
  else if((bufferI == 14) && strncmp(buffer, "AT+seta=\r\n", 8)==0)//AT    
  {
    USART_OUT(USART1,"OK\r\n");
	  union{
		float   val;
		uint8_t data[4];
		}temp_float;
		temp_float.data[0]=buffer[8];
		temp_float.data[1]=buffer[9];
		temp_float.data[2]=buffer[10];
		temp_float.data[3]=buffer[11];
		SetAngle(temp_float.val);
  }
  else if((bufferI == 11) && strncmp(buffer, "AT+mode=", 8)==0)//AT    
  {
    USART_OUT(USART1,"OK\r\n");
    if(buffer[8]=='1')
      *chartMode=1;
    else if(buffer[8]=='0')
      *chartMode=0;
    else
      USART_OUT(USART1,"mode command error\r\n");
    Flash_Write(GetFlashArr(),TempTable_Num);
    TempTablePrintf();
  }
  else if((bufferI == 17) && strncmp(buffer, "AT+select=", 10)==0)//AT    
  {
    USART_OUT(USART1,"OK\r\n");
    for(int i=10;i<15;i++){
      if(buffer[i]=='1')
        chartSelect[i-10]=1;
      else if(buffer[i]=='0')
        chartSelect[i-10]=0;
      else
        USART_OUT(USART1,"select %d error\r\n",i-10);
    }
    Flash_Write(GetFlashArr(),TempTable_Num);
    TempTablePrintf();
  }
  else 
    USART_OUT(USART1,"error\r\n");
  
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
