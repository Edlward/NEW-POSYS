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
#include "iwdg.h"
#include "character.h"


extern AllPara_t allPara;
static char buffer[20];
static int bufferI=0;
static int atCommand=0;

void AT_CMD_Judge(void);
void bufferInit(void);
void AT_CMD_Judge(void);


void USART1_IRQHandler(void)
{
  uint8_t data;
  if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    data=USART_ReceiveData(USART1);
    buffer[bufferI]=data;
    bufferI++;
		if(bufferI>=20)
			bufferI=0;
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_CMD_Judge();
    }else{
      if(buffer[0]!='A'){
        bufferInit();
      }
    }
  }else{
    data=USART_ReceiveData(USART1);
  }
}

void USART3_IRQHandler(void)
{
  uint8_t data;
  if(USART_GetITStatus(USART3,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit(USART3,USART_IT_RXNE);
    data=USART_ReceiveData(USART3);
    buffer[bufferI]=data;
    bufferI++;
		if(bufferI>=20)
			bufferI=0;
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_CMD_Judge();
    }else{
      if(buffer[0]!='A'){
        bufferInit();
      }
    }
  }else{
    data=USART_ReceiveData(USART3);
  }
}

void USART6_IRQHandler(void)
{
  uint8_t data;
  if(USART_GetITStatus(USART6,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit(USART6,USART_IT_RXNE);
    data=USART_ReceiveData(USART6);
    buffer[bufferI]=data;
    bufferI++;
		if(bufferI>=20)
			bufferI=0;
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_CMD_Judge();
    }else{
      if(buffer[0]!='A'){
        bufferInit();
      }
    }
  }else{
    data=USART_ReceiveData(USART6);
  }
}

void bufferInit(void){
  bufferI=0;
  for(int i=0;i<20;i++)
    buffer[i]=0;
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
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 6) && strncmp(buffer, "AT+R\r\n", 6)==0)//AT    
	{
		USART_OUT(USART_USED,"OK");
    bufferInit();
		delay_us(10);
		IWDG_Reset();
	}
	else if((bufferI == 4) && strncmp(buffer, "AS\r\n", 4)==0)//AT    
	{
    bufferInit();
		SetFlag(STATIC_FORCE);
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AB\r\n", 4)==0)//AT    
	{
    bufferInit();
		SetFlag(~STATIC_FORCE);
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AR\r\n", 4)==0)//AT    
	{
    bufferInit();
		allPara.sDta.Result_Angle[2]=0.0;
		SetXY(0.0,0.0);
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 8) && strncmp(buffer, "AX", 2)==0)//AT    
	{
		convert_u.data[0]=*(buffer+2);
		convert_u.data[1]=*(buffer+3);
		convert_u.data[2]=*(buffer+4);
		convert_u.data[3]=*(buffer+5);
		if(!isnan(convert_u.value))
			SetXY(convert_u.value,allPara.talkData.y);
    bufferInit();
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 8) && strncmp(buffer, "AY", 2)==0)//AT    
	{
		convert_u.data[0]=*(buffer+2);
		convert_u.data[1]=*(buffer+3);
		convert_u.data[2]=*(buffer+4);
		convert_u.data[3]=*(buffer+5);
		if(!isnan(convert_u.value))
			SetXY(allPara.talkData.x,convert_u.value);
    bufferInit();
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 8) && strncmp(buffer, "AA", 2)==0)//AT    
	{
		convert_u.data[0]=*(buffer+2);
		convert_u.data[1]=*(buffer+3);
		convert_u.data[2]=*(buffer+4);
		convert_u.data[3]=*(buffer+5);
		if(fabs(convert_u.value-allPara.sDta.Result_Angle[2])<181.f&&!isnan(convert_u.value))
			allPara.sDta.Result_Angle[2]=convert_u.value;
    bufferInit();
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 14) && strncmp(buffer, "AT+printData\r\n", 14)==0)
    atCommand=5;
  else if((bufferI == 11) && strncmp(buffer, "AT+noheat\r\n", 11)==0)//AT    
	{
    bufferInit();
		SetFlag(~HEATING);
		USART_OUT(USART_USED,"OK");
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
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 5) && strncmp(buffer, "AME\r\n", 3)==0)//AT    
	{
    bufferInit();
		SetFlag(TEST_MACHINERY);
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 6) && strncmp(buffer, "AMEN\r\n", 4)==0)//AT    
	{
    bufferInit();
		SetFlag(~TEST_MACHINERY);
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 5) && strncmp(buffer, "AMC\r\n", 3)==0)//AT    
	{
    bufferInit();
		SetFlag(TEST_MACHINERY_CLEAR);
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 10) && strncmp(buffer, "AWR1", 4)==0)//AT    
	{
		atCommand=UPDATE_WHEEL_R1;
//		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 10) && strncmp(buffer, "AWR2", 4)==0)//AT    
	{
		atCommand=UPDATE_WHEEL_R2;
//		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 10) && strncmp(buffer, "AAGE", 4)==0)//AT    
	{
		atCommand=UPDATE_ANGLE_ERROR;
//		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 10) && strncmp(buffer, "ACAF", 4)==0)//AT    
	{
		atCommand=UPDATE_CALIBRATION_FACTOR;
//		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 10) && strncmp(buffer, "AGYS", 4)==0)//AT    
	{
		atCommand=UPDATE_GYRO_SCALE;
//		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 9) && strncmp(buffer, "ATFLASH", 7)==0)//AT    
	{
		atCommand=RETURN_DATA_TO_TESTPLAN;
//		USART_OUT(USART_USED,"OK");
	}
  else 
	{
    atCommand=666;
    bufferInit();
	}
  
}

void AT_CMD_Handle(void){
	
	union{
		uint8_t data[4];
		float value;
	}convert_u;
	switch(atCommand)
	{
		case 0:
			
			break;
		case UPDATE_WHEEL_R1:
			USART_OUT(USART_USED,"OK");

			convert_u.data[0]=*(buffer+4);
			convert_u.data[1]=*(buffer+5);
			convert_u.data[2]=*(buffer+6);
			convert_u.data[3]=*(buffer+7);
			allPara.sDta.para.rWheelNo1=(double)convert_u.value;
			USART_OUTByDMAF(allPara.sDta.para.rWheelNo1);
			USART_EnterByDMA();

			bufferInit();
			writeCharacters();
			USART_OUT(USART_USED,"DONE");
			break;
		case UPDATE_WHEEL_R2:
			USART_OUT(USART_USED,"OK");

			convert_u.data[0]=*(buffer+4);
			convert_u.data[1]=*(buffer+5);
			convert_u.data[2]=*(buffer+6);
			convert_u.data[3]=*(buffer+7);
			allPara.sDta.para.rWheelNo2=(double)convert_u.value;
			USART_OUTByDMAF(allPara.sDta.para.rWheelNo2);
			USART_EnterByDMA();

			bufferInit();
			writeCharacters();
			USART_OUT(USART_USED,"DONE");
			break;
		case UPDATE_ANGLE_ERROR:
			USART_OUT(USART_USED,"OK");

			convert_u.data[0]=*(buffer+4);
			convert_u.data[1]=*(buffer+5);
			convert_u.data[2]=*(buffer+6);
			convert_u.data[3]=*(buffer+7);
			allPara.sDta.para.angleWheelError=(double)convert_u.value;
			USART_OUTByDMAF(allPara.sDta.para.angleWheelError);
			USART_EnterByDMA();

			bufferInit();
			writeCharacters();
		
			USART_OUT(USART_USED,"DONE");
			break;
		case UPDATE_CALIBRATION_FACTOR:
			USART_OUT(USART_USED,"OK");

			convert_u.data[0]=*(buffer+4);
			convert_u.data[1]=*(buffer+5);
			convert_u.data[2]=*(buffer+6);
			convert_u.data[3]=*(buffer+7);
			USART_OUTByDMAF(allPara.sDta.para.calibrationFactor);
			USART_EnterByDMA();

			allPara.sDta.para.calibrationFactor=(double)convert_u.value;

			bufferInit();
			writeCharacters();
		
			USART_OUT(USART_USED,"DONE");
			break;
		case UPDATE_GYRO_SCALE:
			USART_OUT(USART_USED,"OK");

			convert_u.data[0]=*(buffer+4);
			convert_u.data[1]=*(buffer+5);
			convert_u.data[2]=*(buffer+6);
			convert_u.data[3]=*(buffer+7);
			allPara.sDta.para.gyroScale=(uint32_t)convert_u.value;
		
			USART_OUTByDMAF(allPara.sDta.para.gyroScale);
			USART_EnterByDMA();
		
			bufferInit();
			writeCharacters();
			IWDG_Reset();
		
			USART_OUT(USART_USED,"DONE");
			break;
		
		case RETURN_DATA_TO_TESTPLAN:
			USART_OUT(USART_USED,"OK");
		  ReturnDataToTestPlan();
			USART_OUT(USART_USED,"DONE");
			bufferInit();
		break;
		
		
		default:
			USART_OUT(USART_USED,"error\r\n");
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
	case TEST_MACHINERY:
    allPara.sDta.flag|=TEST_MACHINERY;
		break;
  case ~TEST_MACHINERY:
    allPara.sDta.flag&=~TEST_MACHINERY;
    break;
	case TEST_MACHINERY_CLEAR:
    allPara.sDta.flag|=TEST_MACHINERY_CLEAR;
		break;
  case ~TEST_MACHINERY_CLEAR:
    allPara.sDta.flag&=~TEST_MACHINERY_CLEAR;
    break;
  }
}
#define RETURN_DATA_LONG     44
void ReturnDataToTestPlan(void)
{

	uint32_t baseAdd=READ_FLASH_SAVE_PHYSICAL_PARA_ADDR;
	
	character_t returnData;
	uint8_t sendReturnData[RETURN_DATA_LONG];
	unsigned int* writeAddr=(unsigned int*)(&returnData);
  unsigned int* endaddr=writeAddr+sizeof(character_t)/4;
	union {
		 double sendDouble;
		 uint8_t sendChar[8];
	}sendDataDouble;

  while(writeAddr<endaddr)//Ð´Êý¾Ý
  {
    *writeAddr=*(volatile unsigned int*)(baseAdd);
    writeAddr++;
    baseAdd+=4;
  }
	
	sendReturnData[0]='R';
	sendReturnData[1]='E';
	
	sendReturnData[RETURN_DATA_LONG-2]='\r';
	sendReturnData[RETURN_DATA_LONG-1]='\n';
	
	sendDataDouble.sendDouble = returnData.angleWheelError;
	memcpy(sendReturnData+2,sendDataDouble.sendChar,8);

  sendDataDouble.sendDouble = returnData.rWheelNo1;
	memcpy(sendReturnData+10,sendDataDouble.sendChar,8);

	sendDataDouble.sendDouble = returnData.rWheelNo2;
	memcpy(sendReturnData+18,sendDataDouble.sendChar,8);

	sendDataDouble.sendDouble = returnData.calibrationFactor;
	memcpy(sendReturnData+26,sendDataDouble.sendChar,8);

	sendDataDouble.sendDouble = returnData.gyroScale;
	memcpy(sendReturnData+34,sendDataDouble.sendChar,8);


	for(int i=0;i<RETURN_DATA_LONG;i++)
	{
		delay_ms(1);
		USART_SendData(USART_USED,sendReturnData[i]);
	}

	USART_OUTByDMAF(returnData.angleWheelError);
	USART_OUTByDMAF(returnData.rWheelNo1);
	USART_OUTByDMAF(returnData.rWheelNo2);
	USART_OUTByDMAF(returnData.calibrationFactor);
	USART_OUTByDMAF(returnData.gyroScale);
	USART_EnterByDMA();



}

