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

extern AllPara_t allPara;
static char buffer[20];
static int bufferI=0;
static int atCommand=0;

void AT_CMD_Judge(void);
void SetParaDefault(void);
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

int32_t getEncoderSum(int num)
{	
	uint16_t encoder[2]={allPara.sDta.codeData[0],allPara.sDta.codeData[1]};
	static uint16_t encoderLast[2]={0};
	static int ignore=0;
	int32_t vell[2]={0};
	static int sum[2]={0};
	
	ignore++;
	if(ignore<5)
	{
		vell[0]=0;
		vell[1]=0;
		encoderLast[0]=encoder[0];
		encoderLast[1]=encoder[1];
	}else{
		vell[0]=encoder[0]-encoderLast[0];
		vell[1]=encoder[1]-encoderLast[1];
		encoderLast[0]=encoder[0];
		encoderLast[1]=encoder[1];
	}
	
	
	vell[0]-=(vell[0]>(32768/2))*32768;
	vell[0]+=(vell[0]<(-(32768/2)))*32768;
	
	vell[1]-=(vell[1]>(32768/2))*32768;
	vell[1]+=(vell[1]<(-(32768/2)))*32768;
	
	sum[0]=sum[0]+vell[0];
	sum[1]=sum[1]+vell[1];
	
	if(allPara.sDta.flag&TEST_MACHINERY_CLEAR)
	{
		sum[0]=0;
		sum[1]=0;
	}
	return sum[num];
}

//return the length of PPS's measured value when it walks directly
double getDirectLine(float wheel1,float wheel2,float errorAngle)
{	
	uint16_t encoder[2]={allPara.sDta.codeData[0],allPara.sDta.codeData[1]};
	static uint16_t encoderLast[2]={0};
	static int ignore=0;
	int32_t vell[2]={0};
	double sumF[2]={0.0};
	static int sum[2]={0};
	
	ignore++;
	if(ignore<5)
	{
		vell[0]=0;
		vell[1]=0;
		encoderLast[0]=encoder[0];
		encoderLast[1]=encoder[1];
	}else{
		vell[0]=encoder[0]-encoderLast[0];
		vell[1]=encoder[1]-encoderLast[1];
		encoderLast[0]=encoder[0];
		encoderLast[1]=encoder[1];
	}
	
	vell[0]-=(vell[0]>(32768/2))*32768;
	vell[0]+=(vell[0]<(-(32768/2)))*32768;
	
	vell[1]-=(vell[1]>(32768/2))*32768;
	vell[1]+=(vell[1]<(-(32768/2)))*32768;
	
	sum[0]=sum[0]+vell[0];
	sum[1]=sum[1]+vell[1];
	
	double real[2]={0.0,0.0};
	
	real[0]=sum[0];
	real[1]=1.0/cos((double)errorAngle/180.0*PI_DOUBLE)*sum[1]-tan((double)errorAngle/180.0*PI_DOUBLE)*sum[0];
	
	sumF[0]=real[0]/32768*2*PI*wheel1;
	sumF[1]=real[1]/32768*2*PI*wheel2;
	
	if(allPara.sDta.flag&TEST_MACHINERY_CLEAR)
	{
		sum[0]=0;
		sum[1]=0;
	}
	return sqrt(pow(sumF[0],2)+pow(sumF[1],2));
}


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
	
	valSend.val=(float)allPara.sDta.Result_Angle[2];
  memcpy(tdata+2,valSend.data,4);
	
	if(allPara.sDta.flag&TEST_MACHINERY)
		valSend.val=(float)getDirectLine(allPara.sDta.para.rWheelNo1,allPara.sDta.para.rWheelNo2,allPara.sDta.para.angleWheelError);
	else
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
	
	if(allPara.sDta.flag&TEST_MACHINERY)
		valSend.val=(float)getEncoderSum(0);
	else
		valSend.val=(float)allPara.sDta.codeData[0];
  memcpy(tdata+26,valSend.data,4);
	 
	if(allPara.sDta.flag&TEST_MACHINERY)
		valSend.val=(float)getEncoderSum(1);
	else
		valSend.val=(float)allPara.sDta.codeData[1];
  memcpy(tdata+30,valSend.data,4);
//	
	#ifdef TEST_SUMMER
	i=i;
	USART_OUTByDMAF(allPara.GYRO_Temperature[0]);
//	USART_OUTByDMAF(allPara.sDta.GYRO_TemperatureAim[0]);
//	for(int i=0;i<AXIS_NUMBER;i++)
//		USART_OUTByDMAF(allPara.sDta.GYRO_Aver[i]);
//	for(int i=0;i<AXIS_NUMBER;i++)
//		USART_OUTByDMAF(allPara.ACC_Raw[0][i]);
	
//	USART_OUT_F(allPara.GYROWithoutRemoveDrift[0][2]);
//	USART_OUT_F(allPara.GYROWithoutRemoveDrift[1][2]);
//	USART_OUT_F(allPara.GYROWithoutRemoveDrift[2][2]);
//	for(int i=0;i<3;i++)
//		USART_OUT_F(allPara.GYROWithoutRemoveDrift[i][2]);

//	for(int i=0;i<GYRO_NUMBER;i++)
//	{
//		USART_OUT_F(allPara.sDta.GYRO_TemperatureAim[i]);
//		USART_OUT_F(allPara.GYRO_Temperature[i]);
//	}
//	USART_OUTByDMAF(allPara.sDta.GYRO_Aver[2]);
////	USART_OUT_F(lowpass);
//	USART_OUTByDMAF(allPara.sDta.Result_Angle[2]);
//	USART_OUT_F(allPara.sDta.GYRO_Bais[2]);
//	USART_OUT_F(allPara.GYRO_Real[2]);
	USART_OUTByDMAF(allPara.sDta.codeData[0]);
	USART_OUTByDMAF(allPara.sDta.codeData[1]);
//	USART_OUT_F(allPara.vell[0]);
//	USART_OUT_F(allPara.vell[1]);
//	USART_OUT_F(allPara.isStatic);
	
//	static int codesum[2]={0};
//	codesum[0]+=allPara.vell[0];
//	codesum[1]+=allPara.vell[1];
////	USART_OUT_F(codesum[0]);
////	USART_OUT_F(codesum[1]);
//	USART_OUTByDMAF(allPara.sDta.posx);
//	USART_OUTByDMAF(allPara.sDta.posy);
//	USART_OUT_F(sqrt(allPara.sDta.posx*allPara.sDta.posx+allPara.sDta.posy*allPara.sDta.posy));
//	USART_OUT_F(codesum[1]);
//	USART_OUTByDMAF(allPara.sDta.flag&STATIC_FORCE);
	USART_EnterByDMA();
	#else
	
	for(i=0;i<DMA_SEND_SIZE;i++)
	{
		if(USART_USED==USART3)
			USART_SendDataToDMA_USART3(tdata[i]);
		else if(USART_USED==USART1)
			USART_SendDataToDMA_USATR1(tdata[i]);
	}
	
	SetFlag(~TEST_MACHINERY_CLEAR);
	#endif
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
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 4) && strncmp(buffer, "AQ\r\n", 4)==0)//AT    
	{
    bufferInit();
		allPara.sDta.Result_Angle[2]=0.0;
		allPara.sDta.posx=0.0;
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 8) && strncmp(buffer, "AX", 2)==0)//AT    
	{
		convert_u.data[0]=*(buffer+2);
		convert_u.data[1]=*(buffer+3);
		convert_u.data[2]=*(buffer+4);
		convert_u.data[3]=*(buffer+5);
		allPara.sDta.posx=convert_u.value;
    bufferInit();
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 8) && strncmp(buffer, "AY", 2)==0)//AT    
	{
		convert_u.data[0]=*(buffer+2);
		convert_u.data[1]=*(buffer+3);
		convert_u.data[2]=*(buffer+4);
		convert_u.data[3]=*(buffer+5);
		allPara.sDta.posy=convert_u.value;
    bufferInit();
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 8) && strncmp(buffer, "AA", 2)==0)//AT    
	{
		convert_u.data[0]=*(buffer+2);
		convert_u.data[1]=*(buffer+3);
		convert_u.data[2]=*(buffer+4);
		convert_u.data[3]=*(buffer+5);
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
	else if((bufferI == 5) && strncmp(buffer, "AMC\r\n", 3)==0)//AT    
	{
    bufferInit();
		SetFlag(TEST_MACHINERY_CLEAR);
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 10) && strncmp(buffer, "AWR1", 4)==0)//AT    
	{
		atCommand=1;
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 10) && strncmp(buffer, "AWR2", 4)==0)//AT    
	{
		atCommand=2;
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 10) && strncmp(buffer, "AAGE", 4)==0)//AT    
	{
		atCommand=3;
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 10) && strncmp(buffer, "ACAF", 4)==0)//AT    
	{
		atCommand=4;
		USART_OUT(USART_USED,"OK");
	}
	else if((bufferI == 10) && strncmp(buffer, "AGYS", 4)==0)//AT    
	{
		atCommand=5;
		USART_OUT(USART_USED,"OK");
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
		case 1:
			convert_u.data[0]=*(buffer+4);
			convert_u.data[1]=*(buffer+5);
			convert_u.data[2]=*(buffer+6);
			convert_u.data[3]=*(buffer+7);
			allPara.sDta.para.rWheelNo1=(double)convert_u.value;
			bufferInit();
			writeCharacters();
			break;
		case 2:
			convert_u.data[0]=*(buffer+4);
			convert_u.data[1]=*(buffer+5);
			convert_u.data[2]=*(buffer+6);
			convert_u.data[3]=*(buffer+7);
			allPara.sDta.para.rWheelNo2=(double)convert_u.value;
			bufferInit();
			writeCharacters();
			break;
		case 3:
			convert_u.data[0]=*(buffer+4);
			convert_u.data[1]=*(buffer+5);
			convert_u.data[2]=*(buffer+6);
			convert_u.data[3]=*(buffer+7);
			allPara.sDta.para.angleWheelError=(double)convert_u.value;
			bufferInit();
			writeCharacters();
			break;
		case 4:
			convert_u.data[0]=*(buffer+4);
			convert_u.data[1]=*(buffer+5);
			convert_u.data[2]=*(buffer+6);
			convert_u.data[3]=*(buffer+7);
			allPara.sDta.para.calibrationFactor=(double)convert_u.value;
			bufferInit();
			writeCharacters();
			break;
		case 5:
			convert_u.data[0]=*(buffer+4);
			convert_u.data[1]=*(buffer+5);
			convert_u.data[2]=*(buffer+6);
			convert_u.data[3]=*(buffer+7);
			allPara.sDta.para.gyroScale=(uint32_t)convert_u.value;
			bufferInit();
			writeCharacters();
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

void SetParaDefault(void)
{

}

void DeadWhileReport(uint8_t a)
{
	if(USART_USED==USART3)
	{
		USART_SendDataToDMA_USART3('h');
		USART_SendDataToDMA_USART3('a');
		USART_SendDataToDMA_USART3('r');
		USART_SendDataToDMA_USART3('d');
		USART_SendDataToDMA_USART3('f');
		USART_SendDataToDMA_USART3('a');
		USART_SendDataToDMA_USART3('u');
		USART_SendDataToDMA_USART3('l');
		USART_SendDataToDMA_USART3('t');
		USART_SendDataToDMA_USART3('\r');
		USART_SendDataToDMA_USART3(a);
	}
	else if(USART_USED==USART1)
	{
		USART_SendDataToDMA_USATR1('h');
		USART_SendDataToDMA_USATR1('a');
		USART_SendDataToDMA_USATR1('r');
		USART_SendDataToDMA_USATR1('d');
		USART_SendDataToDMA_USATR1('f');
		USART_SendDataToDMA_USATR1('a');
		USART_SendDataToDMA_USATR1('u');
		USART_SendDataToDMA_USATR1('l');
		USART_SendDataToDMA_USATR1('t');
		USART_SendDataToDMA_USATR1('\r');
		USART_SendDataToDMA_USATR1(a);
	}
}

void ReportHardFault(void)
{
	USART_OUT(USART_USED,"hardfault\r\n");
}
