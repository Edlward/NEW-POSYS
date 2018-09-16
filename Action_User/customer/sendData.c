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
	
	UpdateTalkInfo();
	
	valSend.val=(float)allPara.talkData.angle;
  memcpy(tdata+2,valSend.data,4);
	
	if(allPara.sDta.flag&TEST_MACHINERY)
		valSend.val=(float)getDirectLine(allPara.sDta.para.rWheelNo1,allPara.sDta.para.rWheelNo2,allPara.sDta.para.angleWheelError);
	else
		valSend.val=(float)allPara.talkData.speedX;
  memcpy(tdata+6,valSend.data,4);
	
	if(allPara.sDta.flag&TEST_MACHINERY)
		valSend.val=(float)getEncoderSum(0);
	else
		valSend.val=(float)allPara.talkData.speedY;
  memcpy(tdata+10,valSend.data,4);
	
	if(allPara.sDta.flag&TEST_MACHINERY)
		valSend.val=(float)allPara.sDta.codeData[0];
	else
		valSend.val=(float)allPara.talkData.x;
  memcpy(tdata+14,valSend.data,4);
	 	
	if(allPara.sDta.flag&TEST_MACHINERY)
		valSend.val=(float)allPara.sDta.codeData[1];
	else
		valSend.val=(float)allPara.talkData.y;
  memcpy(tdata+18,valSend.data,4);
	 
	if(allPara.sDta.flag&TEST_MACHINERY)
		valSend.val=(float)getEncoderSum(1);
	else
		valSend.val=(float)allPara.talkData.wz;
  memcpy(tdata+22,valSend.data,4);
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