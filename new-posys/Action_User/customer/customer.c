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

void DataSend(void)
{
	int i;
	uint8_t 
	tdata[20];
	three_axis angle;
	gyro_t w_icm;
	
  union{
		float   val;
		uint8_t data[4];
	}valSend;
	angle=getAngle();
	
  tdata[0]=0x0d;
  tdata[1]=0x0a;
  tdata[18]=0x0a;
  tdata[19]=0x0d;
	
	valSend.val=angle.z;
  memcpy(tdata+2,valSend.data,4);
	
	valSend.val=getPosX();
  memcpy(tdata+6,valSend.data,4);
	 
	valSend.val=getPosY();
  memcpy(tdata+10,valSend.data,4);
	 
	icm_read_gyro_rate(&w_icm);
	valSend.val=w_icm.No1.z;
  memcpy(tdata+14,valSend.data,4);
	
	for(i=0;i<20;i++)
   USART_SendData(USART1,tdata[i]);	

}
