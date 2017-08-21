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
#include "string.h"
#include "stm32f4xx_usart.h"
#include "figurePos.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static float R=0.012;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
void Set_R_Zaxis(float val)
{
	R=val;
}
float Get_R_Zaxis(void)
{
	return R;
}
void DataSend(void)
{
	int i;
	uint8_t tdata[28];
	three_axis angle;
	three_axis w_icm;
	/*
	因为串口只能八位八位地发,
	所以要构建联合体
	*/
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
	 
	icm_read_gyro_rate(&w_icm);
	valSend.val=w_icm.z;
  memcpy(tdata+22,valSend.data,4);
	
	for(i=0;i<28;i++)
   USART_SendData(USART1,tdata[i]);	

}

