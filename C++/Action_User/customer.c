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
#include "motion_attitude_algorithm.h"
#include "string.h"
#include "stm32f4xx_usart.h"
#include "pos.h"
#include "can.h"
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
extern	uint16_t data[2];
extern int route_summer;
void DataSend(void)
{
	int i;
	uint8_t 
	tdata[28];
	three_axis angle;
	three_axis w_icm;
	/*
	��Ϊ����ֻ�ܰ�λ��λ�ط�,
	����Ҫ����������
	*/
  union{
		int   val;
		uint8_t data[4];
	}valSend;
	angle=getAngle();
	
  tdata[0]=0x0d;
  tdata[1]=0x0a;
  tdata[26]=0x0a;
  tdata[27]=0x0d;
	
	valSend.val=route_summer;
  memcpy(tdata+2,valSend.data,4);
	
	valSend.val=route_summer;
  memcpy(tdata+6,valSend.data,4);
	
	valSend.val=route_summer;
  memcpy(tdata+10,valSend.data,4);
	
	valSend.val=route_summer;
  memcpy(tdata+14,valSend.data,4);
	 
	valSend.val=route_summer;
  memcpy(tdata+18,valSend.data,4);
	 
	valSend.val=route_summer;
  memcpy(tdata+22,valSend.data,4);
	
	for(i=0;i<28;i++)
   USART_SendData(USART1,tdata[i]);	

}
