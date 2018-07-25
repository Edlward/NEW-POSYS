/**  
  ******************************************************************************
  * @file    main.cpp
  * @author  Luo Xiaoyi(gay) and Qiao Zhijian 
  * @version V1.0
  * @date    2017.2.27
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "timer.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "spi.h"
#include "device.h"
#include "action_matrix.h"
#include "usart.h"
#include "kalmanFilter.h"
#include "calculateAttitude.h"
#include "LSM303AGR.h"
#include "I3G4250D.h"
#include "ICM20602.h"
#include "device.h"
#include "pos.h"
#include "ahrs.h"
#include "user.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
static void running(void);
/* main functions -------------------------------------------------------------*/
/** 
  * @brief   
  * @none    none
  * @retval  none
  */
int main(void)
{	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	USART1_Init();
	USART6DMAInit(921600);
	SPI1_Init();
	SPI2_Init();
	CS_Config();
	delay_ms(500);
	//devices all initiate
	deviceBase::devicesAllInit();
	TIM_Init(TIM7,999,83,1,0);
	running(); 
}
/* Private  functions ---------------------------------------------------------*/
/**
  * @brief  
  * @none    none
  * @retval  none
  */
static void running(void)
{
	initAHRS();
	while(DEVICE_IS_RUNNING)
	{
		while(!getTimeFlag());
		//judeg getICM20602_Gyro() return null
		getICM20602_Gyro().UpdateData();
		UpdateEncoder();
		updateAHRS();
		calculatePos();
		dataSend();
	}
	running();
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
