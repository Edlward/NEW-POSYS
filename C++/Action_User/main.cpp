/**  
  ******************************************************************************
  * @file    main.cpp
  * @author  Luo Xiaoyi 
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
#include "device.h"
#include "pos.h"
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
	USART1_INIT();
	SPI1_Init();
	SPI2_Init();
	SPI3_Init();
	CS_Config();
	delay_ms(500);
	//设备全初始化
	deviceBase::devicesAllInit();
	delay_ms(1000);
	TIM_Init(TIM7,399,83,0,0);
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
		getLSM303AGR_Acc().updateData();
		getLSM303AGR_Mag().updateData();
		getI3G4250D().updateData();
		UpdateEncoder();
		updateAHRS();
		calculatePos();
		dataSend();
	}
	running();
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
