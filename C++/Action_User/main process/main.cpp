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
#include "gpio.h"
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
	USART3_DMA_Init(115200);
	SPI1_Init();
	SPI2_Init();
	CS_Config();
	Led_Init();
	//devices all initiate
	deviceBase::devicesAllInit();
	TIM_Init(TIM2,999,83,1,0);
  pwm_init(999, 83);
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
	AHRS_Init();
	while(DEVICE_IS_RUNNING)
	{
		while(!getTimeFlag());
		/*temperature control*/
		getICM20602_Gyro()->tempControl.temp_pid_ctr(getICM20602_Gyro()->temp);
		updateAHRS();
		calculatePos();
		dataSend();
	}
	running();
}

//	initAHRS();
//	while(DEVICE_IS_RUNNING)
//	{
//		while(!getTimeFlag());
//		getLSM303AGR_Acc().UpdateData();
//		getLSM303AGR_Mag().UpdateData();
//		getI3G4250D().UpdateData();
//		UpdateEncoder();
//		updateAHRS();
//		calculatePos();
//		dataSend();
//	}
//	running();
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
