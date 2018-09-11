/**
  ******************************************************************************
  * @file    *.h
  * @author  Qzj Action
  * @version 
  * @date   
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  *
  * 
  * 
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RECEIVE_DATA_H
#define __RECEIVE_DATA_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define UPDATE_WHEEL_R1											1
#define UPDATE_WHEEL_R2											2
#define UPDATE_ANGLE_ERROR									3
#define UPDATE_CALIBRATION_FACTOR						4
#define UPDATE_GYRO_SCALE										5

/*������������ƽ̨*/
#define RETURN_ANGLE_ERROR_TO_TESTPLAN      6
#define RETURN_WHEEL_R1_TO_TESTPLAN         7
#define RETURN_WHEEL_R2_TO_TESTPLAN         8
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void SetFlag(int val);
void AT_CMD_Handle(void);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
