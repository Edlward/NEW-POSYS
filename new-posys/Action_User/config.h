/**
  ******************************************************************************
  * @file    *.h
  * @author  Lxy zlq
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
#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "buildExcel.h"
#include "spi.h"
#include "figureAngle.h"
#include "ADXRS453Z.h"
#include "customer.h"
#include "timer.h"
#include "usart.h"
#include "arm_math.h"
#include "string.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "figurePos.h"
#include "quarternion.h"
#include "adc.h"
#include "odom.h"
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

//#define ADXRS453Z
#define NEW_BOARD
#ifdef NEW_BOARD
	#define GPIO_GYRO	GPIOA
	#define GPIO_PIN	GPIO_Pin_4
#else
	#define GPIO_GYRO	GPIOA
	#define GPIO_PIN	GPIO_Pin_2
#endif

/* Exported constants --------------------------------------------------------*/

//#define TEST_SUMMER

#define AUTOCAR
//#define TESTCAR
/* Flash Read Protection */
//#define FLASH_ENCRYP
#define PERIOD    				0.005f
#define dT 					    0.005         

#define R_wheel1        25.284126
#define R_wheel2        25.42820678
#define LEASTNUM			 	50

#define TempTable_min 				  0.35
#define TempTable_max  					0.45
#define TempTable_NUMBER  			150
#define HEATTIME	   	 30.0

#define DOUBLE_SIZE											8
#define FLOAT_SIZE											4
#define UINT8_SIZE											1


#define GYRO_NUMBER    									1
#define AXIS_NUMBER    									3
#define TEMP_SAMPLE_NUMBER    					5

#include "icm_20608_g.h"
#include "temperature_control.h"
#include "flash.h"


#define CORRECT    								0X01
#define START_COMPETE 						0X02
#define NULL_FLAG									0X04
#define HEATING										0X08
#define STATIC_FORCE							0X10

#define ICM20608G_WHO_AM_I							0x75

#define ICM20608G_SELF_TEST_X_GYRO			0x00
#define ICM20608G_SELF_TEST_Y_GYRO			0x01
#define ICM20608G_SELF_TEST_Z_GYRO			0x02
#define ICM20608G_SELF_TEST_X_ACCEL			0x0D
#define ICM20608G_SELF_TEST_Y_ACCEL			0x0E
#define ICM20608G_SELF_TEST_Z_ACCEL			0x0F

#define ICM20608G_XG_OFFS_USRH					0x13
#define ICM20608G_XG_OFFS_USRL					0x14
#define ICM20608G_YG_OFFS_USRH					0x15
#define ICM20608G_YG_OFFS_USRL					0x16
#define ICM20608G_ZG_OFFS_USRH					0x17
#define ICM20608G_ZG_OFFS_USRL					0x18

#define ICM20608G_SMPLRT_DIV						0x19
#define ICM20608G_CONFIG								0x1A
#define ICM20608G_GYRO_CONFIG						0x1B
#define ICM20608G_ACCEL_CONFIG					0x1C
#define ICM20608G_ACCEL_CONFIG2					0x1D
#define ICM20608G_LP_MODE_CFG						0x1E
#define ICM20608G_ACCEL_WOM_THR					0x1F
#define ICM20608G_FIFO_EN								0x23
#define ICM20608G_FSYNC_INT							0x36
#define ICM20608G_INT_PIN_CFG						0x37
#define ICM20608G_INT_ENABLE						0x38
#define ICM20608G_INT_STATUS						0x3A

#define ICM20608G_ACCEL_XOUT_H					0x3B
#define ICM20608G_ACCEL_XOUT_L					0x3C
#define ICM20608G_ACCEL_YOUT_H					0x3D
#define ICM20608G_ACCEL_YOUT_L					0x3E
#define ICM20608G_ACCEL_ZOUT_H					0x3F
#define ICM20608G_ACCEL_ZOUT_L					0x40
#define ICM20608G_TEMP_OUT_H						0x41
#define ICM20608G_TEMP_OUT_L						0x42
#define ICM20608G_GYRO_XOUT_H						0x43
#define ICM20608G_GYRO_XOUT_L						0x44
#define ICM20608G_GYRO_YOUT_H						0x45
#define ICM20608G_GYRO_YOUT_L						0x46
#define ICM20608G_GYRO_ZOUT_H						0x47
#define ICM20608G_GYRO_ZOUT_L						0x48

#define ICM20608G_SIGNAL_PATH_RESET			0x68							
#define ICM20608G_ACCEL_INTEL_CTRL			0x69
#define ICM20608G_USER_CTRL							0x6A
#define ICM20608G_PWR_MGMT_1						0x6B
#define ICM20608G_PWR_MGMT_2						0x6C

#define ICM20608G_FIFO_COUNTH						0x72
#define ICM20608G_FIFO_COUNTL						0x73
#define ICM20608G_FIFO_R_W							0x74

#define ICM20608G_XA_OFFSET_H						0x77
#define ICM20608G_XA_OFFSET_L						0x78
#define ICM20608G_YA_OFFSET_H						0x7A
#define ICM20608G_YA_OFFSET_L						0x7B
#define ICM20608G_ZA_OFFSET_H						0x7D
#define ICM20608G_ZA_OFFSET_L						0x7E

typedef struct{
	/*判断此次是否为看门狗复位*/
  uint32_t isReset;
  
	double posx;
	
	double posy;

	float vellx;
	
	float velly;
	
	float GYRO_TemperatureAim[GYRO_NUMBER];
	
	double GYRO_Aver[AXIS_NUMBER];
	
	/*陀螺仪处理后的数据*/
	double GYRO_Bais[AXIS_NUMBER];
  
	/*用于角度积分的四元数*/
	double quarternion[4];
  
	/*最终确定的三轴角度*/
	double Result_Angle[AXIS_NUMBER];
  
	uint32_t codeData[2];
	
  uint32_t data_last[2];
	
	int vell[2];
	
  uint32_t flag;
	
  uint32_t time;
}DataSave_t;


typedef struct{
	
	/*重启需要储存的数据*/
	DataSave_t sDta;
	
	/*陀螺仪原始数据*/
	double GYROWithoutRemoveDrift[GYRO_NUMBER][AXIS_NUMBER];
	
	double GYRORemoveDrift[GYRO_NUMBER][AXIS_NUMBER];
	
	/*陀螺仪处理后的数据*/
	double GYRO_Real[AXIS_NUMBER];

	/*陀螺仪原始数据*/
	float ACC_Raw[GYRO_NUMBER][AXIS_NUMBER];
	
	float ACC_Aver[GYRO_NUMBER][AXIS_NUMBER];
	
	float ACC_InitSum;
	
	/*加速度计的角度 只有X Y轴角度*/
	float ACC_Angle[GYRO_NUMBER][AXIS_NUMBER-1];
	
	float ACC_RealAngle[AXIS_NUMBER-1];
	
	/*陀螺仪温度*/
	float GYRO_Temperature[GYRO_NUMBER];
	
	float GYRO_TemperatureDif[GYRO_NUMBER];
	/*三个陀螺仪三个轴随温度变化的系数*/
	float driftCoffecient[GYRO_NUMBER][AXIS_NUMBER];
	
	int cpuUsage;
	
	uint32_t resetTime;
	
	double kalmanZ;
	
	int resetFlag;
}AllPara_t;


typedef enum{FALSE = 0,TRUE = !FALSE}BOOL;

#endif	

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
