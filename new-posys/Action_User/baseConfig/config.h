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
#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include <stdint.h>
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

//#define SINGLESYSTEM

/* Hardware Test enable*/
//#define HD_TEST

#define DEBUG_ENABLE

/* Flash Read Protection */
//#define FLASH_ENCRYP

#define R_wheel1        25.25f
#define R_wheel2        25.2f

#define ECD_RANGE      4096
#define dT 					   0.005f           //积分的步长
#define PI             3.1415926f       //圆周率的值
#define Temp_ctr       42             //控温的值    //大车上控制的温度 37 45 30
#define TempTable_max  50
#define TempTable_min  30
#define TIME_HEAT      5              //初始化时加热的时间值

#define BF_TH          2               //最下二乘法拟合的阶数
/* Exported macro ------------------------------------------------------------*/
/* L3GD20H 寄存器地址-----------------------*/
#define L3GD20H_WHO_AM_I								0x0F

#define L3GD20H_CTRL_REG1								0x20
#define L3GD20H_CTRL_REG2 							0x21
#define L3GD20H_CTRL_REG3 							0x22
#define L3GD20H_CTRL_REG4 							0x23
#define L3GD20H_CTRL_REG5 							0x24

#define L3GD20H_REFERENCE 							0x25
#define L3GD20H_OUT_TEMP 								0x26
#define L3GD20H_STATUS_REG 							0x27

#define L3GD20H_OUT_X_L 								0x28											/* X轴角速率数据，值为2的补码						 */
#define L3GD20H_OUT_X_H 								0x29
#define L3GD20H_OUT_Y_L 								0x2A											/* Y轴角速率数据，值为2的补码						 */
#define L3GD20H_OUT_Y_H 								0x2B
#define L3GD20H_OUT_Z_L 								0x2C											/* Z轴角速率数据，值为2的补码						 */
#define L3GD20H_OUT_Z_H 								0x2D

#define L3GD20H_FIFO_CTRL_REG 					0x2E
#define L3GD20H_FIFO_SRC_REG 						0x2F
#define L3GD20H_INT1_CFG 								0x30
#define L3GD20H_INT1_SRC 								0x31

#define L3GD20H_INT1_TSH_XH 						0x32
#define L3GD20H_INT1_TSH_XL 						0x33
#define L3GD20H_INT1_TSH_YH 						0x34
#define L3GD20H_INT1_TSH_YL 						0x35
#define L3GD20H_INT1_TSH_ZH 						0x36
#define L3GD20H_INT1_TSH_ZL 						0x37
#define L3GD20H_INT1_DURATION 					0x38
#define L3GD20H_LOW_ODR               	0x39


/* ICM20608G 陀螺仪寄存器地址--------------*/
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


/* Exported functions ------------------------------------------------------- */
typedef struct{
	float x;
	float y;
	float z;
}three_axis;

typedef struct{
	float pitch;
	float roll;
	float yaw;
}euler_angles;

typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;
}Quarternion;


#ifdef SINGLESYSTEM
#define ICM_ReadByte(address)            	SPI_Read(SPI2,GPIOB,GPIO_Pin_12,address)
#define ICM_WriteByte(address,value)     	SPI_Write(SPI2,GPIOB,GPIO_Pin_12,address,value)
#else
#define ICM_ReadByte(address)            	SPI_Read(SPI1,GPIOA,GPIO_Pin_4,address)
#define ICM_WriteByte(address,value)     	SPI_Write(SPI1,GPIOA,GPIO_Pin_4,address,value)
#endif
typedef enum{FALSE = 0,TRUE = !FALSE}BOOL;

#endif	// !__ALGORITHM_H

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
