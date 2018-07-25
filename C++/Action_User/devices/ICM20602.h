/**
  ******************************************************************************
  * @file    ICM20602.h
  * @author  Luo Xiaoyi and Qiao Zhijian 
  * @version V1.0
  * @date    2017.3.13
  * @brief   This file contains the headers of usart.cpp
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
#ifndef __ICM20602_H
#define __ICM20602_H

/* C&C++ ---------------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"	 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/	 
/* Exported functions ------------------------------------------------------- */
#ifdef __cplusplus
}
/* Exported functions ------------------------------------------------------- */

#endif
#include "signalProcess.h"
#include "device.h"

#define ICM_GYRO_NUM		3

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

#define ICM_OVER_SAMPLE_NUM			5
class ICM20602_Gyro:public device<threeAxis,uint8_t>
{	
	private:
		void multiRead(uint8_t address,uint8_t *data,uint32_t len);
		threeAxis rateSeq[ICM_OVER_SAMPLE_NUM]={0.f};
	  float tempSeq[ICM_OVER_SAMPLE_NUM]={0.f};
	public:
		int16_t temp;
		class BaisHandle<threeAxis> baisHandle;
		ICM20602_Gyro(SPI_TypeDef* SPI,GPIO_TypeDef* GPIO,uint16_t Pin):device(SPI,GPIO,Pin){};
		virtual ~ICM20602_Gyro()=default;
		virtual uint8_t 	rawDataRead(uint8_t address);
		virtual void 		 	rawDataWrite(uint8_t address,uint8_t value);
		virtual void 		 	init(void);
		virtual void     	UpdateData(void);
		virtual void 			UpdateBais(void);
};
ICM20602_Gyro& getICM20602_Gyro(void);
#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
