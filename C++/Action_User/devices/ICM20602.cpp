/**
  ******************************************************************************
  * @file    ICM20602.cpp
  * @author  Luo Xiaoyi and Qiao Zhijian 
  * @version V1.0
  * @date    2017.3.13
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
#include "ICM20602.h"
#include "spi.h"
#include "main.h"
#include "stm32f4xx_spi.h"
#include "map"
#include "timer.h"
#include "signalProcess.h"
#include "usart.h"
#include "device.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
#define HARDWARE_TEST_REG 0x75
#define HARDWARE_TEST_VAL 0x12

#define DATA_AVAIABLE ((rawDataRead(0x27)&0x08)>>3)

/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
#ifdef ICM_20602_USED_1
static ICM20602_Gyro ICM20602_Gyro1(SPI1,GPIOA,GPIO_Pin_1);
//static deviceICM20602 ICM20602_Acc1(SPI1,GPIOA,GPIO_Pin_1);
#endif
#ifdef ICM_20602_USED_2
static ICM20602_Gyro ICM20602_Gyro2(SPI1,GPIOA,GPIO_Pin_2);
//static deviceICM20602 ICM20602_Acc2(SPI1,GPIOA,GPIO_Pin_2);
#endif
#ifdef ICM_20602_USED_3
static ICM20602_Gyro ICM20602_Gyro3(SPI1,GPIOC,GPIO_Pin_6);
//static deviceICM20602 ICM20602_Acc3(SPI1,GPIOC,GPIO_Pin_6);
#endif

static ICM20602_Gyro* pICM20602_Gyro_Array[ICM_GYRO_NUM];

uint8_t ICM20602_Gyro:: instanceNum=0;

ICM20602_Gyro::ICM20602_Gyro(SPI_TypeDef* SPI,GPIO_TypeDef* GPIO,uint16_t Pin)
{
  csGPIOx=GPIO;
	csGPIO_Pin=Pin;
	SPIx=SPI;
	if(instanceNum<ICM_GYRO_NUM)
		pICM20602_Gyro_Array[instanceNum]=this;
	else
		while(1);
	instanceNum++;
}
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
static void  changeSPIconfigure(SPI_TypeDef* SPIx)
{
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_Cmd(SPIx,DISABLE);
	SPI_I2S_DeInit(SPIx);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;											   
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;														
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																				
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;			
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;									
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPIx, &SPI_InitStructure);
	SPI_Cmd(SPIx, ENABLE);
	
}
void ICM20602_Gyro::multiRead(uint8_t address,uint8_t *data,uint32_t len)
{
	changeSPIconfigure(SPIx);
	SPI_MultiRead(SPIx,csGPIOx,csGPIO_Pin,address,data,len);
}
/* Exported function prototypes -----------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/
ICM20602_Gyro** getICM20602_Gyro(void)
{	
	return pICM20602_Gyro_Array;
}
uint8_t 	ICM20602_Gyro::rawDataRead(uint8_t address)
{
	changeSPIconfigure(SPIx);
	return SPI_Read(SPIx,csGPIOx,csGPIO_Pin,address);
}
void 		ICM20602_Gyro::rawDataWrite(uint8_t address,uint8_t value)
{
	changeSPIconfigure(SPIx);
	SPI_Write(SPIx,csGPIOx,csGPIO_Pin,address,value);
}
void 		ICM20602_Gyro::init(void)
{
	#define MAX_STARTUP_FROM_SLEEP_MAX_TIME (40)
	#define MAX_POWER_RAMP_TIME (100)
	#define MAX_REGISTER_STARTUP_TIME (100)

	//create map  note: this is an ordered sequence
	std::map<uint8_t,uint8_t> regWriteCheck;
	
	/*first: register address; second: value written*/
	regWriteCheck[ICM20608G_GYRO_CONFIG]=0x00;
	#ifdef	AUTOCAR
	regWriteCheck[ICM20608G_CONFIG]=0x00;
	regWriteCheck[ICM20608G_SMPLRT_DIV]=0x07;
	#else
	regWriteCheck[ICM20608G_CONFIG]=0x06;
	regWriteCheck[ICM20608G_SMPLRT_DIV]=0x00;
	#endif
	regWriteCheck[ICM20608G_ACCEL_CONFIG]=0x00;
	regWriteCheck[ICM20608G_ACCEL_CONFIG2]=0x02;
	regWriteCheck[ICM20608G_SIGNAL_PATH_RESET]=0x00;
	regWriteCheck[ICM20608G_USER_CTRL]=0x10;
	regWriteCheck[ICM20608G_LP_MODE_CFG]=0x00;
 	regWriteCheck[ICM20608G_FIFO_EN]=0x00;
	regWriteCheck[ICM20608G_ACCEL_WOM_THR]=0x00;
	regWriteCheck[ICM20608G_INT_PIN_CFG]=0x00;
	regWriteCheck[ICM20608G_INT_ENABLE]=0x00;
	regWriteCheck[ICM20608G_ACCEL_INTEL_CTRL]=0x00;
	regWriteCheck[ICM20608G_PWR_MGMT_2]=0x00;
	
  Delay_ms(MAX_POWER_RAMP_TIME + MAX_REGISTER_STARTUP_TIME);							//in consideration of worse case, we need wait this much time
	
	rawDataWrite(ICM20608G_PWR_MGMT_1,0x80);																//Wake up chip from sleep mode,enable temperature sensor
	
  Delay_ms(MAX_POWER_RAMP_TIME + MAX_REGISTER_STARTUP_TIME);							//in consideration of worse case, we need wait this much time
	
	while(rawDataRead(ICM20608G_PWR_MGMT_1)!=0x00)
	{
		rawDataWrite(ICM20608G_PWR_MGMT_1,0x00);	
		Delay_ms(1);
	}
	
	while(rawDataRead(HARDWARE_TEST_REG)!=HARDWARE_TEST_VAL)
	{
		Delay_ms(1);
	}
	
	/*loop through the map, and write the registers*/
	for(std::map<uint8_t,uint8_t>::iterator iter=regWriteCheck.begin();iter!=regWriteCheck.end();iter++)
	{
		rawDataWrite(iter->first,iter->second);
		Delay_ms(1);
	}
	
	/*check the validity of writing registers*/
	for(std::map<uint8_t,uint8_t>::iterator iter=regWriteCheck.begin();iter!=regWriteCheck.end();iter++)
	{
		//if different
		if(rawDataRead(iter->first)!=iter->second)
		{
			/*check hardware recognition*/
			if(rawDataRead(HARDWARE_TEST_REG)!=HARDWARE_TEST_VAL)
				while(1);
			else
				init();
		}
	}
	Delay_ms(MAX_STARTUP_FROM_SLEEP_MAX_TIME);
}
void    ICM20602_Gyro::UpdateData(void)
{
	union
	{
		uint8_t byte_2[2];
		uint8_t byte_6[6];
	}byte={0};
	
	int16_t tempVal=0;
	float tempValF=0.f;
	threeAxis tempVal_t={0.f};
	
	/*read temperature*/
	multiRead(ICM20608G_TEMP_OUT_H,byte.byte_2,2);
	tempValF=25.0f+static_cast<float>((byte.byte_2[0] << 8) | byte.byte_2[1])/ 326.8f;
	//right shift temperature array
	shiftRightData(tempSeq,ICM_OVER_SAMPLE_NUM);
	//update data at leftmost end
	tempSeq[0]=tempValF;
	//make mean temSeq as the real temp
	temp=meanData(tempSeq,ICM_OVER_SAMPLE_NUM);
	
	/*read raw datas of angular velocity*/
	multiRead(ICM20608G_GYRO_XOUT_H,byte.byte_6,6);
	/*process raw datas*/
	tempVal=(static_cast<uint16_t>(byte.byte_6[0])<<8)|(static_cast<uint16_t>(byte.byte_6[1]));
	tempVal_t.y=-tempVal/SCALE_FACTOR;
	tempVal=(static_cast<uint16_t>(byte.byte_6[2])<<8)|(static_cast<uint16_t>(byte.byte_6[3]));
	tempVal_t.x=-tempVal/SCALE_FACTOR;
	tempVal=(static_cast<uint16_t>(byte.byte_6[4])<<8)|(static_cast<uint16_t>(byte.byte_6[5]));
	tempVal_t.z=-tempVal/SCALE_FACTOR;
	
	/*exchange and convert because the installed locations of gyroscopes differ from one another*/
	switch(csGPIO_Pin)
	{
		case GPIO_Pin_1:
			tempValF = tempVal_t.x;
			tempVal_t.x = tempVal_t.y;
			tempVal_t.y = -tempValF;
			break;
		case GPIO_Pin_6:
			tempValF = tempVal_t.x;
			tempVal_t.x = tempVal_t.y;
			tempVal_t.y = -tempValF;
			break;
	}
	
	//right shift one unit
	shiftRightData(rateSeq,ICM_OVER_SAMPLE_NUM);
	//update data at leftmost end
	rateSeq[0]=tempVal_t;
	//make mean rateSeq as the real rate
	val=meanData(rateSeq,ICM_OVER_SAMPLE_NUM);
	
}

void ICM20602_Gyro:: UpdateBais(void)
{
	
}

uint8_t ICM20602_Gyro:: getInstanceNum(void)
{
	return instanceNum;
};

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
