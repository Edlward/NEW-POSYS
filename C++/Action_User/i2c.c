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
#include "i2c.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stdint.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/**
  * @brief  
  * @param  
  * @retval none
  */
void I2C2_Init(void)
{
	I2C_InitTypeDef  I2C_InitStructure; 
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);  

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_I2C2);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	
	
  I2C_DeInit(I2C2); 
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; 
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;  
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;  //100k  
	I2C_Cmd(I2C2, ENABLE); 
	I2C_Init(I2C2, &I2C_InitStructure); 
  //I2C_AcknowledgeConfig(I2C2, ENABLE);
}
#define SLAVE_Write_Address  0x6a
#define SLAVE_Read_Address   0x6a
void I2C_write_onebyte(u16 WriteAddr,u8 DataToWrite)
{
	 while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)); 
	
	  I2C_AcknowledgeConfig(I2C2,ENABLE);//循环读写不可少
	
		I2C_GenerateSTART(I2C2, ENABLE);
	 /*      EV5          */
	 while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));/*EV5,主模式*/
   I2C_Send7bitAddress(I2C2,SLAVE_Write_Address,I2C_Direction_Transmitter); 
	
	/*     EV6   卡在这里是因为上面的从机地址没写对  */
  while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C2,WriteAddr);
	
	/*       EV8   !!    !!!  */
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING)); 
	I2C_SendData(I2C2,DataToWrite);
	
	/*        EV8_2   !!     */
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2C2,ENABLE);
}


u8 I2C_read_onebyte(u16 ReadAddr)
{
	  u8 temp=0;
	 while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
		
   I2C_GenerateSTART(I2C2, ENABLE);
	
	 //EV5事件
   while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));
   I2C_Send7bitAddress(I2C2,SLAVE_Write_Address, I2C_Direction_Transmitter); 
	
	 //EV6 EV8_1事件（该事件判断同时判断了EV8_1事件）   
	 while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));	 
	 I2C_SendData(I2C2,ReadAddr); 
	/*       EV8   !!    !!!  */
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); 
	 I2C_GenerateSTART(I2C2, ENABLE);
		 
	//EV5事件
   while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));
   I2C_Send7bitAddress(I2C2, SLAVE_Write_Address, I2C_Direction_Receiver ); 
	//!!
	//EV6事件
	 while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
		 
  //EV7事件
	 while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED));

	 temp= I2C_ReceiveData(I2C2);		 

	 I2C_NACKPositionConfig(I2C2,DISABLE);   //失能ACK

	 I2C_GenerateSTOP(I2C2,ENABLE);  
	        
		return temp;
}
