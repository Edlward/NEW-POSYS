/**
  ******************************************************************************
  * @file    
  * @author Billy
  * @version V2.0
  * @date    01-May-2018
  * @brief   This file contains all the functions prototypes for orthogonal odom 
  *          information provided by two megnetic encoders AS5045B
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 Action Robot</center></h2>
  *
  * @note
  ******************************************************************************
  */ 

#include <stdint.h>
#include <stdlib.h>
#include "timer.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "odom.h"
#include "spi.h"
/* Private define ------------------------------------------------------------*/

#define   DUMMY_BYTE          0xFFFF      //��Ԫ�ֽ� SPI ��������

/* Private variables ---------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

static void	TLE5012_A_CS_ENABLE(void);
static void	TLE5012_A_CS_DISABLE(void);
static void	TLE5012_B_CS_ENABLE(void);
static void	TLE5012_B_CS_DISABLE(void);
static void	SPI2_TX_OFF(void);
static void	SPI2_TX_ON(void);
static void	SPI3_TX_OFF(void);
static void	SPI3_TX_ON(void);
uint16_t	TLE5012WriteReg(SPI_TypeDef* SPIx, uint16_t u16Command, uint16_t* pu16Data);

/**
 * @brief  ��ȡTLE5012�ļĴ���
 * @param  commond��������
 * @param  reData������ָ��
 * @retval safety word
 */
 
uint16_t TLE5012ReadAbsPos_A(void)
{
	uint16_t u16Command = 0xFFFF;
	uint16_t u16Data = 0x0000;
	u16Command = READ_ANGLE_VALUE;
	TLE5012ReadReg(SPI2, u16Command, &u16Data);//����safe
	u16Data = u16Data & 0x7FFF;
	return ((int16_t)(u16Data << 1)) >> 1;
}
uint16_t TLE5012ReadAbsPos_B(void)
{
	uint16_t u16Command = 0xFFFF;
	uint16_t u16Data = 0x0000;
	u16Command = READ_ANGLE_VALUE;
	TLE5012ReadReg(SPI3, u16Command, &u16Data);//����safe
	u16Data = u16Data & 0x7FFF;
	return ((int16_t)(u16Data << 1)) >> 1;
}

uint16_t	TLE5012ReadReg(SPI_TypeDef* SPIx, uint16_t u16Command, uint16_t* pu16Data)
{
	uint16_t u16Safe = 0;
	
	if(SPIx==SPI2)
	{
		SPI2_TX_ON();
		TLE5012_A_CS_ENABLE();
	}else if(SPIx==SPI3){
		SPI3_TX_ON();
		TLE5012_B_CS_ENABLE();
	}
	
	*pu16Data = SPIx_RdWr_HalfWord(SPIx, u16Command);	
	
	if(SPIx==SPI2)
	{
		SPI2_TX_OFF();
	}else if(SPIx==SPI3){
		SPI3_TX_OFF();
	}
	
	*pu16Data = SPIx_RdWr_HalfWord(SPIx, DUMMY_BYTE);	
	u16Safe = SPIx_RdWr_HalfWord(SPIx, DUMMY_BYTE);
	
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
	
	if(SPIx==SPI2)
	{
		TLE5012_A_CS_DISABLE();											//��ֹģ���Ƭѡ�ź�
	}else if(SPIx==SPI3){
		TLE5012_B_CS_DISABLE();											//��ֹģ���Ƭѡ�ź�
	}

	return u16Safe;
}

uint16_t	TLE5012WriteReg(SPI_TypeDef* SPIx, uint16_t u16Command, uint16_t* pu16Data)
{
	uint16_t	u16Safe = 0;

	if(SPIx==SPI2)
	{
		SPI2_TX_ON();
		TLE5012_A_CS_ENABLE();
	}else if(SPIx==SPI3){
		SPI3_TX_ON();
		TLE5012_B_CS_ENABLE();
	}

	u16Safe = SPIx_RdWr_HalfWord(SPIx, u16Command);	
	u16Safe = SPIx_RdWr_HalfWord(SPIx, *pu16Data);
	
	if(SPIx==SPI2)
	{
		SPI2_TX_OFF();
	}else if(SPIx==SPI3){
		SPI3_TX_OFF();
	}
	
	u16Safe = SPIx_RdWr_HalfWord(SPIx, DUMMY_BYTE);
	/* Wait until SPIx TXD RXD Complete */
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
	
	if(SPIx==SPI2)
	{
		TLE5012_A_CS_DISABLE();											//��ֹģ���Ƭѡ�ź�
	}else if(SPIx==SPI3){
		TLE5012_B_CS_DISABLE();											//��ֹģ���Ƭѡ�ź�
	}

//	printf("\r\n @Sky [TLE5012ReadReg] RegData��0x%04X; SafeWord��0x%04X\r\n", *pu16Data, u16Safe);
	return u16Safe;
}


static void	TLE5012_A_CS_ENABLE(void)
{
//	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
}

static void	TLE5012_A_CS_DISABLE(void)
{
//	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}
static void	TLE5012_B_CS_ENABLE(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
}

static void	TLE5012_B_CS_DISABLE(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
}
static void	SPI2_TX_OFF(void)
{         
	//PB15--MOSI����ģʽ                   
	GPIOB->MODER &= 0x3FFFFFFF; 
	GPIOB->MODER |= 0x00000000; 
}
static void	SPI2_TX_ON(void)
{
	//PB15--MOSI����                            
	GPIOB->MODER &= 0x3FFFFFFF; 
	GPIOB->MODER |= 0x80000000; 
}
static void	SPI3_TX_OFF(void)
{         
	//PC12--MOSI����ģʽ                   
	GPIOC->MODER &= 0xFCFFFFFF; 
	GPIOC->MODER |= 0x00000000; 
}
static void	SPI3_TX_ON(void)
{
	//PC12--MOSI����                            
	GPIOC->MODER &= 0xFCFFFFFF; 
	GPIOC->MODER |= 0x02000000; 
}

uint32_t SPI_ReadAS5045All(uint8_t num)
{
	uint8_t  buf[3]={0,0,0},i=0;
	uint32_t AS5045_Val=0;
	
	if(num==1)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		SPI_Cmd(SPI2,ENABLE);
		for(i=0;i<3;i++)
		{
			while((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET){}
				buf[i] = SPI2->DR;
		}
	  GPIO_SetBits(GPIOB,GPIO_Pin_12);
		SPI_Cmd(SPI2,DISABLE);
	}
	else if(num==0)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_15);
		SPI_Cmd(SPI3,ENABLE);
		for(i=0;i<3;i++)
		{
			while((SPI3->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET){}
				buf[i] = SPI3->DR;
		}
	  GPIO_SetBits(GPIOA,GPIO_Pin_15);
		SPI_Cmd(SPI3,DISABLE);
	}
	
	delay_us(15);
	
  AS5045_Val = (((uint32_t)buf[0]<<17) | ((uint32_t)buf[1]<<9) | ((uint32_t)buf[2]<<1) | (uint32_t)0);
	
	return AS5045_Val;
}

uint16_t SPI_ReadAS5045_Parity(uint8_t num)
{
	uint32_t  AbsEncData  = SPI_ReadAS5045All(num); //SPI�����ı�����������
	/*����������*/
	int16_t   tmpAbs      = (AbsEncData >> 12) & 0X0FFF;
	/*����������żУ��λ*/
	uint8_t evebParity = (AbsEncData >> 6) & 0x01;
	/*��У����*/
	uint32_t parity = (AbsEncData>>7) & 0x1FFFF;
	/*����������żУ��λ*/
	uint8_t evebParityCal = 0;
	/*��ñ�־λ*/
	uint16_t MagSta = (AbsEncData >> 7) & 0x1F;
	static int count=0;
	while(1)
	{
		/*������żУ����*/
		while (parity)
		{
			evebParityCal =!evebParityCal;
			parity = parity & (parity - 1);
		}
		/*�����żУ��ɹ������ұ�־λ��ȷ*/
		if(evebParityCal==evebParity&&(MagSta==0x10||MagSta==0x13))
		{
			//����ѭ��
			count=0;
			break;
		}
		//���¶�ȡ
		else
		{
			count++;
			/*�ٴλ�ȡ24λ����*/
			AbsEncData  = SPI_ReadAS5045All(num);
			/*ȡ������λ*/
			tmpAbs      = (AbsEncData >> 12) & 0X0FFF;
			/*��ø�������żУ��λ*/
			evebParity = (AbsEncData >> 6) & 0x01;
			/*��ñ�У����*/
			parity = (AbsEncData>>7) & 0x1FFFF;
			/*��ñ�־λ*/
			MagSta = (AbsEncData >> 7) & 0x1F;
			/*��ʼ����żУ����*/
			evebParityCal = 0;
			/*��ֹ��ζ�������*/
			if(count>3)
			{
				count=0;
				break;
			}
		}
	}
	return tmpAbs;
}

#define READ_NUM	3
/*�ҵ�������С��ֵ*/
uint16_t FindMin2(int codes[READ_NUM])
{
	uint16_t Min=codes[0]; 
	uint16_t index = 0;
  for(int i=1;i<READ_NUM;i++)
	{
		if(codes[i]<Min) 
		{
			Min=codes[i];
			index=i;
		}
	}
	return index;
}


uint16_t SPI_ReadAS5045(uint8_t num)
{
	/*�����洢��*/
	uint16_t value[READ_NUM]={0};
	/*����һ��ֵ�Ĳ�ֵ*/
	int delValue[READ_NUM]={0};
	/*���ս��*/
	uint16_t endValue=0;
	static uint16_t endValueLast[2]={0,0};
	
	/*����������*/
	for(int i=0;i<READ_NUM;i++)
		value[i]=SPI_ReadAS5045_Parity(num);
	
	/*��������һ�εĲ�ֵ*/
	for(int i=0;i<READ_NUM;i++)
	{
		if(num==0)
			delValue[i]=(value[i]-endValueLast[0]);
		else if(num==1)
			delValue[i]=(value[i]-endValueLast[1]);
		if(delValue[i]>2048)
			delValue[i]-=4096;
		if(delValue[i]<-2048)
			delValue[i]+=4096;
		delValue[i]=abs(delValue[i]);
	}
	
	/*�ҵ�����һ�β�ֵ��С��ֵ�����кţ�������*/
	endValue= value[FindMin2(delValue)];
	
	if(num==0)
		endValueLast[0]=endValue;
	else if(num==1)
		endValueLast[1]=endValue;
	return endValue;
	
}
