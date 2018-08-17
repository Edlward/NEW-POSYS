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
#include "usart.h"
/* Private define ------------------------------------------------------------*/

#define   DUMMY_BYTE          0xFFFF      //哑元字节 SPI 读数据用

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

//转换成二进制数，帮助理解数据包组成
void sendBinary(uint32_t value)
{
	uint32_t a,b,k,i;
	int remainder[33];
	//定义了一个remainder数组，用来收集短除法除得的余数，栈倒序输出。
	a=value;
	k=0;
	while(a!=0){
		b=a/2;
		k++;
		remainder[k]=a-b*2;
		a=a/2;
	};
	for (i=k;i>=1;i--){
		USART_SendDataToDMA_USART6(remainder[i]+48);
	} 

	USART_SendDataToDMA_USART6('\t');
}
/**
 * @brief  读取TLE5012的寄存器
 * @param  commond：命令字
 * @param  reData：数据指针
 * @retval safety word
 */
 
uint16_t TLE5012ReadAbsPos_A(void)
{
	uint16_t u16Command = 0xFFFF;
	uint16_t u16Data = 0x0000;
	u16Command = READ_ANGLE_VALUE;
	TLE5012ReadReg(SPI3, u16Command, &u16Data);//返回safe
	u16Data = u16Data & 0x7FFF;
	
	//为了老版编码器芯片保持一致，都是顺时针转动为正而做的处理。如果不想与老板一致，坐标计算公式还有矫正平台测轮子半径部分矫正算法都要改。
	u16Data=0x7FFF-u16Data;
	
	//USART_OUTByDMAF(u16Data);
	//帮助理解数据包组成
	//sendBinary(u16Data);

	return u16Data;
}

uint16_t TLE5012ReadAbsPos_B(void)
{
	uint16_t u16Command = 0xFFFF;
	uint16_t u16Data = 0x0000;
	u16Command = READ_ANGLE_VALUE;
	TLE5012ReadReg(SPI2, u16Command, &u16Data);//返回safe
	u16Data = u16Data & 0x7FFF;
	
	u16Data=0x7FFF-u16Data;
	
	//USART_OUTByDMAF(u16Data);
	//帮助理解数据包组成
	//sendBinary(u16Data);
	//USART_EnterByDMA();
	
	return u16Data;
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
	
	//帮助理解数据包组成
//	uint32_t value=0;
//	value=(uint32_t)(((uint32_t)(*pu16Data)<<16)|(uint32_t)u16Safe);
//	sendBinary(value);
	
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
	
	if(SPIx==SPI2)
	{
		TLE5012_A_CS_DISABLE();											//禁止模块的片选信号
	}else if(SPIx==SPI3){
		TLE5012_B_CS_DISABLE();											//禁止模块的片选信号
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
		TLE5012_A_CS_DISABLE();											//禁止模块的片选信号
	}else if(SPIx==SPI3){
		TLE5012_B_CS_DISABLE();											//禁止模块的片选信号
	}

//	printf("\r\n @Sky [TLE5012ReadReg] RegData：0x%04X; SafeWord：0x%04X\r\n", *pu16Data, u16Safe);
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
	//PB15--MOSI输入模式                   
	GPIOB->MODER &= 0x3FFFFFFF; 
	GPIOB->MODER |= 0x00000000; 
}
static void	SPI2_TX_ON(void)
{
	//PB15--MOSI复用                            
	GPIOB->MODER &= 0x3FFFFFFF; 
	GPIOB->MODER |= 0x80000000; 
}
static void	SPI3_TX_OFF(void)
{         
	//PC12--MOSI输入模式                   
	GPIOC->MODER &= 0xFCFFFFFF; 
	GPIOC->MODER |= 0x00000000; 
}
static void	SPI3_TX_ON(void)
{
	//PC12--MOSI复用                            
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
		Delay_us(1);
		SPI_Cmd(SPI2,ENABLE);
		for(i=0;i<3;i++)
		{
			StartCount();
			while((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET)
			{
				if(getCount()>1000)
				{
					DeadWhileReport(36);
				}
			}
			EndCnt(); 
			buf[i] = SPI2->DR;
		}
	  GPIO_SetBits(GPIOB,GPIO_Pin_12);
		SPI_Cmd(SPI2,DISABLE);
	}
	else if(num==0)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_15);
		Delay_us(1);
		SPI_Cmd(SPI3,ENABLE);
		for(i=0;i<3;i++)
		{
			StartCount();
			while((SPI3->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET)
			{
				if(getCount()>1000)
				{
					DeadWhileReport(37);
				}
			}
			EndCnt(); 
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
	uint32_t  AbsEncData  = SPI_ReadAS5045All(num); //SPI读到的编码器的数据
	/*编码器数据*/
	int16_t   tmpAbs      = (AbsEncData >> 12) & 0X0FFF;
	/*发送来的奇偶校验位*/
	uint8_t evebParity = (AbsEncData >> 6) & 0x01;
	/*被校验数*/
	uint32_t parity = (AbsEncData>>7) & 0x1FFFF;
	/*计算来的奇偶校验位*/
	uint8_t evebParityCal = 0;
	/*获得标志位*/
	uint16_t MagSta = (AbsEncData >> 7) & 0x1F;
	static int count=0;
	while(1)
	{
		/*计算奇偶校验结果*/
		while (parity)
		{
			evebParityCal =!evebParityCal;
			parity = parity & (parity - 1);
		}
		/*如果奇偶校验成功，并且标志位正确*/
		if(evebParityCal==evebParity&&(MagSta==0x10||MagSta==0x13))
		{
			//跳出循环
			count=0;
			break;
		}
		//重新读取
		else
		{
			count++;
			/*再次获取24位数据*/
			AbsEncData  = SPI_ReadAS5045All(num);
			/*取出数据位*/
			tmpAbs      = (AbsEncData >> 12) & 0X0FFF;
			/*获得给出的奇偶校验位*/
			evebParity = (AbsEncData >> 6) & 0x01;
			/*获得被校验数*/
			parity = (AbsEncData>>7) & 0x1FFFF;
			/*获得标志位*/
			MagSta = (AbsEncData >> 7) & 0x1F;
			/*初始化奇偶校验结果*/
			evebParityCal = 0;
			/*防止多次读不出来*/
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
/*找到数组最小的值*/
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
	/*连读存储区*/
	uint16_t value[READ_NUM]={0};
	/*与上一刻值的差值*/
	int delValue[READ_NUM]={0};
	/*最终结果*/
	uint16_t endValue=0;
	static uint16_t endValueLast[2]={0,0};
	
	/*连续读三次*/
	for(int i=0;i<READ_NUM;i++)
		value[i]=SPI_ReadAS5045_Parity(num);
	
	/*三次与上一次的差值*/
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
	
	/*找到与上一次差值最小的值的序列号，并传入*/
	endValue= value[FindMin2(delValue)];
	
	if(num==0)
		endValueLast[0]=endValue;
	else if(num==1)
		endValueLast[1]=endValue;
	return endValue;
	
}
