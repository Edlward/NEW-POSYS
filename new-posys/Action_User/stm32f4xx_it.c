/**
******************************************************************************
* @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
* @author  MCD Application Team
* @version V1.0.1
* @date    13-April-2012
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and 
*          peripherals interrupt service routine.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "flash.h"
#include "config.h"
#include "icm_20608_g.h"
#include "buildExcel.h"
#include "spi.h"
#include "figureAngle.h"
#include "ADXRS453Z.h"

/*************定时器2******start************/
//每1ms调用一次  用于读取编码器的值和计算坐标

//int posx,posy;
static uint32_t timeCount=0;
static uint8_t timeFlag=0;
gyro_t gyr_data;
gyro_t acc_data;
float temp_icm;
extern uint16_t data[2];
static char readOrder=0;
void TIM2_IRQHandler(void)
{
	
  gyro_t gyr_temp;
	#ifndef ADXRS453Z
  gyro_t act_temp;
  static gyro_t act_sum;
	#endif
  float  temp_temp;
  static uint32_t timeCnt=0;
  static gyro_t gyro_sum;
  static float  temp_sum;
  if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
  {	
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    timeCount++;
    timeCnt++;
    if(timeCount>=PERIOD*1000)
    {
      timeCount=0;
      timeFlag=1;
    }
      icm_update_gyro_rate();
			icm_update_temp();
			icm_read_temp(&temp_temp);
      icm_read_gyro_rate(&gyr_temp);
      gyro_sum.No1.x=gyro_sum.No1.x+gyr_temp.No1.x;
      gyro_sum.No1.y=gyro_sum.No1.y+gyr_temp.No1.y;
      gyro_sum.No1.z=gyro_sum.No1.z+gyr_temp.No1.z;
      temp_sum=temp_sum+temp_temp;
			if(timeCnt==4){
				//放到中断里
				data[0]=SPI_ReadAS5045(0);
				data[1]=SPI_ReadAS5045(1);
			}
      if(timeCnt==5){
				readOrder++;
        timeCnt=0;
        gyr_data.No1.x=gyro_sum.No1.x/5.f;
        gyr_data.No1.y=gyro_sum.No1.y/5.f;
        gyr_data.No1.z=gyro_sum.No1.z/5.f;
				temp_icm=temp_sum/5.f;
				temp_icm=KalmanFilterT(temp_icm);
        gyro_sum.No1.x=0.f;
        gyro_sum.No1.y=0.f;
        gyro_sum.No1.z=0.f;
				temp_sum=0.f;
      }
  }
	else{
		USART_OUT(USART1,"TIM2 error");
	}
}

char getReadOrder(void){
	return readOrder;
}

uint8_t getTimeFlag(void)
{
  uint8_t nowFlag;
  nowFlag=timeFlag;
  
  if(nowFlag)
  {
    timeFlag=0;
    return 1;
  }
  return 0;
}

uint32_t getTimeCount(void)
{
  return (timeCount);
}

//定时器1  
void TIM1_UP_TIM10_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM1, TIM_IT_Update)==SET)    
  {                                                
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  }
}

//定时器8  
void TIM8_UP_TIM13_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM8, TIM_IT_Update)==SET)    
  {                                                
    TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
  }
}

/********************************************************/
/*****************普通定时TIM5*****Start*****************/
void TIM5_IRQHandler(void)
{
  
  if(TIM_GetITStatus(TIM5, TIM_IT_Update)==SET)    
  {              
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
  }
}

void TIM3_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET)    
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	
  }
}



//定时器4  
void TIM4_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)
  {                                  
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  }
}

void UART5_IRQHandler(void)
{
  //uint8_t data = 0;
  if(USART_GetITStatus(UART5, USART_IT_RXNE)==SET)   
  {
    USART_ClearITPendingBit( UART5,USART_IT_RXNE);
    //data=USART_ReceiveData(UART5);
    
  }
  
}

void USART3_IRQHandler(void)
{
  //uint8_t data = 0;
  if(USART_GetITStatus(USART3, USART_IT_RXNE)==SET)   
  {
    USART_ClearITPendingBit( USART3,USART_IT_RXNE);
    //data=USART_ReceiveData(USART3);
  }
  
}


/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/	

void HardFault_Handler(void)
{  
	  static uint32_t r_sp ;
		/*判断发生异常时使用MSP还是PSP*/
		if(__get_PSP()!=0x00) //获取SP的值
			r_sp = __get_PSP(); 
		else
			r_sp = __get_MSP(); 
		/*因为经历中断函数入栈之后，堆栈指针会减小0x10，所以平移回来（可能不具有普遍性）*/
		r_sp = r_sp+0x10;
		/*串口发数通知*/
		USART_OUT(USART1,"HardFault");
  	char sPoint[2]={0};
		USART_OUT(USART1,"%s","0x");
		/*获取出现异常时程序的地址*/
		for(int i=3;i>=-28;i--){
			Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
			USART_OUT(USART1,"%s",sPoint);
			if(i%4==0)
				USART_Enter();
		}
		/*发送回车符*/
		USART_Enter();
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

