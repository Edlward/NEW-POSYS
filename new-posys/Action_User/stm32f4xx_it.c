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

#include "config.h"
#include "DataRecover.h"
#include "iwdg.h"
/*************定时器2******start************/
//每1ms调用一次  用于读取编码器的值和计算坐标


extern AllPara_t allPara;
static uint32_t timeCount=0;
static uint8_t timeFlag=1;
static char readOrder=0;
void TIM2_IRQHandler(void)
{
  double gyr_temp[GYRO_NUMBER][AXIS_NUMBER];
  double acc_temp[GYRO_NUMBER][AXIS_NUMBER];
  float  temp_temp[GYRO_NUMBER];
  static double gyro_sum[GYRO_NUMBER][AXIS_NUMBER];
  static double acc_sum[GYRO_NUMBER][AXIS_NUMBER];
  static float  temp_sum[GYRO_NUMBER];
  static uint32_t timeCnt=0;
	static int badGyro[AXIS_NUMBER][GYRO_NUMBER]={0};
	int axis = 0;
	int gyro = 0;
	
  if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
  {	
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    timeCount++;
    timeCnt++;
    if(timeCount>=PERIOD*1000)
    {
      timeCount=0;
      timeFlag=1;
			allPara.cpuUsage++;
    }
		/*读取角速度，温度的数据，并进行累加*/
		for(gyro=0;gyro<GYRO_NUMBER;gyro++)
		{
      icm_update_gyro_rate(gyro);
			icm_update_acc(gyro);
			icm_update_temp(gyro);
			icm_read_temp(&(temp_temp[gyro]));
      icm_read_gyro_rate(gyr_temp[gyro]);
			icm_read_accel_acc(acc_temp[gyro]);
			for(axis=0;axis<AXIS_NUMBER;axis++)
			{
				gyro_sum[gyro][axis]=gyro_sum[gyro][axis]+gyr_temp[gyro][axis];
				acc_sum[gyro][axis]=acc_sum[gyro][axis]+acc_temp[gyro][axis];
			}

      temp_sum[gyro]=temp_sum[gyro]+temp_temp[gyro];
		}
		/*确定加热温度*/
		if(!getTempInitSuces())
			HeatingInit(temp_temp);
		
	  if(timeCnt==5)
		{	
			
			//第一次判断静止时可以不用判断角速度
			allPara.codeData[0]=SPI_ReadAS5045(0);
			allPara.codeData[1]=SPI_ReadAS5045(1);
			double percentages[3][3]={
			0.33333333,0.33333333,0.33333333,
			0.33333333,0.33333333,0.33333333,
		//	0.33333333,0.33333333,0.33333333};
			0.387132729300339,0.255395342771687,0.357471927927974	};
			readOrder++;
      timeCnt=0;
			//取得5ms的原始数据总和的平均数
			for(gyro=0;gyro<GYRO_NUMBER;gyro++)
			{
				allPara.GYRO_Temperature[gyro]=temp_sum[gyro]/5.0f;
				for(axis=0;axis<AXIS_NUMBER;axis++)
				{
					allPara.GYROWithoutRemoveDrift[gyro][axis]=gyro_sum[gyro][axis]/5.0;	
					allPara.ACC_Raw[gyro][axis]=acc_sum[gyro][axis]/5.0;
					acc_sum[gyro][axis]=0.0;
				}
			}
			
			//温度进行低通滤波
			for(gyro=0;gyro<GYRO_NUMBER;gyro++)
				allPara.GYRO_Temperature[gyro]=LowPassFilter(allPara.GYRO_Temperature[gyro],gyro)/100.f;
			//去除温飘
			for(gyro=0;gyro<GYRO_NUMBER;gyro++)
			{
				temp_sum[gyro]=0.f;
				for(axis=0;axis<AXIS_NUMBER;axis++)
				{
					allPara.GYRORemoveDrift[gyro][axis]=allPara.GYROWithoutRemoveDrift[gyro][axis];//-allPara.driftCoffecient[gyro][axis]*(allPara.GYRO_Temperature[gyro]);
					gyro_sum[gyro][axis]=0.0;
				}
			}
			//对三个陀螺仪的数据取平均数
			for(axis=0;axis<GYRO_NUMBER;axis++)
			{
				for(gyro=0;gyro<AXIS_NUMBER;gyro++)
				{
					/*正常情况小于这个值，没啥用，不正常时，值是1/131=0.0076*/
					/*满足条件时，把该项变成0*/
					if(fabs(allPara.GYRORemoveDrift[gyro][axis])<0.01)
					{
						badGyro[axis][gyro]++;
					}
					else
						badGyro[axis][gyro]=0;
					if(badGyro[axis][gyro]>10)
						percentages[axis][gyro]=0.0;
				}
			}
			for(axis=0;axis<GYRO_NUMBER;axis++)
			{
				double sum=percentages[axis][0]+percentages[axis][1]+percentages[axis][2];
				for(gyro=0;gyro<AXIS_NUMBER;gyro++)
				{
					/*为了防止所有陀螺仪都坏掉*/
					if(sum>0.001)
					{
						percentages[axis][gyro]=percentages[axis][gyro]/sum;
					}
					else
					{
						percentages[axis][gyro]=0.0;
					}
				}
			}
				
//			allPara.GYRO_Aver[0]=allPara.GYRORemoveDrift[0][0]*percentages[0][0]+allPara.GYRORemoveDrift[1][0]*percentages[0][1]+allPara.GYRORemoveDrift[2][0]*percentages[0][2];
//			allPara.GYRO_Aver[1]=allPara.GYRORemoveDrift[0][1]*percentages[1][0]+allPara.GYRORemoveDrift[1][1]*percentages[1][1]+allPara.GYRORemoveDrift[2][1]*percentages[1][2];
			allPara.GYRO_Aver[2]=allPara.GYRORemoveDrift[0][2]*percentages[2][0]+allPara.GYRORemoveDrift[1][2]*percentages[2][1]+allPara.GYRORemoveDrift[2][2]*percentages[2][2];
			//前面的计算，如果真有一个坏了，角速度会突变
//			allPara.GYRO_Aver[2]=allPara.GYRORemoveDrift[0][2]*0.387132729300339+allPara.GYRORemoveDrift[1][2]*0.255395342771687+allPara.GYRORemoveDrift[2][2]*0.357471927927974;
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


uint32_t startCnt=0;
uint32_t Cnt=0;

void TIM7_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM7, TIM_IT_Update)==SET)
  {	
		if(startCnt==1)
			Cnt++;
		//printf("%d\r\n",Cnt);
		IWDG_Feed();
		
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

void StartCount(void)
{
	//printf("%d\t",startCnt);
	startCnt=1;
	Cnt=0;
}

uint32_t returnEndUs(void)
{
	uint32_t	end;
	end=Cnt*100;
	Cnt=0;
	startCnt=0;
	return end;
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
//	  static uint32_t r_sp ;
//		/*判断发生异常时使用MSP还是PSP*/
//		if(__get_PSP()!=0x00) //获取SP的值
//			r_sp = __get_PSP(); 
//		else
//			r_sp = __get_MSP(); 
//		/*因为经历中断函数入栈之后，堆栈指针会减小0x10，所以平移回来（可能不具有普遍性）*/
//		r_sp = r_sp+0x10;
	debugsend(1.f,2.f,3.f,4.f,5.f,6.f);
	if(allPara.resetTime<=500)
	{
		FindResetTime();
		/*不擦出的情况下写flash大约只要70us*/
		
		allPara.isReset=1;
		
		WriteFlashData(allPara,allPara.resetTime);
		
		//STMFLASH_Read(&allPara,allPara.resetTime);
	}
		
		
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
//		/*串口发数通知*/
//		USART_OUT(USART1,"\r\nHardFault");
//  	char sPoint[2]={0};
//		USART_OUT(USART1,"%s","0x");
//		/*获取出现异常时程序的地址*/
//		for(int i=3;i>=-28;i--){
//			Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
//			USART_OUT(USART1,"%s",sPoint);
//			if(i%4==0)
//				USART_Enter();
//		}
//		/*发送回车符*/
//		USART_Enter();
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

