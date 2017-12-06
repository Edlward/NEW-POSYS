/**
******************************************************************************
* @file    main.c
* @author  Summer
* @version V1.0.1
* @date    13-August-2017
* @brief   the start
******************************************************************************
* @attention
* Summer is the most handsome man !
*
**/
#include "misc.h"
#include "config.h"

AllPara_t allPara;

void init(void)
{
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
  /* 陀螺仪加热电阻PWM初始化--------------------------*/
  pwm_init(999, 83);//此时PWM的频率为84MHz/(83+1)/(999+1)=1KHz
  
	/* SPI初始化---------------------------------------*/
	//单轮模式时磁编码器的SPI初始化
	ICM_SPIInit();
	//单轮模式时陀螺仪的SPI初始化
	SPI2_Init();
	
  //片选的初始化
  CS_Config();
	
	USART1_Init(921600);

  Flash_Init();
	
	for(int gyro;gyro<GYRO_NUMBER;gyro++)
	{
		/*ICM20608G模块初始化*/
		ICM20608G_init(gyro);
		/*初始时不加热*/
		ICM_HeatingPower(gyro,0);
	}
	
	SetCommand(HEATING);
  driftCoffecientInit();
  TIM_Init(TIM2,999,83,0,0);					//主周期定时5ms
	
	while(!getTempInitSuces());
	
}


int main(void)
{
  init();
  static uint32_t cpuUsage;
	char readOrderLast=(char)-1;
  while(1)
  {
    while(getTimeFlag())
    {
////      				uint8_t test[3];
////      				test[0]=SPI_Read(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_WHO_AM_I); //测试ICM20608G，正确值为0XAF
//			//使数据能够同步，但是不同步情况很少
//			while(readOrderLast==getReadOrder());
//			readOrderLast=getReadOrder();
//			//AT指令处理
//			AT_CMD_Handle();
//      if(!(GetCommand()&CORRECT)){
//				if(GetCommand()&HEATING)
//				{
//					for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
//						temp_pid_ctr(gyro,allPara.GYRO_TemperatureAim[gyro]);
//				}
//				else
//				{
//					for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
//						temp_pid_ctr(gyro,allPara.GYRO_TemperatureAim[gyro]-0.5f);
//				}
//        //计算角度 
//        if(!RoughHandle())
//				  TemporaryHandle();
//        else {
//          if(GetCommand()&ACCUMULATE)
//					{
//            updateAngle();
//            calculatePos();	
//          }
//						#ifndef TEST_SUMMER
//						//串口被中断打断依然能正常发送（试验了几分钟）
//					//	 DataSend();
//						#endif
//        }
//      }
//      else{
//        UpdateVDoffTable();
//			}
//      //真实的占空比会等于cpuUsage或大于其一个单位
//      cpuUsage=getTimeCount();
    }
  }
}

