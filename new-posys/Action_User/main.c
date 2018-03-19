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
#include "DataRecover.h"
#include "iwdg.h"
AllPara_t allPara;

void init(void)
{
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	
  TIM_Init(TIM7,99,83,0,0);					//主周期定时5ms
	//StartCount();
	SoftWareReset();
	
  /* 陀螺仪加热电阻PWM初始化--------------------------*/
  pwm_init(999, 83);//此时PWM的频率为84MHz/(83+1)/(999+1)=1KHz
  
	/* SPI初始化---------------------------------------*/
	//单轮模式时磁编码器的SPI初始化
	ICM_SPIInit();
	//单轮模式时陀螺仪的SPI初始化
	SPI2_Init();
	
  //片选的初始化
  CS_Config();
	
	USART1_Init(115200);
	
  Flash_Init();
	
	for(int gyro;gyro<GYRO_NUMBER;gyro++)
	{
		/*ICM20608G模块初始化*/
		if(!allPara.resetFlag)
			MEMS_Configure(gyro);
		/*初始时不加热*/
		ICM_HeatingPower(gyro,0);
	}
	
  TIM_Init(TIM2,999,83,1,0);					//主周期定时5ms
	
	SetCommand(HEATING);
	SetCommand(ACCUMULATE);
  driftCoffecientInit();
	
	IWDG_Init(1,5); // 1.5ms
	
	while(!getTempInitSuces())
	{
		//如果是重启的，直接跳过
		if(allPara.resetFlag)
			SetTempInitSuces();
	}
}


int main(void)
{
  init();
//	char readOrderLast=(char)-1;
  while(1)
  {
    while(getTimeFlag())
    {
			//重启时，可能因为这个多耽误5ms
//			while(readOrderLast==getReadOrder()){;}
//				readOrderLast=getReadOrder();

//      				uint8_t test[3];
//      				test[0]=SPI_Read(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_WHO_AM_I); //测试ICM20608G，正确值为0XAF
			//使数据能够同步，但是不同步情况很少
			//AT指令处理
			if(isnan(allPara.Result_Angle[2])||isnan(allPara.GYRO_Real[2]))
				;
			AT_CMD_Handle();
      if(!(GetCommand()&CORRECT)){
				if(GetCommand()&HEATING)
				{
					for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
						temp_pid_ctr(gyro,allPara.GYRO_TemperatureAim[gyro]);
				}
				else
				{
					for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
						temp_pid_ctr(gyro,allPara.GYRO_TemperatureAim[gyro]-0.5f);
				}
				/*判断是否静止*/
				JudgeStatic();
        //计算角度 
        if(RoughHandle())
				{
          updateAngle();
          calculatePos();
					#ifndef TEST_SUMMER
					//串口被中断打断依然能正常发送（试验了几分钟）
					DataSend();
					#endif
				}
			}
      else{
        UpdateVDoffTable();
			}
			allPara.resetFlag=0;
      //真实的占空比会等于cpuUsage或大于其一个单位
      allPara.cpuUsage--;
    }
  }
}

