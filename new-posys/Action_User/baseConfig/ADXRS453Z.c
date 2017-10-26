#include "config.h"
#include "ADXRS453Z.h"
#include "timer.h"
#include "spi.h"

extern uint8_t   *scaleMode;
void ADXRS453Z_init(void)
{
  uint8_t order=0;
  uint8_t registers[REGISTERS]={
    ICM20608G_PWR_MGMT_1,0,/*10011 Wake up chip from sleep mode,enable temperature sensor,select pll	*/
    ICM20608G_PWR_MGMT_2,0,/*disable FIFO,enable gyr and accel*/
    ICM20608G_GYRO_CONFIG,8,/* gyro range:±500dps, Used to bypass DLPF				*/
    ICM20608G_CONFIG,0,/*  DLPF低通滤波器的设置	低通滤波器截止频率为176Hz 根据↓*/
    ICM20608G_SMPLRT_DIV,7,/* 设置采样速率为1kHz		*/
    ICM20608G_ACCEL_CONFIG,0,/* accel:2g																	*/
    ICM20608G_ACCEL_CONFIG2,6,/*000110 DLPF:5.1	低通滤波器的设置	 不能设置低功耗模式的均值滤波，否则数字不对	*/
    ICM20608G_SIGNAL_PATH_RESET,0,/* Use SIG_COND_RST to clear sensor registers.*/
    ICM20608G_USER_CTRL,16,/*disable iic and make spi only  里面有一个设置可以清空数据通道（看是否会出现第一个值极大的情况）*/
    ICM20608G_LP_MODE_CFG,0,//不进入低功耗模式
    ICM20608G_FIFO_EN,0,//不使能FIFO
    ICM20608G_ACCEL_WOM_THR,0,//不使用中断
    ICM20608G_INT_PIN_CFG,0,//不使用中断
    ICM20608G_INT_ENABLE,0,//不使用中断
    ICM20608G_ACCEL_INTEL_CTRL,0//不使能Wake-on-Motion detection logic
  };	
  	switch(*scaleMode)
	{
		/*250dps*/
		case 0:
			registers[5]=0;
			break;
		/*500dps*/
		case 1:
			registers[5]=8;
			break;
  }
  Delay_ms(100);																					//Start-up time from power-up for register read/write  max 100
  SPI_Write(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_PWR_MGMT_1,0x80);
  //SPI_Write(SPI2,GPIOB,GPIO_Pin_10,ICM20608G_PWR_MGMT_1,0x80);
  Delay_ms(1);
  Delay_ms(5);																				  	//Start-up time from sleep for register read/write  max 5
  
  for(order=0;order<REGISTERS/2;order++){
    uint8_t i=0;
    uint8_t data[2]={0xFF,0XFF};
    do{
      i++;
      SPI_Write(SPI1,GPIOA,GPIO_Pin_4,registers[order*2],registers[order*2+1]);
      //SPI_Write(SPI2,GPIOB,GPIO_Pin_10,registers[order*2],registers[order*2+1]);
      Delay_ms(i);
      if(i>5)
      {
        i=1;
        //USART_OUT(USART1,"init error");
        //break;
      }
      data[0]=SPI_Read(SPI1,GPIOA,GPIO_Pin_4,registers[order*2]);
    //  data[1]=SPI_Read(SPI2,GPIOB,GPIO_Pin_10,registers[order*2]);
    }while(data[0]!=registers[order*2+1]);//||data[1]!=registers[order*2+1]);
  }
  
}

