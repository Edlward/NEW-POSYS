#include "icm_20608_g.h"
#include "timer.h"
#include "spi.h"
#include "arm_math.h"
#include "usart.h"
#include "figureAngle.h"

void ICM20608G_init(void)
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



static gyro_t gyro;
void icm_update_gyro_rate(void)
{
  short data1[3] = {0,0,0};
  //	short data2[3] = {0,0,0};
  unsigned char raw[6];
  /*
  raw从低地址到高地址依次是
  X高八位,X第八位,Y高八位,Y第八位,Z高八位,Z第八位
  */
  SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_GYRO_XOUT_H,raw,6);
  /*X的原始角速度值*/
  data1[0] = (raw[0]<<8) | raw[1];
  /*Y的原始角速度值*/
  data1[1] = (raw[2]<<8) | raw[3];
  /*Y的原始角速度值*/
  data1[2] = (raw[4]<<8) | raw[5];
  
  gyro.No1.x = -data1[1]/65.5f;
  gyro.No1.y = -data1[0]/65.5f;
  gyro.No1.z = -data1[2]/65.5f;
  
  //	SPI_MultiRead(SPI2,GPIOB,GPIO_Pin_10,ICM20608G_GYRO_XOUT_H,raw,6);
  //	/*X的原始角速度值*/
  //	data2[0] = (raw[0]<<8) | raw[1];
  //	/*Y的原始角速度值*/
  //	data2[1] = (raw[2]<<8) | raw[3];
  //	/*Y的原始角速度值*/
  //	data2[2] = (raw[4]<<8) | raw[5];
  
  //	gyro.No2.x = -data2[1]/131.f;
  //	gyro.No2.y = data2[0]/131.f;
  //	gyro.No2.z = data2[2]/131.f;
}
void icm_read_gyro_rate(gyro_t *data)
{
  (*data).No1.x=gyro.No1.x;
  (*data).No1.y=gyro.No1.y;
  (*data).No1.z=gyro.No1.z;
  
  
  (*data).No2.x=gyro.No2.x;
  (*data).No2.y=gyro.No2.y;
  (*data).No2.z=gyro.No2.z;
}

static gyro_t acc;
void icm_update_acc(void)
{
  short data1[3] = {0,0,0};
  //	short data2[3] = {0,0,0};
  unsigned char raw[6];
  /*
  raw从低地址到高地址依次是
  X高八位,X第八位,Y高八位,Y第八位,Z高八位,Z第八位
  */
  SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_ACCEL_XOUT_H,raw,6);
  /*X的原始角速度值*/
  data1[0] = (raw[0]<<8) | raw[1];
  /*Y的原始角速度值*/
  data1[1] = (raw[2]<<8) | raw[3];
  /*Y的原始角速度值*/
  data1[2] = (raw[4]<<8) | raw[5];
  /*
  16384 LSB/g
  */
  acc.No1.x = data1[1]/16384.0;
  acc.No1.y = data1[0]/16384.0;
  acc.No1.z = data1[2]/16384.0;
  
  //	SPI_MultiRead(SPI2,GPIOB,GPIO_Pin_10,ICM20608G_ACCEL_XOUT_H,raw,6);
  //	/*X的原始角速度值*/
  //	data2[0] = (raw[0]<<8) | raw[1];
  //	/*Y的原始角速度值*/
  //	data2[1] = (raw[2]<<8) | raw[3];
  //	/*Y的原始角速度值*/
  //	data2[2] = (raw[4]<<8) | raw[5];
  //	
  //	acc.No2.x = -data2[0]/16384.0;
  //	acc.No2.y = data2[1]/16384.0;
  //	acc.No2.z = -data2[2]/16384.0;
  
}
void icm_read_accel_acc(gyro_t *data)
{
  (*data).No1.x=acc.No1.x;
  (*data).No1.y=acc.No1.y;
  (*data).No1.z=acc.No1.z;
  
  //	(*data).No2.x=acc.No2.x;
  //	(*data).No2.y=acc.No2.y;
  //	(*data).No2.z=acc.No2.z;
}

static float temp;
void icm_update_temp(void)
{
  short raw;
  uint8_t data[2];
  
  SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_4,ICM20608G_TEMP_OUT_H,data,2);
  
  raw = (data[0] << 8) | data[1];
  /*
  326.8f LSB/℃
  TEMP_degC = ((TEMP_OUT –
  RoomTemp_Offset)/Temp_Sensitivity) + 25degC
  */
  temp = 25.f + (float)raw / 326.8f;
}
void icm_read_temp(float *data)
{
  *data=temp;
}

void icm_update_AccRad(double accInit[3],three_axis *rad)
{
  double sum=sqrt(accInit[0]*accInit[0]+accInit[1]*accInit[1]+accInit[2]*accInit[2]);
  float X_G,Y_G,Z_G;
  
  X_G=accInit[0]/sum;
  Y_G=accInit[1]/sum;
  Z_G=accInit[2]/sum;
  /*初始坐标为0,0,g,然后可以通过坐标变换公式轻易推导*/
	(*rad).y= safe_atan2( X_G , -Z_G);
  (*rad).x=-safe_atan2( Y_G , X_G/sin((*rad).y));
}
