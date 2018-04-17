#include "icm_20608_g.h"
#include "config.h"

extern AllPara_t allPara;

//gyro startup time from sleep mode is 35ms.
//accelerometer startup time from sleep mode is 20ms
//and 30ms for cold startup.
//we set 40ms for safety

#define MAX_STARTUP_FROM_SLEEP_MAX_TIME (40)
#define MAX_POWER_RAMP_TIME (100)
#define MAX_REGISTER_STARTUP_TIME (100)
void MEMS_Configure(int gyroNum)
{
  uint8_t order=0;
  uint8_t registers[REGISTERS]={
    ICM20608G_PWR_MGMT_1,0,/*10011 Wake up chip from sleep mode,enable temperature sensor,select pll	*/
    ICM20608G_GYRO_CONFIG,0,/* gyro range:±500dps, Used to bypass DLPF				*/
		#ifdef	AUTOCAR
    ICM20608G_CONFIG,0,/*  DLPF低通滤波器的设置	低通滤波器截止频率为176Hz 根据↓*/
    ICM20608G_SMPLRT_DIV,7,/* 设置采样速率为1kHz		*/
		#else
    ICM20608G_CONFIG,0,/*  DLPF低通滤波器的设置	低通滤波器截止频率为176Hz 根据↓*/
    ICM20608G_SMPLRT_DIV,7,/* 设置采样速率为1kHz		*/
		#endif
    ICM20608G_ACCEL_CONFIG,0,/* accel:2g																	*/
    ICM20608G_ACCEL_CONFIG2,2,/*000110 DLPF:5.1	低通滤波器的设置	 不能设置低功耗模式的均值滤波，否则数字不对	*/
    ICM20608G_SIGNAL_PATH_RESET,0,/* Use SIG_COND_RST to clear sensor registers.*/
    ICM20608G_USER_CTRL,0x10,/*disable iic and make spi only  里面有一个设置可以清空数据通道（看是否会出现第一个值极大的情况）*/
    ICM20608G_LP_MODE_CFG,0,//不进入低功耗模式
    ICM20608G_FIFO_EN,0,//不使能FIFO
    ICM20608G_ACCEL_WOM_THR,0,//不使用中断
    ICM20608G_INT_PIN_CFG,0,//不使用中断
    ICM20608G_INT_ENABLE,0,//不使用中断
    ICM20608G_ACCEL_INTEL_CTRL,0,//不使能Wake-on-Motion detection logic
    ICM20608G_PWR_MGMT_2,0/*disable FIFO,enable gyr and accel*/
  };	

  Delay_ms(MAX_POWER_RAMP_TIME + MAX_REGISTER_STARTUP_TIME);							//in consideration of worse case, we need wait this much time
	
	switch(gyroNum)
	{
		case 0:
		SPI_Write(SPI1,GPIO_GYRO,GPIO_PIN,ICM20608G_PWR_MGMT_1,0x80);
		break;
		case 1:
		SPI_Write(SPI1,GPIOA,GPIO_Pin_2,ICM20608G_PWR_MGMT_1,0x80);
		break;
		case 2:
		SPI_Write(SPI1,GPIOC,GPIO_Pin_6,ICM20608G_PWR_MGMT_1,0x80);
		break;
	}
	
  Delay_ms(MAX_POWER_RAMP_TIME + MAX_REGISTER_STARTUP_TIME);							//in consideration of worse case, we need wait this much time
  
  for(order=0;order<REGISTERS/2;order++){
    uint8_t i=0;
    uint8_t data=0xFF;
    do{
      i++;
			switch(gyroNum)
			{
				case 0:
					SPI_Write(SPI1,GPIO_GYRO,GPIO_PIN,registers[order*2],registers[order*2+1]);
					Delay_ms(i);
					data=SPI_Read(SPI1,GPIO_GYRO,GPIO_PIN,registers[order*2]);
					break;
				case 1:
					SPI_Write(SPI1,GPIOA,GPIO_Pin_2,registers[order*2],registers[order*2+1]);
					Delay_ms(i);
					data=SPI_Read(SPI1,GPIOA,GPIO_Pin_2,registers[order*2]);
					break;
				case 2:
					SPI_Write(SPI1,GPIOC,GPIO_Pin_6,registers[order*2],registers[order*2+1]);
					Delay_ms(i);
					data=SPI_Read(SPI1,GPIOC,GPIO_Pin_6,registers[order*2]);
					break;
			}
      if(i>5)
        break;

    }while(data!=registers[order*2+1]);//||data[1]!=registers[order*2+1]);
  }
	
	Delay_ms(MAX_STARTUP_FROM_SLEEP_MAX_TIME);
}



static double gyro[3];
void icm_update_gyro_rate(int gyroNum)
{
  short data1[3] = {0,0,0};
  unsigned char raw[6];

	switch(gyroNum)
	{
		case 0:
			SPI_MultiRead(SPI1,GPIO_GYRO,GPIO_PIN,ICM20608G_GYRO_XOUT_H,raw,6);
		break;
		case 1:
			SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_2,ICM20608G_GYRO_XOUT_H,raw,6);
		break;
		case 2:
			SPI_MultiRead(SPI1,GPIOC,GPIO_Pin_6,ICM20608G_GYRO_XOUT_H,raw,6);
		break;
	}
  data1[0] = (raw[0]<<8) | raw[1];  
	data1[1] = (raw[2]<<8) | raw[3];
  data1[2] = (raw[4]<<8) | raw[5];
  
	//auto old board 131.140172004891
	#ifdef NEW_BOARD
	gyro[0] = -data1[1]/131.140172004891;
	gyro[1] = data1[0]/131.140172004891;
	gyro[2] = data1[2]/131.140172004891;
	#else
	gyro[0] = -data1[1]/131.140172004891;
	gyro[1] = -data1[0]/131.140172004891;
	gyro[2] = -data1[2]/131.140172004891;
	#endif

	float middlePerson = 0.f;
	switch(gyroNum)
	{
		case 0:
			middlePerson = gyro[0];
			gyro[0] = gyro[1];
			gyro[1] = -middlePerson;
			break;
		case 2:
			middlePerson = gyro[0];
			gyro[0] = gyro[1];
			gyro[1] = -middlePerson;
			break;
	}
}
void icm_read_gyro_rate(double data[GYRO_NUMBER])
{
  (data)[0]=gyro[0];
  (data)[1]=gyro[1];
  (data)[2]=gyro[2];
}

static float acc[3];
void icm_update_acc(int gyroNum)
{
  short data1[3] = {0,0,0};
  //	short data2[3] = {0,0,0};
  unsigned char raw[6];

	switch(gyroNum)
	{
		case 0:
			SPI_MultiRead(SPI1,GPIO_GYRO,GPIO_PIN,ICM20608G_ACCEL_XOUT_H,raw,6);
		break;
		case 1:
			SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_2,ICM20608G_ACCEL_XOUT_H,raw,6);
		break;
		case 2:
			SPI_MultiRead(SPI1,GPIOC,GPIO_Pin_6,ICM20608G_ACCEL_XOUT_H,raw,6);
		break;
	}
 
  data1[0] = (raw[0]<<8) | raw[1];

  data1[1] = (raw[2]<<8) | raw[3];

  data1[2] = (raw[4]<<8) | raw[5];
  /*
  16384 LSB/g
  */
  acc[0] = data1[1]/16384.0;
  acc[1] = data1[0]/16384.0;
  acc[2] = data1[2]/16384.0;
  
	float middlePerson = 0.f;
	switch(gyroNum)
	{
		case 0:
			middlePerson = acc[0];
			acc[0] = acc[1];
			acc[1] = -middlePerson;
			break;
		case 2:
			middlePerson = acc[0];
			acc[0] = acc[1];
			acc[1] = -middlePerson;
			break;
	}
	
}
void icm_read_accel_acc(double data[GYRO_NUMBER])
{
  (data)[0]=acc[0];
  (data)[1]=acc[1];
  (data)[2]=acc[2];
}

static float temp;
void icm_update_temp(int gyroNum)
{
  short data;
  uint8_t byte[2];
  
	switch(gyroNum)
	{
		case 0:
			SPI_MultiRead(SPI1,GPIO_GYRO,GPIO_PIN,ICM20608G_TEMP_OUT_H,byte,2);
		break;
		case 1:
			SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_2,ICM20608G_TEMP_OUT_H,byte,2);
		break;
		case 2:
			SPI_MultiRead(SPI1,GPIOC,GPIO_Pin_6,ICM20608G_TEMP_OUT_H,byte,2);
		break;
	}
  
  data = (byte[0] << 8) | byte[1];
  /*
  326.8f LSB/℃
  TEMP_degC = ((TEMP_OUT –
  RoomTemp_Offset)/Temp_Sensitivity) + 25degC
  */
  temp = 25.f + (float)data / 326.8f;
}
void icm_read_temp(float *data)
{
  *data=temp;
}

void icm_update_AccRad(float ACC_Init[GYRO_NUMBER][AXIS_NUMBER])
{
	float sum[GYRO_NUMBER]={0.f};
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
	{
		float X_G,Y_G,Z_G;
		sum[gyro]=sqrt(ACC_Init[gyro][0]*ACC_Init[gyro][0]+ACC_Init[gyro][1]*ACC_Init[gyro][1]+ACC_Init[gyro][2]*ACC_Init[gyro][2]);
		
		X_G=ACC_Init[gyro][0]/sum[gyro];
		Y_G=ACC_Init[gyro][1]/sum[gyro];
		Z_G=ACC_Init[gyro][2]/sum[gyro];
		/*初始坐标为0,0,g,然后可以通过坐标变换公式轻易推导*/
		allPara.ACC_Angle[gyro][1]= safe_atan2( X_G , -Z_G);
		allPara.ACC_Angle[gyro][0]=-safe_atan2( Y_G , X_G/sin(allPara.ACC_Angle[gyro][1]));
	}
	allPara.ACC_RealAngle[0]=(allPara.ACC_Angle[0][0]+allPara.ACC_Angle[1][0]+allPara.ACC_Angle[2][0])/3.f;
	allPara.ACC_RealAngle[1]=(allPara.ACC_Angle[0][1]+allPara.ACC_Angle[1][1]+allPara.ACC_Angle[2][1])/3.f;
}

int CheckNan(void)
{
	static int count=0;
	if(isnan(allPara.sDta.codeData[0])||isnan(allPara.sDta.Result_Angle[2])||isnan(allPara.sDta.posx)||isnan(allPara.sDta.posy)||isnan(allPara.sDta.flag)||isnan(allPara.sDta.vellx)\
		||isnan(allPara.sDta.velly)||isnan(allPara.sDta.isReset))
	{
		USART_OUT(SEND_USART,"HN");
		USART_OUT(SEND_USART,"HN");
		return 1;
	}
	
	for(int i=0;i<GYRO_NUMBER;i++)
	{
		if(allPara.sDta.Result_Angle[i]>200.0||allPara.sDta.Result_Angle[i]<-200.0)
		{
			return 1;
		}
		if(allPara.sDta.GYRO_TemperatureAim[i]>200.0f||allPara.sDta.GYRO_TemperatureAim[i]<-200.0f)
		{
			return 1;
		}
	}
	
	for(int i=0;i<GYRO_NUMBER;i++)
		for(int j=0;j<3;j++)
		{
			if(allPara.GYROWithoutRemoveDrift[i][j]==0.0)
				count++;
			else
				count=0;
		}
	if(count>100)
	{
		USART_OUT(SEND_USART,"HZ");
		USART_OUT(SEND_USART,"HZ");
		return 1;
	}
	return 0;
}

void AllParaInit(void)
{
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			allPara.GYROWithoutRemoveDrift[i][j]=0.0;
			allPara.GYRORemoveDrift[i][j]=0.0;
			allPara.ACC_Raw[i][j]=0.0f;
			allPara.ACC_Aver[i][j]=0.0f;
			allPara.driftCoffecient[i][j]=0.0f;
		}
	for(int i=0;i<3;i++)
		for(int j=0;j<2;j++)
			allPara.ACC_Angle[i][j]=0.0f;
	
	for(int j=0;j<2;j++)
		allPara.ACC_RealAngle[j]=0.0f;
		
	for(int j=0;j<3;j++)
	{
		allPara.GYRO_Real[j]=0.0f;
		allPara.GYRO_Temperature[j]=0.0f;
		allPara.GYRO_TemperatureDif[j]=0.0f;
	}
	
	allPara.ACC_InitSum=0.f;
	
	allPara.cpuUsage=0;
	
	allPara.resetTime=0;
	
	allPara.kalmanZ=0.0;
	
	allPara.resetFlag=0;
	
	allPara.sDta.isReset=0;
  
	allPara.sDta.posx=0.0;
	
	allPara.sDta.posy=0.0;

	allPara.sDta.vellx=0.f;
	
	allPara.sDta.velly=0.f;
  
	for(int j=0;j<2;j++)
	{
		allPara.sDta.codeData[j]=0;
		allPara.sDta.data_last[j]=0;
		allPara.sDta.vell[j]=0;
	}
	
	allPara.sDta.flag=0;
	
	allPara.sDta.time=0;
	
	for(int i=0;i<3;i++)
	{
		allPara.sDta.GYRO_TemperatureAim[i]=0.f;
		allPara.sDta.GYRO_Aver[i]=0.f;
		allPara.sDta.GYRO_Bais[i]=0.f;
		allPara.sDta.Result_Angle[i]=0.f;
	}
  
	for(int i=0;i<4;i++)
		allPara.sDta.quarternion[i]=0.0;
}



