#include "icm_20608_g.h"
#include "config.h"

extern flashData_t flashData;
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
    ICM20608G_GYRO_CONFIG,0,/* gyro range:��500dps, Used to bypass DLPF				*/
    ICM20608G_CONFIG,1,/*  DLPF��ͨ�˲���������	��ͨ�˲�����ֹƵ��Ϊ176Hz ���ݡ�*/
    ICM20608G_SMPLRT_DIV,0,/* ���ò�������Ϊ1kHz		*/
    ICM20608G_ACCEL_CONFIG,0,/* accel:2g																	*/
    ICM20608G_ACCEL_CONFIG2,2,/*000110 DLPF:5.1	��ͨ�˲���������	 �������õ͹���ģʽ�ľ�ֵ�˲����������ֲ���	*/
    ICM20608G_SIGNAL_PATH_RESET,0,/* Use SIG_COND_RST to clear sensor registers.*/
    ICM20608G_USER_CTRL,0x10,/*disable iic and make spi only  ������һ�����ÿ����������ͨ�������Ƿ����ֵ�һ��ֵ����������*/
    ICM20608G_LP_MODE_CFG,0,//������͹���ģʽ
    ICM20608G_FIFO_EN,0,//��ʹ��FIFO
    ICM20608G_ACCEL_WOM_THR,0,//��ʹ���ж�
    ICM20608G_INT_PIN_CFG,0,//��ʹ���ж�
    ICM20608G_INT_ENABLE,0,//��ʹ���ж�
    ICM20608G_ACCEL_INTEL_CTRL,0,//��ʹ��Wake-on-Motion detection logic
    ICM20608G_PWR_MGMT_2,0/*disable FIFO,enable gyr and accel*/
  };	

  Delay_ms(MAX_POWER_RAMP_TIME + MAX_REGISTER_STARTUP_TIME);							//in consideration of worse case, we need wait this much time
	
	switch(gyroNum)
	{
		case 0:
		SPI_Write(SPI1,GPIOA,GPIO_Pin_1,ICM20608G_PWR_MGMT_1,0x80);
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
			switch(*(flashData.scaleMode+gyroNum))
			{
				/*250dps*/
				case 0:
					registers[5]=0;
					break;
				/*500dps*/
				case 1:
					registers[5]=8;
					break;
				default:
					registers[5]=0;
					break;
			}
			switch(gyroNum)
			{
				case 0:
					SPI_Write(SPI1,GPIOA,GPIO_Pin_1,registers[order*2],registers[order*2+1]);
					Delay_ms(i);
					data=SPI_Read(SPI1,GPIOA,GPIO_Pin_1,registers[order*2]);
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


/**
�Զ���131.170291536093
�ֶ���131.524243090403
*/
static double gyro[3];
void icm_update_gyro_rate(int gyroNum)
{
  short data1[3] = {0,0,0};
  unsigned char raw[6];
  /*
  raw�ӵ͵�ַ���ߵ�ַ������
  X�߰�λ,X�ڰ�λ,Y�߰�λ,Y�ڰ�λ,Z�߰�λ,Z�ڰ�λ
  */
	switch(gyroNum)
	{
		case 0:
			SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_1,ICM20608G_GYRO_XOUT_H,raw,6);
		break;
		case 1:
			SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_2,ICM20608G_GYRO_XOUT_H,raw,6);
		break;
		case 2:
			SPI_MultiRead(SPI1,GPIOC,GPIO_Pin_6,ICM20608G_GYRO_XOUT_H,raw,6);
		break;
	}
  /*X��ԭʼ���ٶ�ֵ*/
  data1[0] = (raw[0]<<8) | raw[1];
  /*Y��ԭʼ���ٶ�ֵ*/
  data1[1] = (raw[2]<<8) | raw[3];
  /*Y��ԭʼ���ٶ�ֵ*/
  data1[2] = (raw[4]<<8) | raw[5];
  
	switch(*(flashData.scaleMode+gyroNum))
	{
		case 0:
			#ifdef TESTCAR
					gyro[0] = -data1[1]/131.0;
					gyro[1] = -data1[0]/131.0;
					gyro[2] = -data1[2]/131.0;
			#else
				#ifdef AUTOCAR
					gyro[0] = -data1[1]/130.901407488835;
					gyro[1] = -data1[0]/130.901407488835;
					gyro[2] = -data1[2]/130.901407488835;
				#else
					gyro[0] = -data1[1]/131.524243090403;
					gyro[1] = -data1[0]/131.524243090403;
					gyro[2] = -data1[2]/131.524243090403;
				#endif
			#endif
			break;
		case 1:
			gyro[0] = -data1[1]/65.5f;
			gyro[1] = -data1[0]/65.5f;
			gyro[2] = -data1[2]/65.5f;
			break;
		default:
			#ifdef TESTCAR
					gyro[0] = -data1[1]/131.0;
					gyro[1] = -data1[0]/131.0;
					gyro[2] = -data1[2]/131.0;
			#else
				#ifdef AUTOCAR
					gyro[0] = -data1[1]/131.170291536093;
					gyro[1] = -data1[0]/131.170291536093;
					gyro[2] = -data1[2]/131.170291536093;
				#else
					gyro[0] = -data1[1]/131.524243090403;
					gyro[1] = -data1[0]/131.524243090403;
					gyro[2] = -data1[2]/131.524243090403;
				#endif
			#endif

			break;
  }
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
  /*
  raw�ӵ͵�ַ���ߵ�ַ������
  X�߰�λ,X�ڰ�λ,Y�߰�λ,Y�ڰ�λ,Z�߰�λ,Z�ڰ�λ
  */	
	switch(gyroNum)
	{
		case 0:
			SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_1,ICM20608G_ACCEL_XOUT_H,raw,6);
		break;
		case 1:
			SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_2,ICM20608G_ACCEL_XOUT_H,raw,6);
		break;
		case 2:
			SPI_MultiRead(SPI1,GPIOC,GPIO_Pin_6,ICM20608G_ACCEL_XOUT_H,raw,6);
		break;
	}
  /*X��ԭʼ���ٶ�ֵ*/
  data1[0] = (raw[0]<<8) | raw[1];
  /*Y��ԭʼ���ٶ�ֵ*/
  data1[1] = (raw[2]<<8) | raw[3];
  /*Y��ԭʼ���ٶ�ֵ*/
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
			SPI_MultiRead(SPI1,GPIOA,GPIO_Pin_1,ICM20608G_TEMP_OUT_H,byte,2);
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
  326.8f LSB/��
  TEMP_degC = ((TEMP_OUT �C
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
		/*��ʼ����Ϊ0,0,g,Ȼ�����ͨ������任��ʽ�����Ƶ�*/
		allPara.ACC_Angle[gyro][1]= safe_atan2( X_G , -Z_G);
		allPara.ACC_Angle[gyro][0]=-safe_atan2( Y_G , X_G/sin(allPara.ACC_Angle[gyro][1]));
	}
	allPara.ACC_RealAngle[0]=(allPara.ACC_Angle[0][0]+allPara.ACC_Angle[1][0]+allPara.ACC_Angle[2][0])/3.f;
	allPara.ACC_RealAngle[1]=(allPara.ACC_Angle[0][1]+allPara.ACC_Angle[1][1]+allPara.ACC_Angle[2][1])/3.f;
}


