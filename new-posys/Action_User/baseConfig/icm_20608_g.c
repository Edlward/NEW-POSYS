#include "icm_20608_g.h"
#include "timer.h"
#include "spi.h"
#include "math.h"
#include "usart.h"
#include "figureAngle.h"

void ICM20608G_init(void)
{
	uint8_t order=0;
	uint8_t registers[REGISTERS]={
	ICM20608G_PWR_MGMT_1,0,/*10011 Wake up chip from sleep mode,enable temperature sensor,select pll	*/
	ICM20608G_PWR_MGMT_2,0,/*disable FIFO,enable gyr and accel*/
	ICM20608G_GYRO_CONFIG,0,/* gyro range:±250dps, Used to bypass DLPF				*/
	ICM20608G_CONFIG,0,/*  DLPF低通滤波器的设置	低通滤波器截止频率为176Hz 根据↓*/
	ICM20608G_SMPLRT_DIV,7,/* 设置采样速率为333Hz			能不失真地对最大为	166Hz的波	*/
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
	ICM_WriteByte(ICM20608G_PWR_MGMT_1,0x80);												/*复位内部寄存器和恢复默认设置。 	初始值为0，成功写入0x80时会自动变成0x40,进入睡眠模式*/	
	Delay_ms(1);
	Delay_ms(5);																				  	//Start-up time from sleep for register read/write  max 5
	
	for(order=0;order<REGISTERS/2;order++){
		uint8_t i=0;
		uint8_t data=0xFF;
		do{
//			i++;
			ICM_WriteByte(registers[order*2],registers[order*2+1]);
			Delay_ms(1);
//			Delay_ms(i);
//			if(i>5)
//			{
//				USART_OUT(USART1,(uint8_t*)"init error");
//				break;
//			}
			data=ICM_ReadByte(registers[order*2]);
		}while(data!=registers[order*2+1]);
	}
	
}

void icm_get_gyro_data(short *data)
{
	unsigned char raw[6];
	/*
	raw从低地址到高地址依次是
	X高八位,X第八位,Y高八位,Y第八位,Z高八位,Z第八位
	*/
	mRead(raw, ICM20608G_GYRO_XOUT_H, 6);
	/*X的原始角速度值*/
	data[0] = (raw[0]<<8) | raw[1];
	/*Y的原始角速度值*/
	data[1] = (raw[2]<<8) | raw[3];
	/*Y的原始角速度值*/
	data[2] = (raw[4]<<8) | raw[5];
}


void icm_get_accel_data(short *data)
{
	uint8_t raw[6];
	mRead(raw, ICM20608G_ACCEL_XOUT_H, 6);
	data[0] = (raw[0]<<8) | raw[1];
	data[1] = (raw[2]<<8) | raw[3];
	data[2] = (raw[4]<<8) | raw[5];
	
}

void icm_get_temp_data(long *data)
{
	short raw;
	uint8_t temp[2];
	
	mRead(temp,ICM20608G_TEMP_OUT_H,2);
	raw = (temp[0] << 8) | temp[1];
	/*
	326.8f LSB/℃
	TEMP_degC = ((TEMP_OUT –
	RoomTemp_Offset)/Temp_Sensitivity) + 25degC
	*/
	data[0] = (long)((25 + ((raw - (float)0) / 326.8f)) * 65536L);//??65536L什么意思
}

BOOL icm_check_whoami(void)
{   
    if(ICM_ReadByte(ICM20608G_WHO_AM_I) == 0xAF) {
      return TRUE;
    } 
    
    return FALSE;
}


/**
 *  @brief      Push biases to the gyro bias 20608 registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. Bias inputs are LSB
 *  in +-1000dps format.
 *  @param[in]  gyro_bias  New biases.
 *  @return     0 if successful.
 */
void icm_set_gyro_bias(long *gyro_bias)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
		
    data[0] = (gyro_bias[0] >> 8) & 0xff;
    data[1] = (gyro_bias[0]) & 0xff;
    data[2] = (gyro_bias[1] >> 8) & 0xff;
    data[3] = (gyro_bias[1]) & 0xff;
    data[4] = (gyro_bias[2] >> 8) & 0xff;
    data[5] = (gyro_bias[2]) & 0xff;
		
		ICM_WriteByte(ICM20608G_XG_OFFS_USRH,data[0]);
		ICM_WriteByte(ICM20608G_XG_OFFS_USRL,data[1]);
		ICM_WriteByte(ICM20608G_YG_OFFS_USRH,data[2]);
		ICM_WriteByte(ICM20608G_YG_OFFS_USRL,data[3]);
		ICM_WriteByte(ICM20608G_ZG_OFFS_USRH,data[4]);
		ICM_WriteByte(ICM20608G_ZG_OFFS_USRL,data[5]);
}

/**
 *  @brief      Push biases to the accel bias 20608 registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. Bias inputs are LSB
 *  in +-16G format.
 *  @param[in]  accel_bias  New biases.
 *  @return     0 if successful.
 */
void icm_set_accel_bias(const long *accel_bias)
{
    unsigned char data[6] = {0, 0, 0, 0, 0, 0};
    long accel_reg_bias[3] = {0, 0, 0};

    // Preserve bit 0 of factory value (for temperature compensation)
    accel_reg_bias[0] -= (accel_bias[0] & ~1);
    accel_reg_bias[1] -= (accel_bias[1] & ~1);
    accel_reg_bias[2] -= (accel_bias[2] & ~1);

    data[0] = (accel_reg_bias[0] >> 8) & 0xff;
    data[1] = (accel_reg_bias[0]) & 0xff;
    data[2] = (accel_reg_bias[1] >> 8) & 0xff;
    data[3] = (accel_reg_bias[1]) & 0xff;
    data[4] = (accel_reg_bias[2] >> 8) & 0xff;
    data[5] = (accel_reg_bias[2]) & 0xff;

		ICM_WriteByte(ICM20608G_XA_OFFSET_H,data[0]);
		ICM_WriteByte(ICM20608G_XA_OFFSET_L,data[1]);
		ICM_WriteByte(ICM20608G_YA_OFFSET_H,data[2]);
		ICM_WriteByte(ICM20608G_YA_OFFSET_L,data[3]);
		ICM_WriteByte(ICM20608G_ZA_OFFSET_H,data[4]);
		ICM_WriteByte(ICM20608G_ZA_OFFSET_L,data[5]);
}

void icm_read_accel_bias(long *accel_bias) 
{
	((uint8_t *)&(accel_bias[0]))[0] = ICM_ReadByte(ICM20608G_XA_OFFSET_L);
	((uint8_t *)&(accel_bias[0]))[1] = ICM_ReadByte(ICM20608G_XA_OFFSET_H);
	((uint8_t *)&(accel_bias[1]))[0] = ICM_ReadByte(ICM20608G_YA_OFFSET_L);
	((uint8_t *)&(accel_bias[1]))[1] = ICM_ReadByte(ICM20608G_YA_OFFSET_H);
	((uint8_t *)&(accel_bias[2]))[0] = ICM_ReadByte(ICM20608G_ZA_OFFSET_L);
	((uint8_t *)&(accel_bias[2]))[1] = ICM_ReadByte(ICM20608G_ZA_OFFSET_H);
}

void icm_read_gyro_bias(long *gyro_bias) 
{
	((uint8_t *)&(gyro_bias[0]))[0] = ICM_ReadByte(ICM20608G_XG_OFFS_USRL);
	((uint8_t *)&(gyro_bias[0]))[1] = ICM_ReadByte(ICM20608G_XG_OFFS_USRH);
	((uint8_t *)&(gyro_bias[1]))[0] = ICM_ReadByte(ICM20608G_YG_OFFS_USRL);
	((uint8_t *)&(gyro_bias[1]))[1] = ICM_ReadByte(ICM20608G_YG_OFFS_USRH);
	((uint8_t *)&(gyro_bias[2]))[0] = ICM_ReadByte(ICM20608G_ZG_OFFS_USRL);
	((uint8_t *)&(gyro_bias[2]))[1] = ICM_ReadByte(ICM20608G_ZG_OFFS_USRH);
}

void icm_read_fifo(short *gyro)
{
	unsigned short fifo_count;
	unsigned char data[6] = {0,0,0,0,0,0},packet_size = 0;
	
	packet_size = 6;			//  使能了陀螺仪的3轴，6个字节的数据
	
	mRead(data,ICM20608G_FIFO_COUNTH,2);
//	data[0] = ICM_ReadByte(ICM20608G_FIFO_COUNTH);
//	data[1] = ICM_ReadByte(ICM20608G_FIFO_COUNTL);
//	
	fifo_count = (data[0] << 8) | data[1];
	
	if(packet_size > fifo_count)
		return;
	
	mRead(data,ICM20608G_FIFO_R_W,6);
	
	gyro[0] = (data[0] << 8) | data[1];
	gyro[1] = (data[2] << 8) | data[3];
	gyro[2] = (data[4] << 8) | data[5];
}



void icm_fifo_enable(void)
{
	ICM_WriteByte(ICM20608G_FIFO_EN,0x00);
	ICM_WriteByte(ICM20608G_USER_CTRL,0x00);
	
	ICM_WriteByte(ICM20608G_USER_CTRL,0x04);
	Delay_ms(100);
	ICM_WriteByte(ICM20608G_USER_CTRL,0x50);
	Delay_ms(100);
	
	ICM_WriteByte(ICM20608G_FIFO_EN,0x70);
}
static three_axis gyro;
void icm_update_gyro_rate(void)
{
	short data[3] = {0,0,0};
	icm_get_gyro_data(data);
	
	/*
	转到东北天坐标系 全部满足右手定则 
	北偏东为正由四元数转换到欧垃角实现
	GYRO_XOUT = Gyro_Sensitivity * X_angular_rate
	Gyro_Sensitivity = 131 LSB/(o/s)
	*/
	gyro.x = -data[1]/128.270614f;
	gyro.y = data[0]/128.270614f;
	gyro.z = data[2]/128.270614f;
}
void icm_read_gyro_rate(three_axis *data)
{
	(*data).x=gyro.x;
	(*data).y=gyro.y;
	(*data).z=gyro.z;
}
static three_axis acc;
void icm_update_acc(void)
{
	short data[3] = {0,0,0};
	three_axis temp;
	
	icm_get_accel_data(data);
	
	/*
	16384 LSB/g
	*/
	temp.x = -data[0]/16384.0;
	temp.y = data[1]/16384.0;
	temp.z = -data[2]/16384.0;
	
	acc.x=temp.y;
	acc.y=temp.x;
	acc.z=temp.z;
	
}
void icm_read_accel_acc(three_axis *val)
{
  val->x=acc.x;
	val->y=acc.y;
	val->z=acc.z;
}

static float temp;
void icm_update_temp(void)
{
	long tem_data = 0;
	icm_get_temp_data(&tem_data);
	
	temp = tem_data/65536.0;
}
void icm_read_temp(float *data)
{
	*data=temp;
}

extern float K_acc;
void icm_update_AccRad(three_axis *rad)
{
	float sum=sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z);
	float X_G,Y_G,Z_G;
	static uint8_t fix_flag=0;
	static float fix_sum;
	
	/*先求静止状态下的十个数*/
	/*
	这个值并不是很小,实际上可以计算,
	即使车的的加速度是1m/s也是在我们的范围内,
	而1m/s已经很大了
	*/
	if(fix_flag<21)
	{
		if(fix_flag>0)
		fix_sum=fix_sum+sum/20;
		fix_flag++;
	}
	else if(fabs(sum-fix_sum)<0.001)
   K_acc =  0.98;
	else if(fabs(sum-fix_sum)<0.002)
   K_acc = 0.985;
	else if(fabs(sum-fix_sum)<0.003)
   K_acc = 0.987;
	else if(fabs(sum-fix_sum)<0.004)
   K_acc = 0.988;
	else if(fabs(sum-fix_sum)<0.01)
	 K_acc = 0.99;
	else 
	 K_acc=1;	
	
	X_G=(acc).x/sum;
	Y_G=(acc).y/sum;
	Z_G=(acc).z/sum;
	/*初始坐标为0,0,g,然后可以通过坐标变换公式轻易推导*/
	(*rad).y= safe_atan2( X_G , -Z_G);
  (*rad).x=-safe_atan2( Y_G , X_G/sin((*rad).y));
}
