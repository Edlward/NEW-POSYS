#include "icm_20608_g.h"
#include "timer.h"
#include "spi.h"
#include "math.h"
#include "motion_attitude_algorithm.h"
void ICM20608G_init(void)
{
	/**********************************************************************************************
				ICM20608G_PWR_MGMT_1
	----------------------------------------------------------------------------------------------
			7			|			6			|			5			|			4			|			3			|			2			|			1			|			0		
	----------------------------------------------------------------------------------------------
		reset		|		sleep		|	acc_cycle	|	g_standby	| temp_dis	|						clksel
	----------------------------------------------------------------------------------------------
			0			|			1			|			-			|		-				|			-			|							-
	**********************************************************************************************/
	ICM_WriteByte(ICM20608G_PWR_MGMT_1,0x80);												/* Reset device. 															*/				
	TIM_Delayms(TIM2,100);
	ICM_WriteByte(ICM20608G_PWR_MGMT_1,0x00);												/* Wake up chip. enable the temp							*/
	
	
	/**********************************************************************************************
				ICM20608G_SMPLRT_DIV
	-----------------------------------------------------------------------------------------------
		SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)		Where INTERNAL_SAMPLE_RATE = 1kHz
	-----------------------------------------------------------------------------------------------
	[7:0]:0~255
	**********************************************************************************************/
	ICM_WriteByte(ICM20608G_SMPLRT_DIV,4);													/* 设置采样速率为200hz											*/
	
	
	ICM_WriteByte(ICM20608G_CONFIG,0x02);														/* 99Hz DLPF																*/
	
	/**********************************************************************************************
				ICM20608G_GYRO_CONFIG
	----------------------------------------------------------------------------------------------
			7			|			6			|			5			|			4:3			|			2			|			1:0		
	----------------------------------------------------------------------------------------------
		XG_ST		|		YG_ST		|		ZG_ST		|		FS_SEL		|			-			|			FCHOICE_B[1:0]
	**********************************************************************************************/
	ICM_WriteByte(ICM20608G_GYRO_CONFIG, 0x00);											/* gyro:250dps, Used to bypass DLPF				*/
	
	/**********************************************************************************************
				ICM20608G_ACCEL_CONFIG
	----------------------------------------------------------------------------------------------
			7			|			6			|			5			|			4:3			|			2:0		
	----------------------------------------------------------------------------------------------
		XA_ST		|		YA_ST		|		ZA_ST		|		FS_SEL		|			-			
	**********************************************************************************************/
	ICM_WriteByte(ICM20608G_ACCEL_CONFIG,0x00);										/* accel:2g															*/
	
	
	/**********************************************************************************************
				ICM20608G_ACCEL_CONFIG2
	----------------------------------------------------------------------------------------------
			7:6			|			5:4			|			3			|			2:0		
	----------------------------------------------------------------------------------------------
			-				|		DEC2_CFG	|	FCHOICE_B	|		DLPF_CFG			
	**********************************************************************************************/
	ICM_WriteByte(ICM20608G_ACCEL_CONFIG2,0x02);										/* lpf:99.0														*/
	
	/**********************************************************************************************
				ICM20608G_LP_MODE_CFG
	----------------------------------------------------------------------------------------------
				7				|				6:4			|				3:0
	----------------------------------------------------------------------------------------------
		GYRO_CYCLE	|		G_AVGCFG		|			LPOSC_CLKSEL
	**********************************************************************************************/
	ICM_WriteByte(ICM20608G_LP_MODE_CFG,0x00);
	ICM_WriteByte(ICM20608G_ACCEL_WOM_THR,0x00);										/* 运动后时的加速度阈值								*/	
	ICM_WriteByte(ICM20608G_FIFO_EN,0x00);												
	ICM_WriteByte(ICM20608G_INT_PIN_CFG,0x00);
	ICM_WriteByte(ICM20608G_INT_ENABLE,0x00);
	ICM_WriteByte(ICM20608G_SIGNAL_INTEL_CTRL,0x00);
	ICM_WriteByte(ICM20608G_ACCEL_INTEL_CTRL,0x00);
	ICM_WriteByte(ICM20608G_USER_CTRL,0x10);
	ICM_WriteByte(ICM20608G_PWR_MGMT_2,0x00);

//  TIM_Delayms(TIM2,100);
//	icm_fifo_enable();
}

void icm_get_gyro_data(short *data)
{
	unsigned char raw[6];
	
	mRead(raw, ICM20608G_GYRO_XOUT_H, 6);
	
	data[0] = (raw[0]<<8) | raw[1];
	data[1] = (raw[2]<<8) | raw[3];
	data[2] = (raw[4]<<8) | raw[5];
}


void icm_get_accel_data(short *data)
{
	((uint8_t *)&(data[0]))[1] = ICM_ReadByte(ICM20608G_ACCEL_XOUT_H);
	TIM_Delay100us(TIM2,1);
	((uint8_t *)&(data[0]))[0] = ICM_ReadByte(ICM20608G_ACCEL_XOUT_L);
	TIM_Delay100us(TIM2,1);
	
	((uint8_t *)&(data[1]))[1] = ICM_ReadByte(ICM20608G_ACCEL_YOUT_H);
	TIM_Delay100us(TIM2,1);
	((uint8_t *)&(data[1]))[0] = ICM_ReadByte(ICM20608G_ACCEL_YOUT_L);
	TIM_Delay100us(TIM2,1);
	
	((uint8_t *)&(data[2]))[1] = ICM_ReadByte(ICM20608G_ACCEL_ZOUT_H);
	TIM_Delay100us(TIM2,1);
	((uint8_t *)&(data[2]))[0] = ICM_ReadByte(ICM20608G_ACCEL_ZOUT_L);
	
//	uint8_t raw[6];
//	mRead(raw, ICM20608G_ACCEL_XOUT_H, 6);
//	data[0] = (raw[0]<<8) | raw[1];
//	data[1] = (raw[2]<<8) | raw[3];
//	data[2] = (raw[4]<<8) | raw[5];
	
}

void icm_get_temp_data(long *data)
{
	short raw;
	uint8_t temp[2];
	
	mRead(temp,ICM20608G_TEMP_OUT_H,2);
	raw = (temp[0] << 8) | temp[1];
	
	data[0] = (long)((25 + ((raw - (float)0) / 326.8f)) * 65536L);
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
	TIM_Delayms(TIM2,100);
	ICM_WriteByte(ICM20608G_USER_CTRL,0x50);
	TIM_Delayms(TIM2,100);
	
	ICM_WriteByte(ICM20608G_FIFO_EN,0x70);
}
static three_axis gyro;
void icm_update_gyro_rate(void)
{
	short data[3] = {0,0,0};
	icm_get_gyro_data(data);
	
	//转到东北天坐标系 全部满足右手定则 北偏东为正由四元数转换到欧垃角实现
	gyro.x = -data[1]/131.0;
	gyro.y = -data[0]/131.0;
	gyro.z = -data[2]/131.0;
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
	
	temp.x = data[0]/16384.0;
	temp.y = data[1]/16384.0;
	temp.z = data[2]/16384.0;
	
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
	
	
	if(fix_flag<10)
	{
		fix_sum=fix_sum+sum/10;
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
	
	
	(*rad).y= safe_atan2( X_G , -Z_G);
  (*rad).x=-safe_atan2( Y_G , X_G/sin((*rad).y));
}
