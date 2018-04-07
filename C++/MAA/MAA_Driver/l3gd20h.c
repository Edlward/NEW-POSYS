#include "l3gd20h.h"
#include "spi.h"
#include "fifo.h"

static float L3GD20H_wx;
static float L3GD20H_wy;
static float L3GD20H_wz;

static int8_t L3GD20H_Temp;

void L3GD20H_Init()
{
  L3GD20H_Write(L3GD20H_CTRL_REG1,0xcf);
	L3GD20H_Write(L3GD20H_CTRL_REG2,0x25);
	L3GD20H_Write(L3GD20H_CTRL_REG3,0x00);
	L3GD20H_Write(L3GD20H_CTRL_REG4,0x00);
	L3GD20H_Write(L3GD20H_CTRL_REG5,0x63);
	
	L3GD20H_Write(L3GD20H_FIFO_CTRL_REG,0x5f);
	
	L3GD20H_Write(L3GD20H_LOW_ODR,  0x00);
}


void  L3GD_read_gyr(three_axis *data)
{
	   (*data).x=L3GD20H_wx;
	   (*data).y=L3GD20H_wy;
	   (*data).z=L3GD20H_wz;
}


void L3GD_updateW(void)
{
	
	uint8_t buf[2]={0,0};
	uint16_t buf16;
	
	buf[0]=L3GD20H_Read(L3GD20H_OUT_X_L);
	buf[1]=L3GD20H_Read(L3GD20H_OUT_X_H);	
	buf16=(buf[0])|((buf[1])<<8);
	L3GD20H_wy= ((short)buf16)*0.00875-0.2;
	
	buf[0]=L3GD20H_Read(L3GD20H_OUT_Y_L);
	buf[1]=L3GD20H_Read(L3GD20H_OUT_Y_H);
	buf16=(buf[0])|((buf[1])<<8);
	L3GD20H_wx=((short)buf16)*0.00875-0.5;
	
	buf[0]=L3GD20H_Read(L3GD20H_OUT_Z_L);
	buf[1]=L3GD20H_Read(L3GD20H_OUT_Z_H);	
	buf16=(buf[0])|((buf[1])<<8);
	L3GD20H_wz=((short)buf16)*0.00875+1.5;
	
}
void L3GD_readTemp(float *val)
{
  (*val)=L3GD20H_Temp;	
}
void L3GD_updateTemp(void)
{
	L3GD20H_Temp=(int8_t)L3GD20H_Read(L3GD20H_OUT_TEMP);
}

void L3GD20H_FIFO_Read(L3GD20H_FIFO_Data *data)
{	
	uint8_t SRC;
	
	uint8_t i;
	uint8_t buf[2]={0,0};
	uint16_t buf16;
	
	SRC=L3GD20H_Read(L3GD20H_FIFO_SRC_REG);
	(*data).FIFO_FTH=SRC>>7;
	(*data).FIFO_Full=(SRC&0X7F)>>6;
	(*data).FIFO_Empty=(SRC&0X3F)>>5;		
	(*data).FIFO_len=(SRC&0X1F);
	for(i=0;i<(*data).FIFO_len;i++)
	{
		buf[0]=L3GD20H_Read(L3GD20H_OUT_X_L);
		buf[1]=L3GD20H_Read(L3GD20H_OUT_X_H);	
		buf16=(buf[0])|((buf[1])<<8);
		(*data).x[i]=((short)buf16)*0.00875;
		
		buf[0]=L3GD20H_Read(L3GD20H_OUT_Y_L);
		buf[1]=L3GD20H_Read(L3GD20H_OUT_Y_H);
		buf16=(buf[0])|((buf[1])<<8);
		(*data).y[i]=((short)buf16)*0.00875;
		
		buf[0]=L3GD20H_Read(L3GD20H_OUT_Z_L);
		buf[1]=L3GD20H_Read(L3GD20H_OUT_Z_H);	
		buf16=(buf[0])|((buf[1])<<8);
		(*data).z[i]=((short)buf16)*0.00875;
	}
}
