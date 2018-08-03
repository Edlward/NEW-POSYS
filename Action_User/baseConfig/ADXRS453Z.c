#include "config.h"

#ifdef ADXRS453Z

void ADXRS453Z_init(void)
{
	uint16_t data=0x00;
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPI1, 0X2000); 																		//通过外设SPI1发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	data=SPI_I2S_ReceiveData(SPI1);	
  
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPI1, 0x0003); 																	//通过外设SPI1发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	SPI_I2S_ReceiveData(SPI1);	
		
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
	delay_ms(50);//第二个命令
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPI1, 0X2000); 																		//通过外设SPI1发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
  data=SPI_I2S_ReceiveData(SPI1);       
  
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPI1, 0x0000); 																//通过外设SPI1发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	data=SPI_I2S_ReceiveData(SPI1);		
		
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
		
	delay_ms(50);//第三个命令
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPI1, 0X2000); 																		//通过外设SPI1发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
  SPI_I2S_ReceiveData(SPI1);	        
  
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPI1, 0x0000); 																	//通过外设SPI1发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
  
	data=SPI_I2S_ReceiveData(SPI1);
		
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
	delay_ms(50);//第四个命令
		
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPI1, 0X2000); 																		//通过外设SPI1发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	data=SPI_I2S_ReceiveData(SPI1);        
  
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
	
	SPI_I2S_SendData(SPI1, 0x0000); 																	//通过外设SPI1发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
 
	data=SPI_I2S_ReceiveData(SPI1);		
		
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
		
	data=data;
	//rawDataRead(0x00);	
}

void shiftRightData(float rateList[5],uint32_t len)
{
	for(uint32_t i=0;i<len-1;i++)
	{
		rateList[len-i-1]=rateList[len-i-1-1];
	}
}
uint16_t rawDataRead(uint8_t address)
{
	return SPI16_Read(SPI1,GPIOA,GPIO_Pin_4,address);
}
void rawDataWrite(uint8_t address,uint16_t value)
{
	SPI16_Write(SPI1,GPIOA,GPIO_Pin_4,address,value);
}

float	meanData(float *data,uint32_t len)
{
	float re;
	arm_mean_f32(data,len,&re);
	return re;
}

// Function for sending and receiving data through SPI
void SpiFunction(unsigned char OutputBuff[],unsigned char InputBuff[])
{	
	 int i;
	 unsigned short int temp;
   for(i=0;i<2;i++)
   {
		 temp=(OutputBuff[i*2]<<8)|OutputBuff[i*2+1];
		 while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
		 SPI_I2S_SendData(SPI1, OutputBuff[i]); 																	//通过外设SPIx发送一个byte  数据
		 while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
		 temp=SPI_I2S_ReceiveData(SPI1);		
		 InputBuff[i*2]=(unsigned char)(temp>>8);
		 InputBuff[i*2+1]=(unsigned char)temp;
   }
}

// Function for sending and receiving data through SPI
//void SpiFunction(unsigned char OutputBuff[],unsigned char InputBuff[])
//{	
//	 int i;
//	 unsigned short int temp;
//   for(i=0;i<2;i++)
//   {
//		 temp=(OutputBuff[i*2]<<8)|OutputBuff[i*2+1];
//		 while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		  //等待发送区空  
//		 SPI_I2S_SendData(SPI1, OutputBuff[i]); 																	//通过外设SPIx发送一个byte  数据
//   }
//   for(i=0;i<2;i++)
//   {
//		 while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} 	  //等待接收完一个byte  
//		 temp=SPI_I2S_ReceiveData(SPI1);	
//		 SPI_I2S_ClearFlag(SPI1,SPI_I2S_FLAG_RXNE);
//		 InputBuff[i*2]=(unsigned char)(temp>>8);
//		 InputBuff[i*2+1]=(unsigned char)temp;
//   }
//}

/*******************************************************************
  @brief void ADXRS453StartUp(void)
         recommended start-up sequence
  @param
         none
  @return   
         none
*******************************************************************/
void ADXRS453StartUp(void)
{
    unsigned char SendTemp[4];
    unsigned char ReceiveTemp[4];
    unsigned char i;
  
    SendTemp[0] = 0x20;
    SendTemp[1] = 0x00;  
    SendTemp[2] = 0x00;
    SendTemp[3] = 0x03;
    SpiFunction(SendTemp, ReceiveTemp);       //0x20000003
    Delay_ms(50);
  
    SendTemp[0] = 0x20;
    SendTemp[1] = 0x00;  
    SendTemp[2] = 0x00;
    SendTemp[3] = 0x00; 
    for (i=0; i<3; i++)
    {
        SpiFunction(SendTemp, ReceiveTemp);   //0x20000000
        Delay_ms(50);     
    }
}

/*******************************************************************
  @brief int ADXRS453Command(unsigned char Address, unsigned int SendValue, unsigned char OperateType)
         send SPI command to ADXRS453
  @param
         unsigned char Address:       Register address
         unsigned int SendValue:      Data
         unsigned char OperateType:   The command is read or write
  @return   
         unsigned int  ReceiveValue:  SPI response from ADXRS453
*******************************************************************/
int ADXRS453Command(unsigned char Address, unsigned int SendValue, unsigned char OperateType)
{
    unsigned int  SendCommand = 0;
    unsigned char SendTemp[4];
    unsigned char ReceiveTemp[4];
    unsigned int  ReceiveValue;
  
    SendCommand = Address;
    SendCommand = SendCommand<<17;
    SendCommand = SendCommand + (SendValue<<1);
  
    if (OperateType == 0)
    {
        SendCommand = SendCommand|0x40000000;          //write command
    }
    else
    {
        SendCommand = SendCommand|0x80000000;          //read command
    }
    
    SendTemp[0] = SendCommand>>24;
    SendTemp[1] = SendCommand>>16;
    SendTemp[2] = SendCommand>>8;
    SendTemp[3] = SendCommand;
    
    SpiFunction(SendTemp, ReceiveTemp);           
    
    ReceiveValue = ReceiveTemp[0];
    ReceiveValue = (ReceiveValue<<8 )+ ReceiveTemp[1];
    ReceiveValue = (ReceiveValue<<8 )+ ReceiveTemp[2];
    ReceiveValue = (ReceiveValue<<8 )+ ReceiveTemp[3];
    
    return(ReceiveValue);
}


/*******************************************************************
  @brief int ADXRS453SingleRead(unsigned char Address)
         Read a single register value from ADXRS453
  @param
         unsigned char Address:       Register address
  @return   
         int ReceiveValue:            Read data from ADXRS453
*******************************************************************/
int ADXRS453SingleRead(unsigned char Address)
{
    int ReceiveValue;
    ADXRS453Command(Address, 0, READ);
    Delay_us(10);
    ReceiveValue =  ADXRS453Command(Address, 0, READ);    //the second sequential response the read command
    ReceiveValue =  (ReceiveValue>>5)&0x0000FFFF;
    return(ReceiveValue);
}

void ADI_UpdateData(float * gyr_temp,float * temp_temp)
{
	
	rawDataRead(0x00);
	gyr_temp->No1[2]=(int16_t)(rawDataRead(0x02))/80.0f;
	*temp_temp=(int16_t)(rawDataRead(0x0C))/64;
	
}

#endif
