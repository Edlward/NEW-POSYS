/********************************************************************************
 Author : CAC (China Applications Support Team) 

 Date :   Mar, 2014

 File name :  ADXRS453.c 

 Description :	 ADXRS453 SPI communication

 Hardware plateform : 	EVAL-ADuCM360MKZ and EVAL-ADXRS453Z-S
 Connection:
                 EVAL-ADuCM360MKZ       EVAL-ADXRS453Z-S
                               
                 P1.4:MISO,             P1 Pin 18: MISO             
                 P1.5:SCLK,             P1 Pin 16: SCLK                                                            
                 P1.6:MOSI,             P1 Pin 17: MOSI
                 P1.7:CS,               P1 Pin 20: CS
********************************************************************************/

#include <ADuCM360.h>

#include "UrtLib.h"
#include "ClkLib.h"
#include "WdtLib.h"
#include "IntLib.h"
#include "DioLib.h"
#include "SpiLib.h"

#include "ADXRS453.h"
#include "ADuCM360Driver.h"

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
    
    SpiFunction(SendTemp, ReceiveTemp, 4);           
    
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
    Delay1(1);
    ReceiveValue =  ADXRS453Command(Address, 0, READ);    //the second sequential response the read command
    ReceiveValue =  (ReceiveValue>>5)&0x0000FFFF;
    return(ReceiveValue);
}

/*******************************************************************
  @brief int  ADXRS453SensorData(void)
         SPI sensor data command 
  @param
         none
  @return   
         unsigned int  ReceiveValue: sensor data  
*******************************************************************/
int  ADXRS453SensorData(void)
{
    unsigned char SendTemp[4];
    unsigned char ReceiveTemp[4];
    unsigned int  ReceiveValue;
  
    SendTemp[0] = 0x20;
    SendTemp[1] = 0x00;  
    SendTemp[2] = 0x00;
    SendTemp[3] = 0x00;
  
    SpiFunction(SendTemp, ReceiveTemp, 4);
    ReceiveValue = ReceiveTemp[0];
    ReceiveValue = (ReceiveValue<<8 )+ ReceiveTemp[1];
    ReceiveValue = (ReceiveValue<<8 )+ ReceiveTemp[2];
    ReceiveValue = (ReceiveValue<<8 )+ ReceiveTemp[3]; 
  
    ReceiveValue =  (ReceiveValue>>10)&0x0000FFFF;
    return(ReceiveValue);
}

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
    SpiFunction(SendTemp, ReceiveTemp, 4);       //0x20000003
    Delay1(5000);
  
    SendTemp[0] = 0x20;
    SendTemp[1] = 0x00;  
    SendTemp[2] = 0x00;
    SendTemp[3] = 0x00; 
    for (i=0; i<3; i++)
    {
        SpiFunction(SendTemp, ReceiveTemp, 4);   //0x20000000
        Delay1(5000);      
    }
}
