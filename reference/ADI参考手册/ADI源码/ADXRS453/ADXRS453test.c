/********************************************************************************
 Author : CAC (China Applications Support Team) 

 Date :   Mar, 2014

 File name :  ADXRS453.c 

 Description :	 Start up the ADXRS453, read product ID and 500 rate samples
                 

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

unsigned int AngularRateBuffer[500];
    
/*******************************************************************
  @brief int main(void);
         Read ADXRS453 product ID
         Read 500 rate samples from the sensor
  @param
         none
  @return   
         none
*******************************************************************/
int main (void)
{  
    unsigned int  DataIndex = 0;
    unsigned int  ReadValueTemp;
  
    ADuCM360Initialise();
        
    Delay1(10000);      //wait 100ms to allow for the internal circuitry to be initialized                                           
    ADXRS453StartUp();                                         //ADXRS start-up sequence
    
    ReadValueTemp = ADXRS453SingleRead(PID1);                  //read product ID
    PutString("Product ID: ");
    PutData(ReadValueTemp,BIT16LENGTH);                        //send the ID via UART
    PutChar(0x0A);  
    
    ADXRS453SensorData();  
    for (DataIndex=0; DataIndex<500; DataIndex++)              //read sensor data
    {
        AngularRateBuffer[DataIndex] = ADXRS453SensorData();   
        Delay1(200);
    }    
    for (DataIndex=0; DataIndex<500; DataIndex++)              //send the data via UART
    {
        PutData(AngularRateBuffer[DataIndex],BIT16LENGTH);     
        PutChar(0x0A);
    }

    while(1)
    { 
    }
}


