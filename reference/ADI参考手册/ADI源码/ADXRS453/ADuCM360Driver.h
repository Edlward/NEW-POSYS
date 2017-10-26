/*******************************************************************************
 Author : Gowrishankar 
          India Applications Support Team

 Date : Febraury 2014

 File name : ADuCM360Driver.h

 Description :

 Hardware plateform : ADuCM360 	
********************************************************************************/

#ifndef ADUCM360_DRIVER_H
#define ADUCM360_DRIVER_H

// Mode
#define WRITE                0x00
#define READ								 0x01

//UART send length
#define BIT16LENGTH          4
#define BIT24LENGTH          6
#define BIT32LENGTH          8

//-------------------------------------------------------
//				EXTERNAL FUNCTION DECLARATIONS
//-------------------------------------------------------
void ADuCM360Initialise(void);
void Delay1(unsigned long int DelayTime);	
void SpiFunction(unsigned char OutputBuff[],unsigned char InputBuff[], unsigned int NoOfBytes);

//-------------------------------------------------------
//				INTERNAL FUNCTION DECLARATIONS
//-------------------------------------------------------
void UARTINIT (void);

//-----------------------------------------------------
//				UART EXTERNAL FUNCTION DECLARATIONS
//-----------------------------------------------------
void PutChar(unsigned char SendCh);
void PutData(unsigned int SendInt, unsigned char BytesNumber);
unsigned int RecString(void);

unsigned int PutString(char SendStr[]);
unsigned int ReceiveFromUart(unsigned char NoOfBytes);

#endif

