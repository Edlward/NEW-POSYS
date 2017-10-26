/*******************************************************************************
 Author : Gowrishankar  
         India Applications Support Team

 Date : Febraury 2014

 File name : ADuCM360Driver.c

 Description :

 Hardware plateform : ADuCM360 	
********************************************************************************/

#include "ADuCM360.h"
#include "ADuCM360Driver.h"

#include "DioLib.h"
#include "ClkLib.h"
#include "WdtLib.h"
#include "spilib.h"
#include "UrtLib.h"

unsigned char ucComRx = 0;
unsigned char ucTxBufferEmpty  = 0;       // Used to indicate that the UART Tx buffer is empty
unsigned char szTemp[16] = "";            // Used to store string before printing to UART
unsigned char nLen = 0;
unsigned char i = 0;
unsigned char ucWaitForUart = 0;          // Used by calibration routines to wait for user input
unsigned char ucWaitForRX = 0;  

// Delay Function
void Delay1(unsigned long int DelayTime)			  //  Delay Function
{ 
	 unsigned char i;

	 while(DelayTime>0)
	 { 
		 for(i=0;i<32;i++) 
	 	 {
			 ;
		 }
	  	 DelayTime--;
	 }
}


// Initialise the controller
void ADuCM360Initialise(void)									//  Initialising the controller
{
	 WdtCfg(T3CON_PRE_DIV1,T3CON_IRQ_EN,T3CON_PD_DIS);      // Disable Watchdog timer resets
   
   //Disable clock to unused peripherals
   ClkDis(CLKDIS_DISI2CCLK|CLKDIS_DISPWMCLK|
      CLKDIS_DIST0CLK|CLKDIS_DIST1CLK|CLKDIS_DISDACCLK);  // Disable unused clock
   ClkCfg(CLK_CD0,CLK_HF,CLKSYSDIV_DIV2EN_DIS,CLK_UCLKCG);// Select CD0 for CPU clock - 16Mhz clock
   ClkSel(CLK_CD0,CLK_CD0,CLK_CD0,CLK_CD7);               // Select CD0 for SPI clocks
	
   ////Configure Port 1 pins for SPI operation
	 DioCfgPin(pADI_GP1,PIN4,2);                           // Configure P1.4 as MISO
	 DioCfgPin(pADI_GP1,PIN5,2);                           // Configure P1.5 as SCLK   
 	 DioCfgPin(pADI_GP1,PIN6,2);                           // Configure P1.6 as MOSI
   DioCfgPin(pADI_GP1,PIN7,2);                           // Configure P1.7 as SS

   ////Configure SPI0 for operation 
   SpiBaud(pADI_SPI0,1,SPIDIV_BCRST_EN);                 // Confiure SPI0 baud rate for 4MHz
	 SpiCfg(pADI_SPI0,SPICON_MOD_TX4RX4,SPICON_MASEN_EN,SPICON_CON_EN|SPICON_RXOF_EN|SPICON_ZEN_EN|
      SPICON_TIM_TXWR|SPICON_ENABLE_EN);
	
	 SpiFifoFlush(pADI_SPI0,SPICON_TFLUSH_EN,SPICON_RFLUSH_EN);
	
	 UARTINIT ();                                          // Initialise UART
   NVIC_EnableIRQ(UART_IRQn);                            // Enable UART interrupt
}


// Function for sending and receiving data through SPI
void SpiFunction(unsigned char OutputBuff[],unsigned char InputBuff[], unsigned int NoOfBytes)
{	
	 int i;

   for(i=0;i<(NoOfBytes);i++)
   {
     SpiTx(pADI_SPI0, OutputBuff[i]);										// Send data
   }
   
   for(i=0;i<(NoOfBytes);i++)
   {
     while((SpiSta(pADI_SPI0) & 0x0700) == 0x0000){}
     InputBuff[i] = SpiRx(pADI_SPI0);										// Receive data
   }
   /**/
}


//Initialize UART
void UARTINIT (void)
{
	 //Select IO pins for UART.
	 pADI_GP0->GPCON |= 0x3C;                      // Configure P0.1/P0.2 for UART	
		
	 DioCfgPin(pADI_GP0,PIN1,3);                   // Configure P0.1 as SIN
	 DioCfgPin(pADI_GP0,PIN2,3);                   // Configure P0.2 as SOUT
	
	 UrtCfg(pADI_UART,B115200,COMLCR_WLS_8BITS,0);  // setup baud rate for 115200, 8-bits,stop bits -1 no parity.
	 UrtMod(pADI_UART,COMMCR_DTR,0);               // Data terminal ready, Setup modem bits
	 UrtIntCfg(pADI_UART,COMIEN_ERBFI|COMIEN_ETBEI|COMIEN_ELSI|COMIEN_EDSSI|COMIEN_EDMAT|COMIEN_EDMAR);   // Setup UART IRQ sources
}


// Send string through UART
void PutChar(unsigned char SendCh)				   //output a char via UART
{          
	 UrtTx(pADI_UART,SendCh);
	 while((UrtLinSta(pADI_UART) & 0x40) == 0x00) {}	  // Wait for Tx buffer empty bit to be set	
}

// Send Integer through UART
void PutData(unsigned int SendData, unsigned char BytesNumber)
{
	 unsigned char SendTemp[8];
	 unsigned char i;

   SendTemp[0] = (SendData>>28)&0x0F;
	 SendTemp[1] = (SendData>>24)&0x0F;
	 SendTemp[2] = (SendData>>20)&0x0F;
	 SendTemp[3] = (SendData>>16)&0x0F;
	 SendTemp[4] = (SendData>>12)&0x0F;
	 SendTemp[5] = (SendData>>8)&0x0F;
	 SendTemp[6] = (SendData>>4)&0x0F;
	 SendTemp[7] = (SendData)&0x0F;

	 for(i=(8-BytesNumber);i<8;i++)
	 {
		 if(SendTemp[i]<10)
        PutChar(SendTemp[i]+'0');
		 else
        PutChar(SendTemp[i]+'A'-10);		
	 }
}


//unsigned int PutString(char *SendStr)						//output a string via UART
unsigned int PutString(char SendStr[])	
{          
	 unsigned int Length=0;

	 while(SendStr[Length] != 0)
	 {
		 PutChar(SendStr[Length]);
		 Length++;
	 }

	 return Length;
}


// Receive string through UART
unsigned int RecString(void)
{
	 unsigned int temp;	
   ucWaitForRX = 1;
	 while(ucWaitForRX){}      
	 temp = ucComRx;	
	 return(temp);
} 


// Receive data from UART
unsigned int ReceiveFromUart(unsigned char NoOfBytes)
{
   unsigned int TempData[4],value=0x00000000;
   unsigned char i;
  
   for(i=0;i<NoOfBytes;i++)
   {
     Delay1(0x6000);
     TempData[i] = RecString();
   }
   for(i=0;i<NoOfBytes;i++)
   {
     value <<= 8;
     value |= TempData[i];
   }
  
   return(value);
}


// UART Interrupt handler
void UART_Int_Handler ()
{
   volatile unsigned char ucCOMSTA0 = 0;
   volatile unsigned char ucCOMIID0 = 0;
   volatile unsigned int uiUartCapTime = 0;
   
   ucCOMSTA0 = UrtLinSta(pADI_UART);         // Read Line Status register
   ucCOMIID0 = UrtIntSta(pADI_UART);         // Read UART Interrupt ID register         
   if ((ucCOMIID0 & 0x2) == 0x2)             // Transmit buffer empty
   {
      ucTxBufferEmpty = 1;
   }
   if ((ucCOMIID0 & 0x4) == 0x4)             // Receive byte
   {
      ucComRx   = UrtRx(pADI_UART);
		  ucWaitForRX = 0;
   }
}
