#ifndef __USART_H
#define __USART_H

#include "stm32f4xx_usart.h"
#include "config.h"
#ifdef TEST_SUMMER
#define DMA_SEND_SIZE   100
#else
#define DMA_SEND_SIZE   36
#endif

#define USART_USED   		USART1


void USART3DMAInit(uint32_t BaudRate);
void USART1DMAInit(uint32_t BaudRate);
void USART_OUTByDMAF(float x);
void USART_EnterByDMA(void);
void USART_SendDataToDMA_USATR1(uint8_t data);
void USART_SendDataToDMA_USART3(uint8_t data);

void USART_OUT(USART_TypeDef* USARTx,const char *Data,...);
char *itoa(int value, char *string, int radix);
void USART_OUT_F(float value);
void USART_Enter(void);

#endif

