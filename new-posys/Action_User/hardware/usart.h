#ifndef __USART_H
#define __USART_H

#include "stm32f4xx_usart.h"

#define DMA_SEND_SIZE   36

#ifdef NEW_BOARD
#define SEND_USART	USART3
#else
#define SEND_USART	USART1
#endif

void usart_Init(uint32_t BaudRate);
void USART_OUT(USART_TypeDef* USARTx,const char *Data,...);
char *itoa(int value, char *string, int radix);
void USART_OUT_F(float value);
void USART_Enter(void);
void USART_SendDataToDMA(uint8_t data);
#endif

