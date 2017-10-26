#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f4xx_usart.h"

void USART6_Init(uint32_t BaudRate);
void USART3_Init(uint32_t baudRate);
void UART5_Init(uint32_t BaudRate);
void USART_OUT(USART_TypeDef* USARTx,const char *Data,...);
char *itoa(int value, char *string, int radix);
void USART1_Init(uint32_t BaudRate);
void USART2_Init(uint32_t BaudRate);
void USART_OUT_F(float value);
void USART_Enter(void);
#endif

