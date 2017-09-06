#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f4xx_usart.h"

void USART_OUT(USART_TypeDef* USARTx,const uint8_t *Data,...);
char *itoa(int value, char *string, int radix);
void USART_OUT_F(float value);
void GyroscopeUsartInit(uint32_t BaudRate);
void WirelessBluetoothUsartInit(uint32_t BaudRate);
#endif

