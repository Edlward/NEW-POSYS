#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C"
{
#endif
	#include "stdint.h"
	#include "stm32f4xx_usart.h"

	//#define USART_REC_LEN  			200  										//�����������ֽ��� 200
	//#define EN_USART1_RX 				1												//ʹ�ܣ�1��/��ֹ��0������1����
	//	  	
	//extern uint8_t  USART_RX_BUF[USART_REC_LEN]; 				//���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
	//extern uint16_t USART_RX_STA;         							//����״̬���	

	////����봮���жϽ��գ��벻Ҫע�����º궨��
	//void uart_init(uint32_t bound);
	void USART1_Init(int baudrate);

	void USART_OUT(USART_TypeDef* USARTx, const char  *Data,...);
	char *itoa(int value, char *string, int radix);
#ifdef __cplusplus
}
#endif
#endif


