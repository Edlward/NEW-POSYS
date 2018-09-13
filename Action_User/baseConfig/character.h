#ifndef __CHARACTER_H
#define __CHARACTER_H

#include "stdint.h"

#define READ_FLASH_SAVE_PHYSICAL_PARA_ADDR 					0x080C0000
typedef struct{

	double  rWheelNo1;
	
	double  rWheelNo2;
	
	double  angleWheelError;
	
	double  calibrationFactor;
	
	double  gyroScale;
	
}character_t;

//�ϵ��ȡ
void ReadCharacters(void);

//�洢��flash��
void writeCharacters(void);

void CharactersReserveSectorErase(void);
#endif


