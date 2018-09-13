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

//上电读取
void ReadCharacters(void);

//存储到flash里
void writeCharacters(void);

void CharactersReserveSectorErase(void);
#endif


