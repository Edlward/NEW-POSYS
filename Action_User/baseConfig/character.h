#ifndef __CHARACTER_H
#define __CHARACTER_H

#include "stdint.h"
typedef struct{

	double  rWheelNo1;
	
	double  rWheelNo2;
	
	double  angleWheelError;
	
	double  calibrationFactor;
	
	uint32_t  gyroScale;
	
}character_t;

//上电读取
void ReadCharacters(void);

//存储到flash里
void writeCharacters(void);

#endif


