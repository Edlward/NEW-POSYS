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

//�ϵ��ȡ
void ReadCharacters(void);

//�洢��flash��
void writeCharacters(void);

#endif


