#ifndef CONFIG_H
#define CONFIG_H
#include "usart.h"
#define PERIOD					      0.01f

//每圈脉冲数  顺时针为正
#define STDPULSE 			4096.0f

//#define CORRECT
typedef struct{
	
	struct{
		uint32_t shootInitPos;
		uint32_t shootTempPos;
		uint32_t shootAimPos;
		uint32_t shootAimPosLast;
		int shootTempVel;
		int shootAimVel;
	}shootMotor_t;
	
	struct{
		int aimVelCode;
		float aimRoateVel;
		float aimLineAngle;
		float roateAngle;
		int aimVelFour[4];
		float angle;
		float x;
		float y;
	}walk_t;
	
	uint16_t gasMotionFlag;
	uint16_t posSystemReady;
	uint16_t controlMode;
	
}Robot_t;
#endif


