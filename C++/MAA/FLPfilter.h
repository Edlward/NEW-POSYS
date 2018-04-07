
/**
******************************************************************************
* @file    action_math.h
* @author  Lxy Action
* @version
* @date
* @brief   This file contains the headers of
******************************************************************************
* @attention
*
*
*
*
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLPFILTER_H
#define __FLPFILTER_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#ifdef __cplusplus
}
#include "action_matrix.h"
/* Exported class ----------------------------------------------------------- */
class flpFilter
{
private:
	action_matrix* bParam;
	action_matrix* lastInput;
	double stepSize;

	float a;
	float b;
	uint8_t variableStepSizeFlag;
public:
	flpFilter(uint16_t n, double stepValue);
	flpFilter(uint16_t n, float a, float b);
	flpFilter(const flpFilter& m);
	~flpFilter();
	float out(double rawData);
};
#endif

#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

