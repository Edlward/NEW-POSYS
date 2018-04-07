/**
******************************************************************************
* @file    action_math.cpp
* @author  lxy
* @version
* @date
* @brief
******************************************************************************
* @attention
*
*
*
*
******************************************************************************
*/
/* Includes -------------------------------------------------------------------*/
#include "flpFilter.h"
#include <math.h>
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
flpFilter::flpFilter(uint16_t n, double stepValue)
	:stepSize(stepValue),  a(0), b(0), variableStepSizeFlag(0)
{
	bParam = new action_matrix(1, n, MATRIX_ZERO);
	lastInput = new action_matrix(1, n, MATRIX_ZERO);

	for (uint16_t i = 0; i < n; i++)
	{
		if (i == 0) 
		{
			(*bParam)[0][i] = 1;
		}
		else
		{
			(*bParam)[0][i] = 0;
		}
	}
}
flpFilter::flpFilter(uint16_t n, float alpha, float beta)
	:a(alpha),b(beta),variableStepSizeFlag(1)
{
	bParam = new action_matrix(1, n, MATRIX_ZERO);
	lastInput = new action_matrix(1, n, MATRIX_ZERO);

	for (uint16_t i = 0; i < n; i++)
	{

			(*bParam)[0][i] = 1.0f/n;;
	}
}
flpFilter::flpFilter(const flpFilter & m)
	:stepSize(m.stepSize),a(m.a),b(m.b), variableStepSizeFlag(m.variableStepSizeFlag)
{
	bParam = new action_matrix(1, m.bParam->get_column(), MATRIX_ZERO);
	lastInput = new action_matrix(1, m.lastInput->get_column(), MATRIX_ZERO);
	(*bParam) = (*m.bParam);
	(*lastInput) = (*m.lastInput);
}
flpFilter::~flpFilter()
{
	delete bParam;
	delete lastInput;
}

static double dataSt[5];

float flpFilter::out(double rawData)
{
	double outTemp, err;
	outTemp = static_cast<double>(((*bParam) * (!(*lastInput)))[0][0]);
	err = rawData - outTemp;
	if (variableStepSizeFlag)
	{
		stepSize = b*(1 - exp(-a*err*err));
	}
	(*bParam) = (*bParam) + stepSize*err*(*lastInput);
	for(uint8_t i=0;i<5;i++)
	{
		dataSt[i]=(*bParam)[0][i];
	}
	
	for (uint16_t i = lastInput->get_column() - 1; i > 0; i--)
	{
		(*lastInput)[0][i] = (*lastInput)[0][i - 1];
	}
	(*lastInput)[0][0] = rawData;

	
	
	return outTemp;
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/


