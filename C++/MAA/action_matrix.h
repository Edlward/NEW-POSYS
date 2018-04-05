/**
******************************************************************************
* @file    *.h
* @author  Lxy Action
* @version
* @date
* @brief   This file contains the headers of C++ FILE
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
#ifndef __ACTION_MATRIX_H
#define __ACTION_MATRIX_H

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
#include <arm_math.h>
#include "usart.h"
typedef uint32_t size_t;

/* Exported enum -------------------------------------------------------------*/
enum  matrixUseErr {
	ERR_EQUAL = 0,
	ERR_PLUS,
	ERR_SUB,
	ERR_MUL,
	ERR_INVERSE,
	ERR_TRACE
};
enum {
	MATRIX_I = 0,
	MATRIX_ZERO,
	MATRIX_ALL_I,
	MATRIX_VOID
};
/* Exported class ------------------------------------------------------------*/
class action_matrix
{
private:
	arm_matrix_instance_f32 matrix;
	int*	referCount;
public:
	action_matrix(uint32_t len1=0, uint32_t len2=0, uint8_t kind=MATRIX_VOID);
	action_matrix(const action_matrix &m);
	~action_matrix();
	uint32_t get_row() const;
	uint32_t get_column() const;
	float get_data(uint32_t x, uint32_t y) const;
	void set_data(uint32_t x, uint32_t y, float val) const;
	void operator = (const action_matrix& y);
	void operator = (float y);
	float* operator [] (size_t i) const;

	friend  action_matrix operator + (const action_matrix& x, const action_matrix& y);
	friend	action_matrix operator - (const action_matrix& x, const action_matrix& y);
	friend	action_matrix operator * (const action_matrix& x, const action_matrix& y);

	friend	action_matrix operator * (const action_matrix& x, float y);
	friend	action_matrix operator * (float x, const action_matrix& y);
	friend	action_matrix operator + (const action_matrix& x, float y);
	friend	action_matrix operator + (float x, const action_matrix& y);
	friend	action_matrix operator - (const action_matrix& x, float y);
	friend	action_matrix operator - (float x, const action_matrix& y);

	friend	action_matrix operator / (const action_matrix& x, float y);

	friend	action_matrix operator ! (const action_matrix &x);
	friend	action_matrix operator ~ (const action_matrix &x);
	friend	float operator * (const action_matrix &x);
	friend	float tr(const action_matrix& x);
	friend	float abs(const action_matrix& x);
};

inline uint32_t action_matrix::get_row() const { return matrix.numRows; }
inline uint32_t action_matrix::get_column() const { return matrix.numCols; }
template<typename stream>
stream &operator<<(stream& os,action_matrix& item)
{
	os << '[';
	for (uint32_t i = 0; i < item.get_row(); ++i) {
		for (uint32_t j = 0; j < item.get_column(); ++j) {
			os << item.get_data(i, j);
			if (item.get_column() != j + 1)
				os << ',';
		}
		if (item.get_row() != i + 1)
			os << ';' << endl << ' ';
		else
			os << ']' << endl;
	}
	return os;
}

/* Exported overload ------------------------------------------------------- */


#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
