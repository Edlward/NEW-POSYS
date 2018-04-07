/**
******************************************************************************
* @file    action_matrix.cpp
* @author  lxy zlq
* @version V1.0
* @date    2016.9.6
* @brief   this is a tool for calculation of matrix
******************************************************************************
* @attention
*
*
*
*
******************************************************************************
*/
/* Includes -------------------------------------------------------------------*/
#include "arm_math.h"
#include <cstdint>
#include "action_matrix.h"
//#include "usart.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported class functions ---------------------------------------------------*/


/**
* @brief        创建矩阵
* @param  len1: 矩阵的行数
* @param  len2: 矩阵的列数
* @param  kind: MATRIX_I       单位阵
								MATRIX_ZERO    0矩阵
								MATRIX_ALL_I   所有元素为1的矩阵
* @retval None
*/
action_matrix::action_matrix(uint32_t len1, uint32_t len2, uint8_t kind)
{
	matrix.numRows=len1;
	matrix.numCols=len2;
	referCount=0;
	matrix.pData=0;
	if(len1*len2)
	{
		matrix.pData=new float[len1*len2];
		referCount=new int(0);	
	}
	
	
	switch (kind)
	{
	case MATRIX_I:
		for (uint32_t i = 0; i < len1; i++)
			for (uint32_t j = 0; j < len2; j++)
			{
				if (i != j)
					(*this)[i][j] = 0;
				else
					(*this)[i][j] = 1;
			}
		break;
	case MATRIX_ZERO:
		for (uint32_t i = 0; i < len1; i++)
			for (uint32_t j = 0; j < len2; j++)
			{
				(*this)[i][j] = 0;
			}
		break;
	case MATRIX_ALL_I:
		for (uint32_t i = 0; i < len1; i++)
			for (uint32_t j = 0; j < len2; j++)
			{
				(*this)[i][j] = 1;
			}
		break;
	case MATRIX_VOID:
		break;
	default:
		while (1);
		//break;
	}
}



/**
* @brief  ????
* @attention ???????
*/
action_matrix::action_matrix(const action_matrix &m)
	: matrix(m.matrix),referCount(m.referCount)
{
	(*referCount)++;
}
/**
* @berif 矩阵析构
*/
action_matrix::~action_matrix()
{
	//通过引用计数来判断矩阵内容是否需要被释放
	if(*referCount)
	{
		(*referCount)--;
	}
	else
	{
		delete[] matrix.pData;
		delete referCount;
	}
}
/**
* @brief  	 获得矩阵中某一行某一列的值
* @param  x: 行数
* @param  y: 列数
* @retval data[x][y]
*/
float action_matrix::get_data(uint32_t x, uint32_t y) const
{
	return matrix.pData[x*matrix.numCols+y];
}
/**
* @brief  改变某一行或者列的值
* @param  x: 行数
* @param  y: 列数
* @retval None
*/
void action_matrix::set_data(uint32_t x, uint32_t y, float val) const
{
	matrix.pData[x*matrix.numCols+y]=val;
}

/**
* @brief  ????
* @retval none
*/
float* action_matrix::operator [] (size_t i) const
{
	return &(matrix.pData[i*matrix.numCols]);
}
/**
* @brief  	 矩阵赋值（深度复制）
* @retval 	 none
*/
void action_matrix::operator = (const action_matrix& y)
{
	if (this->get_column() == 0 && this->get_row() == 0 && this->matrix.pData==0 && this->referCount==0)
	{
		this->matrix.numRows=y.get_row();
		this->matrix.numCols=y.get_column();
		this->matrix.pData=new float[matrix.numRows*matrix.numCols];
		referCount=new int(0);
	}
	for (uint32_t i = 0; i <matrix.numRows; i++)
	{
		for (uint32_t j = 0; j < matrix.numCols; j++)
		{
			(*this)[i][j]=y[i][j];
		}
	}
}
void action_matrix::operator = (float y)
{
	if (this->get_column() == 0 && this->get_row() == 0)
	{
		this->matrix.numRows=1;
		this->matrix.numCols=1;
		this->matrix.pData=new float[matrix.numRows*matrix.numCols];
		referCount=new int(0);
	}
	for (uint32_t i = 0; i <matrix.numRows; i++)
	{
		for (uint32_t j = 0; j < matrix.numCols; j++)
		{
			(*this)[i][j]=y;
		}
	}
}
/**
* @brief  ????
* @attention ????????????????
* @param  none
* @retval none
*/
action_matrix operator + (const action_matrix& x, const action_matrix& y)
{
	if (x.get_column() != y.get_column() || x.get_row() != y.get_row())
	{
		throw ERR_PLUS;
	}
	else
	{
		action_matrix result(x.get_row(), x.get_column());
		arm_mat_add_f32(&x.matrix,&y.matrix,&result.matrix);                                         
		return result;
	}
}
action_matrix operator+(const action_matrix& x, float y)
{

	if (x.get_row() != x.get_column())
	{
		throw ERR_PLUS;
	}
	else
	{
		action_matrix result(x.get_row(), x.get_column());
		for (uint32_t i = 0; i < x.get_row(); i++)
		{
			for (uint32_t j = 0; j < x.get_column(); j++)
			{
				if (i == j)
					result.set_data(i, j, x.get_data(i, j) + y);
				else
					result.set_data(i, j, x.get_data(i, j));
			}
		}

		return result;
	}
}
action_matrix operator+(float x, const action_matrix& y)
{
	if (y.get_row() != y.get_column())
	{
		throw ERR_PLUS;
	}
	else
	{
		action_matrix result(y.get_row(), y.get_column());
		for (uint32_t i = 0; i < y.get_row(); i++)
		{
			for (uint32_t j = 0; j < y.get_column(); j++)
			{
				if (i == j)
					result.set_data(i, j, y.get_data(i, j) + x);
				else
					result.set_data(i, j, y.get_data(i, j));
			}
		}
		return result;
	}
}


/**
* @brief  ????
* @param  none
* @retval none
*/
action_matrix operator - (const action_matrix& x, const action_matrix& y)
{
	if (x.get_column() != y.get_column() || x.get_row() != y.get_row())
	{
		throw ERR_SUB;
	}
	else
	{
		action_matrix result(x.get_row(), x.get_column());
		arm_mat_sub_f32(&x.matrix,&y.matrix,&result.matrix);
		return result;
	}
}
action_matrix operator - (const action_matrix& x, float y)
{
	return x + (-y);
}
action_matrix operator - (float x, const action_matrix& y)
{
	if (y.get_row() != y.get_column())
	{
		throw ERR_PLUS;
	}
	else
	{
		action_matrix result(y.get_row(), y.get_column());
		for (uint32_t i = 0; i < y.get_row(); i++)
		{
			for (uint32_t j = 0; j < y.get_column(); j++)
			{
				if (i == j)
					result.set_data(i, j, x - y.get_data(i, j));
				else
					result.set_data(i, j, y.get_data(i, j));
			}
		}
		return result;
	}
}


/**
* @brief  ????
* @param  none
* @retval none
*/
action_matrix operator * (const action_matrix& x, const action_matrix& y)
{
	if (x.get_column() != y.get_row())
	{
		throw ERR_MUL;
	}
	else
	{
		action_matrix result(x.get_row(), y.get_column());
		
		arm_mat_mult_f32(&x.matrix,&y.matrix,&result.matrix);
		
		return result;
	}
}
action_matrix operator * (const action_matrix& x, float y)
{

	action_matrix result(x.get_row(), x.get_column());
	arm_mat_scale_f32(&x.matrix,y,&result.matrix);
	return result;
}
action_matrix operator * (float x, const action_matrix& y)
{
	action_matrix result(y.get_row(), y.get_column());
	arm_mat_scale_f32(&y.matrix,x,&result.matrix);
	return result;
}

action_matrix operator / (const action_matrix& x, float y)
{
	action_matrix result(x.get_row(), x.get_column());
	arm_mat_scale_f32(&x.matrix,1/y,&result.matrix);
	return result;
}
/**
* @brief  ????
* @param  none
* @retval none
*/
action_matrix operator !(const action_matrix &x)
{
	action_matrix result(x.get_column(), x.get_row());
	arm_mat_trans_f32(&x.matrix,&result.matrix);
	return result;
}
/**
* @brief  ????
* @attention ???delete_data,??????,????,????????
* @param  none
* @retval none
*/
action_matrix operator ~(const action_matrix &x) //????
{
	if (x.get_column() != x.get_row())
	{
		throw ERR_INVERSE;
	}
	action_matrix x_copy(x.get_row(),x.get_column());
	x_copy=x;
	action_matrix result(x.get_row(),x.get_column());
	arm_mat_inverse_f32(&x_copy.matrix,&result.matrix);
	return result;
}
/**
* @brief  ??????
* @attention ???delete_data,??????,??..
* @param  none
* @retval none
*/
float operator *(const action_matrix &x)
{
	float out = 0;
	if (x.get_row()>2)
	{
		action_matrix *data;
		for (uint8_t i = 0; i<x.get_row(); i++)
		{
			data = new action_matrix(x.get_row() - 1, x.get_row() - 1);
			for (uint8_t j = 0; j<data->get_row(); j++)
				for (uint8_t w = 0; w<data->get_row(); w++)
				{
					if (w<i)
						data->set_data(j, w, x.get_data(j + 1, w));
					else
						data->set_data(j, w, x.get_data(j + 1, w + 1));
				}
			if (i % 2 == 0)
				out += (*(*data))*x.get_data(0, i);
			else
				out -= (*(*data))*x.get_data(0, i);
			delete data;
		}
	}
	else
		out = x.get_data(0, 0)*x.get_data(1, 1) - x.get_data(0, 1)*x.get_data(1, 0);
	return out;
}
/**
* @brief  矩阵求迹（对角线元素之和）
* @param  action_matrix: 矩阵
* @retval 迹
*/
float tr(const action_matrix& x)
{
	if (x.get_column() != x.get_row())
	{
		throw ERR_TRACE;
	}
	float re = 0;
	for (uint32_t i = 0; i<x.get_column(); i++)
	{
		re += x.get_data(i, i);
	}
	return re;
}
float abs(const action_matrix& x)
{
	float sum=0;
	for (uint32_t i = 0; i<x.get_row(); i++)
	{
		for (uint32_t j = 0; j<x.get_column(); j++)
		{
			sum+=x[i][j]*x[i][j];
		}
	}
	return sqrt(sum);
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
