#ifndef __AHRS_MATH_H
#define __AHRS_MATH_H

#include "math.h"


#define MAA_ZERO_MATRIX		0
#define MAA_ONES_MATRIX		1
#define MAA_IDEN_MATRIX		2


typedef struct
{
	short int m_nRow;      /*!< Number of Rows in the Matrix */
	short int m_nCol;      /*!< Number of Columns in the Matrix */
	float **m_pData;       /*!< Pointer to Data in the Matrix */
} MAA_Matrix_Typedef;


//???????0???
#define MAA_fMatCreateZero(R,C) (MAA_fMatCreateInit((R),(C),MAA_ZERO_MATRIX))

//??????
#define MAA_fMatCreateUnit(R,C) (MAA_fMatCreateInit((R),(C),MAA_IDEN_MATRIX))

MAA_Matrix_Typedef* MAA_fMatCreate(int nRow, int nCol);
MAA_Matrix_Typedef *MAA_sMatCreate(int nRow, int nCol);
MAA_Matrix_Typedef* MAA_fMatCreateInit(int nRow, int nCol, int nType);
MAA_Matrix_Typedef* MAA_fMatCopy(MAA_Matrix_Typedef* pSource, MAA_Matrix_Typedef* pDest);
int MAA_MatLUP(MAA_Matrix_Typedef* pSourceDestLU, MAA_Matrix_Typedef* pPerm);
MAA_Matrix_Typedef *MAA_fMatFill(MAA_Matrix_Typedef *pMat, float fValue);
int MAA_Free(MAA_Matrix_Typedef *pMat);
	
MAA_Matrix_Typedef* MAA_fMatMulMatMT(MAA_Matrix_Typedef* pTerm1,
                                       MAA_Matrix_Typedef* pTerm2,
                                       MAA_Matrix_Typedef* pMul);

MAA_Matrix_Typedef* MAA_fMatAdd(MAA_Matrix_Typedef* pTerm1,
																MAA_Matrix_Typedef* pTerm2,
																MAA_Matrix_Typedef* pAdd);
																
MAA_Matrix_Typedef* MAA_fMatSub(MAA_Matrix_Typedef* pTerm1,
                                  MAA_Matrix_Typedef* pTerm2,
                                  MAA_Matrix_Typedef* pSub);

MAA_Matrix_Typedef* MAA_fMatMulMat(MAA_Matrix_Typedef* pTerm1,
																	MAA_Matrix_Typedef* pTerm2,
																	MAA_Matrix_Typedef* pMul);
																	
MAA_Matrix_Typedef* MAA_MatBackSubs(MAA_Matrix_Typedef* pSourceLU,
																		MAA_Matrix_Typedef* pSourceDestColumn,
																		MAA_Matrix_Typedef* pPerm,
																		MAA_Matrix_Typedef* pDest,
																		int iResultCol);
																		
MAA_Matrix_Typedef* MAA_fMatInv(MAA_Matrix_Typedef* pSource,
																MAA_Matrix_Typedef* pDest);

#endif //!__AHRS_MATH_H

