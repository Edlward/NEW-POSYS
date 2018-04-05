#include "ahrs_math.h"
#include "stdlib.h"


MAA_Matrix_Typedef* MAA_fMatCreate(int nRow, int nCol)
{
	int i;

	MAA_Matrix_Typedef* pTmp = (MAA_Matrix_Typedef *)malloc(sizeof(MAA_Matrix_Typedef));

	if (pTmp != NULL) {
		if (nRow == 0 || nCol == 0) {
			return (NULL);
		}

		pTmp->m_nRow = nRow;
		pTmp->m_nCol = nCol;

		pTmp->m_pData = malloc(nRow*sizeof(float*));

		for (i = 0;i < nRow;++i) {
			pTmp->m_pData[i] = malloc(nCol *sizeof(float));
		}
	}
	return pTmp;
}

/**
********************************************************************************
* @brief Create an Empty Matrix of Short Int
* @param  nRow : Number of Rows
* @param  nCol : Number of columns
* @retval Pointer to iNEMO_sMATRIX_TYPE
* @par Functions called:
* None
*/
MAA_Matrix_Typedef *MAA_sMatCreate(int nRow, int nCol)
{
	int i;
	MAA_Matrix_Typedef *pTmp = (MAA_Matrix_Typedef*)malloc(sizeof(MAA_Matrix_Typedef));

	if (pTmp != NULL)
	{
		// Check on null values
		if (nRow == 0 || nCol == 0)
			return(NULL);

		pTmp->m_nRow = nRow;
		pTmp->m_nCol = nCol;

		// Allocate memory for data
		pTmp->m_pData = malloc(nRow * sizeof(short int*));
		for (i = 0; i < nRow; ++i)
			pTmp->m_pData[i] = malloc(nCol * sizeof(short int));
	}

	return pTmp;
}


MAA_Matrix_Typedef* MAA_fMatCreateInit(int nRow, int nCol, int nType)
{
	MAA_Matrix_Typedef* pTmp;
	int i, j;

	if (nRow == 0 || nCol == 0)
		return(NULL);

	if ((pTmp = MAA_fMatCreate(nRow, nCol)) != NULL)
	{
		switch (nType)
		{
		case MAA_ZERO_MATRIX:
		case MAA_ONES_MATRIX:
		case MAA_IDEN_MATRIX:
			for (i = 0;i < nRow;i++) {
				for (j = 0;j < nCol; j++) {
					if (nType == MAA_ONES_MATRIX) {
						pTmp->m_pData[i][j] = 1.0f;
						continue;
					}
					if (nType == MAA_IDEN_MATRIX) {
						if (i == j) {
							pTmp->m_pData[i][j] = 1.0f;
							continue;
						}
					}
					pTmp->m_pData[i][j] = 0.0f;
				}	
			}
			break;
		}
		return (pTmp);
	}
	else {
		return (NULL);
	}
}



/**
********************************************************************************
* @brief Multiply two Matrixes
* @param[in]  pTerm1 : the first source Matrix
* @param[in]  pTerm2 : the second source Matrix
* @param[out]  pMul   : the destination Matrix
* @retval the resulting Matrix
* @par Functions called:
* None
*/
MAA_Matrix_Typedef* MAA_fMatMulMat(MAA_Matrix_Typedef* pTerm1,
																	MAA_Matrix_Typedef* pTerm2,
																	MAA_Matrix_Typedef* pMul)
{
	int	i, j, k;

	// if dimensions are wrong
	if (pMul->m_nRow != pTerm1->m_nRow ||
		pMul->m_nCol != pTerm2->m_nCol){
			return NULL;
	}
	else
	{
		for (i = 0; i < pTerm1->m_nRow; i++)
			for (j = 0; j < pTerm2->m_nCol; j++)
				for (k = 0, pMul->m_pData[i][j] = 0.0; k < pTerm1->m_nCol; k++)
					pMul->m_pData[i][j] += pTerm1->m_pData[i][k] * pTerm2->m_pData[k][j];
	}
	return(pMul);
}


/**
*******************************************************************************
* @brief Add two Matrix iNEMO_fMATRIX_TYPE
* @param pTerm1 : the first Addend source Matrix
* @param pTerm2 : the second Addend source Matrix
* @param pAdd   : the resulting Matrix
* @retval the resulting Matrix
* @par Functions called:
* None
*/
MAA_Matrix_Typedef* MAA_fMatAdd(MAA_Matrix_Typedef* pTerm1,
																MAA_Matrix_Typedef* pTerm2,
																MAA_Matrix_Typedef* pAdd)
{
	int	i, j;

	/* Check if dimensions are wrong */
	if (pAdd->m_nRow != pTerm1->m_nRow || pAdd->m_nCol != pTerm2->m_nCol){
		return NULL;
	}
	else
	{
		for (i = 0; i < pTerm1->m_nRow; ++i)
			for (j = 0; j < pTerm1->m_nCol; ++j)
				pAdd->m_pData[i][j] = pTerm1->m_pData[i][j] + pTerm2->m_pData[i][j];
	}
	return(pAdd);
}


/**
*******************************************************************************
* @brief Sub two Matrix iNEMO_fMATRIX_TYPE 
* @param pTerm1 : the first source Matrix 
* @param pTerm2 : the second source Matrix 
* @param pSub   : the resulting Matrix
* @retval the resulting Matrix 
* @par Functions called:
* None
*/
MAA_Matrix_Typedef* MAA_fMatSub(MAA_Matrix_Typedef* pTerm1,
                                  MAA_Matrix_Typedef* pTerm2,
                                  MAA_Matrix_Typedef* pSub)
{
  int	i, j;

  /* Check if dimensions are wrong */
  if ( pTerm1->m_nRow != pSub->m_nRow ||
      pTerm1->m_nCol != pSub->m_nCol )
    return NULL;
  else
  {
    for (i=0; i < pTerm1->m_nRow; ++i)
	for (j=0; j < pTerm1->m_nCol; ++j)
		pSub->m_pData[i][j] = pTerm1->m_pData[i][j] -
                  pTerm2->m_pData[i][j];
   }
   return(pSub);

}

/**
********************************************************************************
* @brief Copy pSource to pDest
* @param pSource : the source Matrix
* @param pDest : the destination Matrix
* @retval the resulting Matrix
* @par Functions called:
* None
*/
MAA_Matrix_Typedef* MAA_fMatCopy(MAA_Matrix_Typedef* pSource, MAA_Matrix_Typedef* pDest)
{
	int		i, j;

	if (pSource->m_nRow != pDest->m_nRow ||
		pSource->m_nCol != pDest->m_nCol){
			return NULL;
	}
	else
	{
		for (i = 0; i < pSource->m_nRow; i++)
			for (j = 0; j < pSource->m_nCol; j++)
				pDest->m_pData[i][j] = pSource->m_pData[i][j];
		return(pDest);
	}
}
/**
********************************************************************************
* @brief Multiply one Matrix for the Transpose of another one
* @param pTerm1 : the first source Matrix
* @param pTerm2 : the second source Matrix (MT)
* @param pMul : the destination Matrix
* @retval the resulting Matrix
* @par Functions called:
* None
*/
MAA_Matrix_Typedef* MAA_fMatMulMatMT(MAA_Matrix_Typedef* pTerm1,
                                       MAA_Matrix_Typedef* pTerm2,
                                       MAA_Matrix_Typedef* pMul)
{
   int	i, j, k;

  // if dimensions are wrong
  if ( pMul->m_nRow != pTerm1->m_nRow ||
      pMul->m_nCol != pTerm2->m_nRow )
    return NULL;
  else
  {
    for (i=0; i < pTerm1->m_nRow; i++)
	for (j=0; j < pTerm2->m_nRow; j++)
	    for (k=0, pMul->m_pData[i][j]=0.0; k < pTerm1->m_nCol; k++)
               pMul->m_pData[i][j] += pTerm1->m_pData[i][k]
                 * pTerm2->m_pData[j][k];
  }
  return(pMul);
}

/**
********************************************************************************
* @brief Find Inverse of a Matrix
* @param pSource : the source Matrix
* @param pDest   : square inverse matrix of pSource, NULL in case of fail
* @retval the resulting Matrix
* @par Functions called:
* @ref iNEMO_fMatCreate
* @ref iNEMO_fMatCopy
* @ref iNEMO_MatLUP
* @ref iNEMO_fMatFree
*/
MAA_Matrix_Typedef* MAA_fMatInv(MAA_Matrix_Typedef* pSource,
																MAA_Matrix_Typedef* pDest)
{
	MAA_Matrix_Typedef* A;
	MAA_Matrix_Typedef* B;
	MAA_Matrix_Typedef* P;
	int i, nCol, nRow;

	nCol = pSource->m_nCol;
	nRow = pSource->m_nRow;

	if (nCol != nRow)
		/* The matrix is not Square */
		return (NULL);

	A = MAA_fMatCreate(nRow, nCol);
	if (A == NULL)
		return (NULL);

	B = MAA_fMatCreate(nRow, nCol);
	if (B == NULL)
		return (NULL);

	/* P is a vector matrix */
	P = MAA_sMatCreate(nRow, 1);
	if (P == NULL)
		return (NULL);

	/* It is to avoid to modify pSource Matrix */
	MAA_fMatCopy(pSource, A);

	/* LU Decomposition and check for Singular Matrix */
	if (MAA_MatLUP(A, P) == -1)
	{
		MAA_Free(A);
		MAA_Free(B);
		MAA_Free(P);

		return (NULL);
	}

	for (i = 0; i<nCol; ++i)
	{
		MAA_fMatFill(B, 0.0f);

		B->m_pData[i][0] = 1.0f;
		MAA_MatBackSubs(A, B, P, pDest, i);
	}

	MAA_Free(A);
	MAA_Free(B);
	MAA_Free(P);
	if (pDest == NULL)
	{
		return(NULL);
	}
	else
	{
		return (pDest);
	}

}






/*******************************************************************************
* Algorithm:
*     Given a squared input matrix (n x n)
*      the algorithm returns a modified matrix and
*            a permutation vector P (n x 1)
*
* --------------------------------------------------------------------------- */
int MAA_MatLUP(MAA_Matrix_Typedef* pSourceDestLU, MAA_Matrix_Typedef* pPerm)
{
	int	i, j, k, iC;
	int	iMax;
	int	retNumPerm = 0;
	short int sTmp;
	float fP1, fP2; /* Pivot Variables */

	iC = pSourceDestLU->m_nCol;

	for (i = 0; i < iC; ++i)
		pPerm->m_pData[i][0] = (short int)(i);

	/* Partial Pivoting */
	for (k = 0; k < iC; ++k)
	{
		for (i = k, iMax = k, fP1 = 0.0f; i < iC; ++i)
		{
			/* Local ABS */
			if (pSourceDestLU->m_pData[(int)(pPerm->m_pData)[i][0]][k] > 0)
				fP2 = pSourceDestLU->m_pData[(int)pPerm->m_pData[i][0]][k];
			else
				fP2 = -pSourceDestLU->m_pData[(int)pPerm->m_pData[i][0]][k];
			if (fP2 > fP1)
			{
				fP1 = fP2;
				iMax = i;
			}
		}
		/* Row exchange, update permutation vector */
		if (k != iMax)
		{
			retNumPerm++;
			sTmp = pPerm->m_pData[k][0];
			pPerm->m_pData[k][0] = pPerm->m_pData[iMax][0];
			pPerm->m_pData[iMax][0] = sTmp;
		}

		/* Suspected Singular Matrix */
		if (pSourceDestLU->m_pData[(int)pPerm->m_pData[k][0]][k] == 0.0f)
			return (-1);

		for (i = k + 1; i < iC; ++i)
		{
			/* Calculate Mat [i][j] */
			pSourceDestLU->m_pData[(int)pPerm->m_pData[i][0]][k] =
				pSourceDestLU->m_pData[(int)pPerm->m_pData[i][0]][k] /
				pSourceDestLU->m_pData[(int)pPerm->m_pData[k][0]][k];

			/* Elimination */
			for (j = k + 1; j < iC; ++j)
			{
				pSourceDestLU->m_pData[(int)pPerm->m_pData[i][0]][j] -=
					pSourceDestLU->m_pData[(int)pPerm->m_pData[i][0]][k] *
					pSourceDestLU->m_pData[(int)pPerm->m_pData[k][0]][j];
			}


		}
	}
	return (retNumPerm);
}



/**
********************************************************************************
* @brief Fill the Matrix with a specific value
* @param  pMat : Pointer to iNEMO_fMATRIX_TYPE
* @param  fValue : the float value to fill the Matrix
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* None
*/
MAA_Matrix_Typedef *MAA_fMatFill(MAA_Matrix_Typedef *pMat, float fValue)
{
	int 	i, j;

	for (i = 0; i<pMat->m_nRow; i++)
		for (j = 0; j<pMat->m_nCol; j++)
			pMat->m_pData[i][j] = (float)(fValue);
	return (pMat);
}

/**
********************************************************************************
* @brief Back Substitution
* @param pSourceLU : the source Matrix (already LU composite)
* @param pSourceDestColumn : the column Matrix
* @param pPerm : the Permutation Vector
* @param pDest : the Destination Matrix
* @param iResultCol : the column of pSourceDestColumn
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* None
*/
MAA_Matrix_Typedef* MAA_MatBackSubs(MAA_Matrix_Typedef* pSourceLU,
																		MAA_Matrix_Typedef* pSourceDestColumn,
																		MAA_Matrix_Typedef* pPerm,
																		MAA_Matrix_Typedef* pDest,
																		int iResultCol)
{
	int i, j, k, iC;
	float fSum, fTmp;

	iC = pSourceLU->m_nCol;

	for (k = 0; k < iC; ++k)
		for (i = k + 1; i < iC; ++i)
			pSourceDestColumn->m_pData[(int)pPerm->m_pData[i][0]][0] -=
			pSourceLU->m_pData[(int)pPerm->m_pData[i][0]][k] *
			pSourceDestColumn->m_pData[(int)pPerm->m_pData[k][0]][0];

	pDest->m_pData[iC - 1][iResultCol] =
		pSourceDestColumn->m_pData[(int)pPerm->m_pData[iC - 1][0]][0] /
		pSourceLU->m_pData[(int)pPerm->m_pData[iC - 1][0]][iC - 1];


	for (k = iC - 2; k >= 0; k--)
	{
		fSum = 0.0f;

		for (j = k + 1; j < iC; ++j)
			fSum += pSourceLU->m_pData[(int)pPerm->m_pData[k][0]][j] *
			pDest->m_pData[j][iResultCol];

		fTmp = pSourceDestColumn->m_pData[(int)pPerm->m_pData[k][0]][0] - fSum;
		pDest->m_pData[k][iResultCol] = fTmp /
			pSourceLU->m_pData[(int)pPerm->m_pData[k][0]][k];
	}

	return pDest;

}


int MAA_Free(MAA_Matrix_Typedef *pMat)
{
	int i;
	
	if(pMat == NULL){
		return 0;
	}
	for(i = 0;i<pMat->m_nRow;i++){
		free(pMat->m_pData[i]);
	}
	free(pMat->m_pData);
	free(pMat);
	return 1;
}
