#include "ahrs.h"
#include <stdlib.h>


MAA_Matrix_Typedef* pMat_P;					/* 误差协方差矩阵								[7][7]					*/
MAA_Matrix_Typedef* pMat_Q;					/* 过程噪声协方差矩阵						[7][7]					*/
MAA_Matrix_Typedef* pMat_Pnew;			/* 保存更新后的误差协方差值			[7][7]					*/
MAA_Matrix_Typedef* pMat_Ka;				/* kalman滤波增益系数						[7][3]					*/
#ifdef MAG_ENABLE
MAA_Matrix_Typedef* pMat_Km;
#endif
MAA_Matrix_Typedef* pMat_Ra;				/* 加速度计测量噪声							[3][3]					*/
#ifdef MAG_ENABLE
MAA_Matrix_Typedef* pMat_Rm;
#endif
MAA_Matrix_Typedef* pMat_Hja;				/* 加速度计的雅克比矩阵					[3][7]					*/
#ifdef MAG_ENABLE
MAA_Matrix_Typedef* pMat_Hjm;
#endif
MAA_Matrix_Typedef* pMat_Fsys;			/* 状态矩阵											[7][7]					*/


float fAccRef;											/* 重力的值？？？																*/
#ifdef MAG_ENABLE
float fMagRef;
#endif

float fSV[7];												/* 状态向量，前四个元素为四元数，后3个应该为加速度*/
float fHalfDt;											/* 周期的一半																		*/


/**
  * @brief  卡尔曼滤波初始化
  *
  * @param[in]  pSensorData
	* @param[in]  pAngle
	* @param[in]  pQuat
  * @retval None
  */
void AHRS_Init(sensor_data* pSensorData, euler_angles* pAngle, Quaternion* pQuat)
{
	short int  i;
	
	pMat_P = MAA_fMatCreateZero(7, 7);				
	pMat_Fsys = MAA_fMatCreateUnit(7, 7);					/* 单位矩阵									*/
	pMat_Q = MAA_fMatCreateZero(7, 7);				
	
	pMat_Pnew = MAA_fMatCreateZero(7, 7);
	
	pMat_Ra = MAA_fMatCreateZero(3, 3);
	pMat_Ka = MAA_fMatCreateZero(7, 3);
#ifdef MAG_ENABLE
	pMat_Rm = MAA_fMatCreateZero(3, 3);
	pMat_Km = MAA_fMatCreateZero(7, 3);
#endif

	for (i = 0; i < 7; ++i){
		pMat_P->m_pData[i][i] = 1.0e-1f;
		pMat_Q->m_pData[i][i] = 5.0e-7f;
	}
	
	for (i = 0;i<3;i++){
		pMat_Ra->m_pData[i][i] = pSensorData->m_VarAcc;
#ifdef MAG_ENABLE
		pMat_Rm->m_pData[i][i] = pSensorData->m_VarMag;
#endif
	}
	
	pMat_Hja = MAA_fMatCreateZero(3, 7);
#ifdef MAG_ENABLE
	pMat_Hjm = MAA_fMatCreateZero(3, 7);
#endif

	//Option 1: {1,0,0,0,0,0,0}
	fSV[0] = 1;
	for (i = 1;i < 7;i++) {
		fSV[i] = 0;
	}
	
	pSensorData->m_fAcc[0] = 0.0f;
	pSensorData->m_fAcc[1] = 0.0f;
	pSensorData->m_fAcc[2] = 0.0f;

	pSensorData->m_fGyro[0] = 0.0f;
	pSensorData->m_fGyro[1] = 0.0f;
	pSensorData->m_fGyro[2] = 0.0f;
#ifdef MAG_ENABLE
	pSensorData->m_fMag[0] = 0.0f;
	pSensorData->m_fMag[1] = 0.0f;
	pSensorData->m_fMag[2] = 0.0f;
#endif
	
	pAngle->roll = 0.0f;
	pAngle->pitch = 0.0f;
	pAngle->yaw = 0.0f;

	fHalfDt = pSensorData->m_fDeltaTime / 2;
	fAccRef = 0.0f;
#ifdef MAG_ENABLE
	fMagRef = 0.0f;
#endif

	for (i = 0;i < 3;i++) {
		fAccRef += (pSensorData->m_fAccRef[i] * pSensorData->m_fAccRef[i]);
#ifdef MAG_ENABLE
		fMagRef += (pSensorData->m_fMagRef[i] * pSensorData->m_fMagRef[i]);
#endif
	}
	fAccRef = sqrt(fAccRef);
#ifdef MAG_ENABLE
	fMagRef = sqrt(fMagRef);
#endif
}

/**
  * @brief   卡尔曼滤波更新
  *
  * @param[in]  pExtSensorData
	* @param[in]  pExtAngle
	* @param[in]  pExtQuat
  * @retval None
  */
void AHRS_Update(sensor_data* pExtSensorData, euler_angles* pExtAngle, Quaternion* pExtQuat)
{
	int i;
	float fGyroX, fGyroY, fGyroZ;
	Quaternion fSVnorm;
	float fH[4];
	float fTempNorm;
	
	pExtSensorData->nCount++;

	/* 计算陀螺仪角度增量						*/
	fGyroX = (pExtSensorData->m_fGyro[0] - fSV[4])*fHalfDt;
	fGyroY = (pExtSensorData->m_fGyro[1] - fSV[5])*fHalfDt;
	fGyroZ = (pExtSensorData->m_fGyro[2] - fSV[6])*fHalfDt;

	/* 四元数微分方程的系数[4][4]*/
	//pMat_Fsys->m_pData[0][0] = 1.0f;
	pMat_Fsys->m_pData[0][1] = -fGyroX;
	pMat_Fsys->m_pData[0][2] = -fGyroY;
	pMat_Fsys->m_pData[0][3] = -fGyroZ;
	pMat_Fsys->m_pData[1][0] =	fGyroX;
	//pMat_Fsys->m_pData[1][1] = 1.0f;
	pMat_Fsys->m_pData[1][2] =	fGyroZ;
	pMat_Fsys->m_pData[1][3] = -fGyroY;
	pMat_Fsys->m_pData[2][0] =	fGyroY;
	pMat_Fsys->m_pData[2][1] = -fGyroZ;
	//pMat_Fsys->m_pData[2][2] = 1.0f;
	pMat_Fsys->m_pData[2][3] =	fGyroX;
	pMat_Fsys->m_pData[3][0] =	fGyroZ;
	pMat_Fsys->m_pData[3][1] =	fGyroY;
	pMat_Fsys->m_pData[3][2] = -fGyroX;
	//pMat_Fsys->m_pData[3][3] = 1.0f;


	pMat_Fsys->m_pData[0][4] =	fSV[1] * fHalfDt;
	pMat_Fsys->m_pData[0][5] =	fSV[2] * fHalfDt;
	pMat_Fsys->m_pData[0][6] =	fSV[3] * fHalfDt;
	pMat_Fsys->m_pData[1][4] = -fSV[0] * fHalfDt;
	pMat_Fsys->m_pData[1][5] =	fSV[3] * fHalfDt;
	pMat_Fsys->m_pData[1][6] = -fSV[2] * fHalfDt;
	pMat_Fsys->m_pData[2][4] = -fSV[3] * fHalfDt;
	pMat_Fsys->m_pData[2][5] = -fSV[0] * fHalfDt;
	pMat_Fsys->m_pData[2][6] =	fSV[1] * fHalfDt;
	pMat_Fsys->m_pData[3][4] =	fSV[2] * fHalfDt;
	pMat_Fsys->m_pData[3][5] = -fSV[1] * fHalfDt;
	pMat_Fsys->m_pData[3][6] = -fSV[0] * fHalfDt;

	/**
	  ***********************************************************
	  *预测部分
	  ***********************************************************
	  */
	/* first-order runge-kutta,fGyroX已经乘过周期和半径 */
	fSVnorm[0] = fSV[0] - (fGyroX*fSV[1]) - (fGyroY*fSV[2]) - (fGyroZ*fSV[3]);
	fSVnorm[1] = fSV[1] + (fGyroX*fSV[0]) + (fGyroZ*fSV[2]) - (fGyroY*fSV[3]);
	fSVnorm[2] = fSV[2] + (fGyroY*fSV[0]) - (fGyroZ*fSV[1]) + (fGyroX*fSV[3]);
	fSVnorm[3] = fSV[3] + (fGyroZ*fSV[0]) + (fGyroY*fSV[1]) - (fGyroX*fSV[2]);

	//
	for (i = 0;i < 4;++i) {
		fSV[i] = fSVnorm[i];
	}

	/* P = F*P*F' + Q			*/
	pMat_Pnew = MAA_PropagateP(pMat_P, pMat_Fsys, pMat_Q, pMat_Pnew);

	/* 	*/
	MAA_fMatCopy(pMat_Pnew, pMat_P);

	/**
	  **********************************************************
	  * 更新过程
	  **********************************************************
	  */
#ifdef MAG_ENABLE
	if(pExtSensorData->nCount%2 == 0){
#endif
		fH[0] = -2.0*fAccRef*(fSV[1] * fSV[3] - fSV[0] * fSV[2]);
		fH[1] = -2.0*fAccRef*(fSV[0] * fSV[1] + fSV[2] * fSV[3]);
		fH[2] = -fAccRef*(fSV[0] * fSV[0] - fSV[1] * fSV[1] - fSV[2] * fSV[2] + fSV[3] * fSV[3]);

		/* Its Jacobian matrix */
		pMat_Hja->m_pData[0][0] =  2.0*fAccRef*fSV[2];
		pMat_Hja->m_pData[0][1] = -2.0*fAccRef*fSV[3];
		pMat_Hja->m_pData[0][2] =  2.0*fAccRef*fSV[0];
		pMat_Hja->m_pData[0][3] = -2.0*fAccRef*fSV[1];

		pMat_Hja->m_pData[1][0] = -2.0*fAccRef*fSV[1];
		pMat_Hja->m_pData[1][1] = -2.0*fAccRef*fSV[0];
		pMat_Hja->m_pData[1][2] = -2.0*fAccRef*fSV[3];
		pMat_Hja->m_pData[1][3] = -2.0*fAccRef*fSV[2];

		pMat_Hja->m_pData[2][0] = -2.0*fAccRef*fSV[0];
		pMat_Hja->m_pData[2][1] =  2.0*fAccRef*fSV[1];
		pMat_Hja->m_pData[2][2] =  2.0*fAccRef*fSV[2];
		pMat_Hja->m_pData[2][3] = -2.0*fAccRef*fSV[3];

		/* Ka = P * Hja' * (Hja * P * Hja' + Ra)^-1  */
		pMat_Ka = MAA_CalculateK(pMat_P, pMat_Hja, pMat_Ra, pMat_Ka);

		/* Update State Vector */
		for (i = 0;i < 7;++i) {
			fSV[i] += pMat_Ka->m_pData[i][0] * (pExtSensorData->m_fAcc[0] - fH[0]) +
				pMat_Ka->m_pData[i][1] * (pExtSensorData->m_fAcc[1] - fH[1]) +
				pMat_Ka->m_pData[i][2] * (pExtSensorData->m_fAcc[2] - fH[2]);
		}
		/* P = (I - Ka*Hja)*P */
		pMat_Pnew = MAA_UpdateP(pMat_P, pMat_Ka, pMat_Hja, pMat_Pnew);

		MAA_fMatCopy(pMat_Pnew, pMat_P);

#ifdef MAG_ENABLE
	}

	if(pExtSensorData->nCount++%2 == 0)
	{
    fH[0] =         fMagRef * (fSV[0]*fSV[0] + fSV[1]*fSV[1] - fSV[2]*fSV[2] - fSV[3]*fSV[3]);
    fH[1] =  2.0f * fMagRef * (fSV[1]*fSV[2] - fSV[3]*fSV[0]);
    fH[2] =  2.0f * fMagRef * (fSV[1]*fSV[3] + fSV[2]*fSV[0]);
		
		  /* Calculate the relative Jacobian Matrix */
    pMat_Hjm->m_pData[0][0] =   2.0f * fMagRef * fSV[0];
    pMat_Hjm->m_pData[0][1] =   2.0f * fMagRef * fSV[1];
    pMat_Hjm->m_pData[0][2] = - 2.0f * fMagRef * fSV[2];
    pMat_Hjm->m_pData[0][3] = - 2.0f * fMagRef * fSV[3];
    pMat_Hjm->m_pData[1][0] = - 2.0f * fMagRef * fSV[3];
    pMat_Hjm->m_pData[1][1] =   2.0f * fMagRef * fSV[2];
    pMat_Hjm->m_pData[1][2] =   2.0f * fMagRef * fSV[1];
    pMat_Hjm->m_pData[1][3] = - 2.0f * fMagRef * fSV[0];
    pMat_Hjm->m_pData[2][0] =   2.0f * fMagRef * fSV[2];
    pMat_Hjm->m_pData[2][1] =   2.0f * fMagRef * fSV[3];
    pMat_Hjm->m_pData[2][2] =   2.0f * fMagRef * fSV[0];
    pMat_Hjm->m_pData[2][3] =   2.0f * fMagRef * fSV[1];
		
		/* Km = P * Hjm' * (Hjm * P * Hjm' + Rm)^-1  */
    pMat_Km = MAA_CalculateK(pMat_P, pMat_Hjm, pMat_Rm, pMat_Km);

    /* State Vector Update */
    for (i=0; i < 7; ++i)
      fSV[i] += pMat_Km->m_pData[i][0] * ( pExtSensorData->m_fMag[0] - fH[0]) +
        pMat_Km->m_pData[i][1] * (pExtSensorData->m_fMag[1] - fH[1]) +
          pMat_Km->m_pData[i][2] * (pExtSensorData->m_fMag[2] - fH[2]);

    /* Update of Error Covariance Matrix */
    /* P = (I - Km*Hjm*P */
    pMat_Pnew = MAA_UpdateP(pMat_P, pMat_Km, pMat_Hjm, pMat_Pnew);
    /* Copy the new P in P */
    MAA_fMatCopy(pMat_Pnew, pMat_P);
	}
#endif
	
	/* 四元数归一化				*/
	fTempNorm = 1.0f / sqrt(fSV[0] * fSV[0] + fSV[1] * fSV[1] + fSV[2] * fSV[2] + fSV[3] * fSV[3]);
	for (i = 0;i < 4;++i) {
		fSV[i] = fTempNorm*fSV[i];
		pExtQuat[0][i] = fSV[i];
	}

	pExtAngle->roll = atan2(2.0f*(fSV[0] * fSV[1] + fSV[2] * fSV[3]),
		1.0f - 2.0f*(fSV[1] * fSV[1] + fSV[2] * fSV[2]));
	pExtAngle->pitch = asin(-2.0f*(fSV[1] * fSV[3] - fSV[0] * fSV[2]));

	pExtAngle->yaw = atan2(2.0f*(fSV[1] * fSV[2] + fSV[0] * fSV[3]),
		(1.0f - 2.0f*(fSV[2] * fSV[2] + fSV[3] * fSV[3])));
}




















/////////////////////////////////////////////////////////////////
/**
* @brief ?????????
* @param  pPoldMat  : ?? P ??
* @param  pStateMat : ????
* @param  pQMat     : the Q Covariance Matrix of noise on state
* @param  pPnewMat  : ?? P ??
* @retval Pointer to MAA_Matrix_Typedef
* @par Functions called:
* @ref iNEMO_fMatCreate
* @ref iNEMO_fMatMulMat
* @ref iNEMO_fMatMulMatMT
* @ref iNEMO_fMatAdd
* @ref iNEMO_fMatFree
* @details
* <b>Algorithm:</b> <br>
*    P = F * P F' + Q <br>
*     became <br>
*    pPnewMat = pStateMat * pPoldMat * pStateMat' + pQMat <br>
*
*  This operation is divided in steps: <br>
* 1)    pPnewMat = pStateMat * pPoldMat <br>
*
* 2)    pPnewMat = pPnewMat * pStateMat' <br>
*
* 3)    pPnewMat += Q <br>
*/
MAA_Matrix_Typedef*		MAA_PropagateP(MAA_Matrix_Typedef* pPoldMat,
	MAA_Matrix_Typedef* pStateMat,
	MAA_Matrix_Typedef* pQMat,
	MAA_Matrix_Typedef* pPnewMat)
{
	/*  ?????		*/
	MAA_Matrix_Typedef* pTmp;

	pTmp = MAA_fMatCreate(pPoldMat->m_nRow, pPoldMat->m_nCol);

	/* First Step: pTmp = pStateMat * pPoldMat */
	pTmp = MAA_fMatMulMat(pStateMat, pPoldMat, pTmp);
	/* Second Step: pPnewMat = pTmp * pStateMat' */
	pPnewMat = MAA_fMatMulMatMT(pTmp, pStateMat, pPnewMat);
	/* Third Step: pPnewMat += Q */
	pPnewMat = MAA_fMatAdd(pPnewMat, pQMat,pPnewMat);

	///* Delete the Temporary Matrix before to exit */
	//MAA_fMatFree(pTmp);
	
	MAA_Free(pTmp);
	return pPnewMat;
}


/**
* @brief Calculate the Kalman Gain
* @param pPMat  : the P matrix
* @param pHJMat : the HJ matrix
* @param pRMat  : the R matrix
* @param pKMat  : the K matrix
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* @ref iNEMO_fMatCreate
* @ref iNEMO_fMatMulMatMT
* @ref iNEMO_fMatMulMat
* @ref iNEMO_fMatAdd
* @ref iNEMO_fMatInv
* @ref iNEMO_fMatFree
* @details
* <b>Algorithm:</b> <br>
*    K = P * Hj' * (Hj * P * Hj' + R)^-1 <br>
*     became <br>
*    pKMat = pPMat * pHJMat' * (pHJMat * pPMat * pHJMat' + pRMat)^-1 <br>
*
* 1) pTmp1 = pPMat * pHJMat' <br>
*
* 2) pTmp2 = pHJMat * (pPMat * pHJMat') = pHJMat * pTmp1 <br>
*
* 3) pTmp2 = pTmp2 + pRMat <br>
*
* 4) pRInvMat = (pTmp2)^-1 <br>
*
* 5) pKMat = pTmp1 * pRInvMat <br>
*/

MAA_Matrix_Typedef* MAA_CalculateK(MAA_Matrix_Typedef* pPMat,
	MAA_Matrix_Typedef* pHJMat,
	MAA_Matrix_Typedef* pRMat,
	MAA_Matrix_Typedef* pKMat)
{
	/* Internal Variables */
	MAA_Matrix_Typedef* pTmp1;
	MAA_Matrix_Typedef* pTmp2;
	MAA_Matrix_Typedef* pRInvMat;

	pTmp1 = MAA_fMatCreate(pPMat->m_nRow, pHJMat->m_nRow);
	pTmp2 = MAA_fMatCreate(pHJMat->m_nRow, pTmp1->m_nCol);

	/* First Step: pTmp1 = pPMat * pHJMat' */
	pTmp1 = MAA_fMatMulMatMT(pPMat, pHJMat, pTmp1);

	/* Second Step: pTmp2 = pHJMat * (pPMat * pHJMat') = pHJMat * pTmp1 */
	pTmp2 = MAA_fMatMulMat(pHJMat, pTmp1, pTmp2);

	/* Third Step: pTmp2 = pTmp2 + pRMat */
	pTmp2 = MAA_fMatAdd(pTmp2, pRMat, pTmp2);

	/* Fourth Step: pRInvMat = (pTmp2)^-1 */
	pRInvMat = MAA_fMatCreate(pTmp2->m_nRow, pTmp2->m_nCol);

	pRInvMat = MAA_fMatInv(pTmp2, pRInvMat);

	/* Fifth Step: pKMat = pTmp1 * pRInvMat */
	pKMat = MAA_fMatMulMat(pTmp1, pRInvMat, pKMat);

	///* Delete the Temporary Matrix before to exit */
	//iNEMO_fMatFree(pTmp1);
	//iNEMO_fMatFree(pTmp2);
	//iNEMO_fMatFree(pRInvMat);
	MAA_Free(pTmp1);
	MAA_Free(pTmp2);
	MAA_Free(pRInvMat);

	return pKMat;

}




/**
* @brief Update the Error Covariance Matrix
* @param pPoldMat : old P matrix
* @param pKMat    : the Gain Matrix
* @param pHJMat   : the Jacobian Matrix
* @param pPnewMat : the new P matrix
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* @ref iNEMO_fMatCreateZero
* @ref iNEMO_fMatCreateUnit
* @ref iNEMO_fMatMulMat
* @ref iNEMO_fMatSub
* @ref iNEMO_fMatFree
* @details
* <b>Algorithm:</b> <br>
*    P = (I - K * Hj) * P <br>
*     became <br>
*    pKMat = (I - pKMat * pHJMat) * pPoldMat <br>
*
*
*    where pHJMat has this structure <br>
*     | * * * *  | 0 0 0 |  <br>
*     | * * * *  | 0 0 0 |  <br>
*     | * * * *  | 0 0 0 |  <br>
*
*/

MAA_Matrix_Typedef* MAA_UpdateP(MAA_Matrix_Typedef* pPoldMat,
	MAA_Matrix_Typedef* pKMat,
	MAA_Matrix_Typedef* pHJMat,
	MAA_Matrix_Typedef* pPnewMat)
{
	MAA_Matrix_Typedef* pTmp;
	MAA_Matrix_Typedef* pI;

	pTmp = MAA_fMatCreateZero(pPoldMat->m_nRow, pPoldMat->m_nCol);
	pI = MAA_fMatCreateUnit(pPoldMat->m_nRow, pPoldMat->m_nCol);

	// First Step: pTmp = (pKMat * pHJMat)
	pTmp = MAA_fMatMulMat(pKMat, pHJMat, pTmp);

	// Second Step: pTmp = I - pTmp
	pTmp = MAA_fMatSub(pI, pTmp, pTmp);

	// Third Step: pPnewMat = pTmp * pPoldMat
	pPnewMat = MAA_fMatMulMat(pTmp, pPoldMat, pPnewMat);

	//iNEMO_fMatFree(pTmp);
	//iNEMO_fMatFree(pI);
	MAA_Free(pTmp);
	MAA_Free(pI);

	return (pPnewMat);
}
