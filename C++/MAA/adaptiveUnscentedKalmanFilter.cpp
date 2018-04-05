#include "adaptiveUnscentedKalmanFilter.h"
#include "arm_math.h"

AUKF::AUKF(float a,float b,float c,
					 const action_matrix& Xx,const action_matrix& Pp,const action_matrix& Qp,const action_matrix& Rr,const action_matrix& QLl,
					 action_matrix (*pre)(const action_matrix&,const action_matrix&),
					 action_matrix (*measure)(const action_matrix&),uint32_t len)
:a(a),b(b),microParam(c),rLen(len),predict(pre),measure(measure)
{
	X=Xx;
	P=Pp;
	Q=Qp;
	R=Rr;
	QL=QLl;
	r=new action_matrix[len];
}



std::vector<action_matrix> AUKF::unscentedTrans(const action_matrix& X,const action_matrix& P)
{
	std::vector<action_matrix> vec;
	action_matrix temp(X.get_row(),1);
	action_matrix L(P.get_row(),P.get_column(),MATRIX_ZERO);
	vec.push_back(X);
	
	//microParam=a*a*P.get_column()-P.get_column();
	
	/*
		可以证明得到P是一个正定矩阵，
		所以P=L*(!L)中L为下三角矩阵，
		可以通过LU分解的特殊形式得到，
		如下
	*/
	for (uint32_t i = 0; i < P.get_row(); i++)
	{
		for (uint32_t j = 0; j < i + 1; j++)
		{
			L[i][j] = P[i][j];
			for (uint32_t k = 0; k < j; k++)
			{
				L[i][j] -= L[i][k] * L[j][k];
			}
			if (i != j)
			{
				L[i][j] /= L[j][j];
				if(fabs(L[j][j])<0.0000001f)
					while(1)
						cout<<"err1"<<endl;
			}
			else
			{
				L[i][j] = sqrt(L[i][j]);
				if(L[i][j]<0)
					while(1)
						cout<<"err2"<<endl;
			}
		}
	}
	for(uint32_t i=0;i<L.get_column();i++)
	{
		for(uint32_t j=0;j<L.get_row();j++)
		{
			temp[j][0]=L[j][i]*sqrt(microParam+P.get_column());
		}
		vec.push_back(X+temp);
		vec.push_back(X-temp);
	}
	
	return vec;
}
action_matrix AUKF::updateM(std::vector<action_matrix> in)
{
	action_matrix temp(in[0].get_row(),in[0].get_column(),MATRIX_ZERO);
//	for(uint32_t i=0;i<in.size();i++)
//	{
//		if(i==0)
//		{
//			temp=temp+microParam*in[i]/(P.get_column()+microParam);
//		}
//		else
//		{
 //			temp=temp+in[i]/(P.get_column()+microParam)/2.0;
//		}
//	}
	temp=in[0];
	return temp;
}
action_matrix AUKF::updateC(std::vector<action_matrix>in1,const action_matrix& aver1,
														std::vector<action_matrix>in2,const action_matrix& aver2)
{
	action_matrix temp(in1[0].get_row(),in2[0].get_row(),MATRIX_ZERO);
	for(uint32_t i=0;i<in1.size();i++)
	{
		if(i==0)
		{
			temp=temp+(in1[i]-aver1)*(!(in2[i]-aver2))*(microParam/(P.get_column()+microParam)+1-a*a+b);
		}
		else
		{
			temp=temp+(in1[i]-aver1)*(!(in2[i]-aver2))/(P.get_column()+microParam)/2.0;
		}
	}
	return temp;
}
void AUKF::predictX(const action_matrix& U)
{
	std::vector<action_matrix> out;
	std::vector<action_matrix> in;
	in=unscentedTrans(X,P);  //得到sigma点
	for(auto obj:in)
	{
		out.push_back((*predict)(obj,U)); //计算sigma点对应的预测状态
	}
	X=updateM(out);		//求解每一个预测状态的均值
	P=updateC(out,X,out,X);
	//cout<<X[0][0]<<'\t'<<X[1][0]<<'\t'<<X[2][0]<<'\t'<<X[3][0]<<'\t'<<tr(P)<<'\t';
	//求预测的协方差
	P=P+QL*Q*(!QL);
	
}
void AUKF::predictZ(void)
{
	std::vector<action_matrix> X_sigma;
	std::vector<action_matrix> Z_sigma;
	
	X_sigma=unscentedTrans(X,P);  //得到预测状态的sigma点
	for(auto obj:X_sigma)
	{
		Z_sigma.push_back((*measure)(obj));   //计算sigma点对应的预测测量值
	}

	Z_pre=updateM(Z_sigma);				//计算预测测量值的均值
	
//	cout<<endl;
//	cout<<Z_pre<<endl;
//	cout<<endl;
	
	P_R=updateC(Z_sigma,Z_pre,Z_sigma,Z_pre); //计算预测测量值在不考虑测量误差时，与真实测量值的误差
	
	P_XZ=updateC(X_sigma,X,Z_sigma,Z_pre);
	
}
void AUKF::estimateX(const action_matrix& Y)
{
	P_R=P_R+R*factorR;	//计算预测测量值的协方差
	
	Kk=P_XZ*(~P_R);							//计算增益系数
	X=X+Kk*(Y-Z_pre);					//更新状态
	
	P=P-Kk*P_R*(!Kk);	//更新估计误差的协方差
}
void AUKF::adaptiveR(const action_matrix& Y)
{
	//更新新息序列
	for(uint32_t count=0;count<rLen-1;count++)
	{
		r[count]=r[count+1];
	}
	r[rLen-1]=Y-Z_pre;
	
	factorR=1.0f;
	
	if(r[0].get_row()!=0&&r[0].get_column()!=0)
	{
		//更新计算新息的协方差矩阵
		action_matrix Cr=r[0]*(!r[0]);
		for(uint32_t count=1;count<rLen;count++)
		{
			Cr=Cr+r[count]*(!r[count]);
		}
		Cr=Cr/static_cast<float>(rLen-1);
		factorR=tr(Cr-P_R)/tr(R);
		if(factorR<1)
			factorR=1;
	}
}


action_matrix AUKF::filter(const action_matrix& Y,const	action_matrix& U)
{
	predictX(U);
	predictZ();
	adaptiveR(Y);
	estimateX(Y);
//	std::vector<action_matrix> out1;
//	std::vector<action_matrix> out2;
//	std::vector<action_matrix> in;
//	in=unscentedTrans(X,P);  //得到sigma点
//	for(auto obj:in)
//	{
//		out1.push_back((*predict)(obj,U)); //计算sigma点对应的预测状态
//	}
//	X=updateM(out1);		//求解每一个预测状态的均值
//	P=updateC(out1,X,out1,X);
//	
//	//求预测的协方差
//	P=P+QL*Q*(!QL);
//	in.clear();
//	
//	in=unscentedTrans(X,P);  //得到预测状态的sigma点
//	for(auto obj:in)
//	{
//		out2.push_back((*measure)(obj));   //计算sigma点对应的预测测量值
//	}

//	action_matrix Z_pre(out2[0].get_row(),out2[0].get_column());
//	action_matrix P_R(R.get_row(),R.get_column());
//	Z_pre=updateM(out2);				//计算预测测量值的均值
//	
//	P_R=updateC(out2,Z_pre,out2,Z_pre); //计算预测测量值在不考虑测量误差时，与真实测量值的误差
//	
//	//更新新息序列
//	for(uint32_t count=0;count<rLen-1;count++)
//	{
//		r[count]=r[count+1];
//	}
//	r[rLen-1]=Y-Z_pre;

//	float factorR=1.0f;
//	if(r[0].get_row()!=0&&r[0].get_column()!=0)
//	{
//		//更新计算新息的协方差矩阵
//		action_matrix Cr=r[0]*(!r[0]);
//		for(uint32_t count=1;count<rLen;count++)
//		{
//			Cr=Cr+r[count]*(!r[count]);
//		}
//		Cr=Cr/static_cast<float>(rLen-1);
//		factorR=tr(Cr-P_R)/tr(R);
//		if(factorR<1)
//			factorR=1;
//	}
//	
//	P_R=P_R+R*factorR;	//计算预测测量值的协方差
//	
//	action_matrix P_XZ(X.get_row(),Z_pre.get_row());
//	P_XZ=updateC(in,X,out2,Z_pre);
//	
//	Kk=P_XZ*(~P_R);							//计算增益系数
//	X=X+Kk*(Y-Z_pre);					//更新状态
//	
//	P=P-Kk*P_R*(!Kk);	//更新估计误差的协方差
	
	
	return X;
}
AUKF::~AUKF()
{
	delete[] r;
}
