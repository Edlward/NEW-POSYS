#include "signalProcess.h"
#include "arm_math.h"
#include "usart.h"
float averageWithRemoveErr(float *data,uint32_t len)
{
	float re;
	float std;
	float *dataWithErr;
	
	dataWithErr=new float[len];
	
	arm_mean_f32(data,len,&re);
	arm_std_f32(data,len,&std);
	
	if(std>0.0000001f)
	{
		uint32_t count=0;
		for(uint32_t i=0;i<len;i++)
		{
			if(fabs(data[i]-re)<(5*std))
			{
				dataWithErr[count]=data[i];
				count++;
			}
//			if(abs(data[i])>10)
//			{
//				cout<<"err point"<<endl;
//			}
		}
		arm_mean_f32(dataWithErr,count,&re);
		
//		if(len-count>0)
//		{
//			cout<<endl;
//			cout<<len<<endl;
//			cout<<"discard: "<<(len-count)<<endl;
//			cout<<"std: "<<std<<endl;
//			for(uint32_t i=0;i<len;i++)
//			{
//				cout<<data[i]<<endl;
//			}
//		}
	}
	
	delete[] dataWithErr;
	
	return re;
}
threeAxis meanData(threeAxis *data,uint32_t len)
{
	threeAxis re;
	float* x=new float[len];
	float* y=new float[len];
	float* z=new float[len];
	
	for(uint32_t i=0;i<len;i++)
	{
		x[i]=data[i].x;
		y[i]=data[i].y;
		z[i]=data[i].z;
	}
	arm_mean_f32(x,len,&re.x);
	arm_mean_f32(y,len,&re.y);
	arm_mean_f32(z,len,&re.z);
	
	delete[] x;
	delete[] y;
	delete[] z;
	
	return re;
}
threeAxis stdData(threeAxis *data,uint32_t len)
{
	threeAxis re;
	float* x=new float[len];
	float* y=new float[len];
	float* z=new float[len];
	
	for(uint32_t i=0;i<len;i++)
	{
		x[i]=data[i].x;
		y[i]=data[i].y;
		z[i]=data[i].z;
	}
	arm_std_f32(x,len,&re.x);
	arm_std_f32(y,len,&re.y);
	arm_std_f32(z,len,&re.z);
	
	delete[] x;
	delete[] y;
	delete[] z;
	
	return re;
}