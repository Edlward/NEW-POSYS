#include "config.h"
#include "kalman.h"


/*算法中 H,φ,Γ均为一*/
/*算法中 H,φ,Γ均为一*/
double KalmanFilterZ1(double measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
  static double P_last=0.00001;   //上一次的预测误差
  static double P_mid;        //对预测误差的预测
  static double Kk;           //滤波增益系数
  
  static double Q=0.00001;       //系统噪声        
  double R=0.001;      //测量噪声 
  static double data=0.0;
  //令预测值为上一次的真实值
  predict=act_value;
  
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return measureData;
  }
  else if(ignore==100){
    predict=data/99.0;
  }else
    ignore=101;
  
  /* 预测本次的预测误差 */
  P_mid=P_last+Q;
  
  /* 计算系数，求得输出值 */
  Kk=P_mid/(P_mid+R);
  
  act_value=predict+Kk*(measureData-predict);
  
  /* 更新预测误差 */
  P_last=(1-Kk)*P_mid;
  
  return act_value;
}

/*算法中 H,φ,Γ均为一*/
double KalmanFilterZ(double measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
  static double P_last=0.00001;   //上一次的预测误差
  static double P_mid;        //对预测误差的预测
  static double Kk;           //滤波增益系数
  
  static double Q=0.00001;       //系统噪声        
  double R=0.001;      //测量噪声 
  static double data=0.0;
  //令预测值为上一次的真实值
  predict=act_value;
  
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return measureData;
  }
  else if(ignore==100){
    predict=data/99.0;
  }else
    ignore=101;
  
  /* 预测本次的预测误差 */
  P_mid=P_last+Q;
  
  /* 计算系数，求得输出值 */
  Kk=P_mid/(P_mid+R);
  
  act_value=predict+Kk*(measureData-predict);
  
  /* 更新预测误差 */
  P_last=(1-Kk)*P_mid;
  
  return act_value;
}
/*算法中 H,φ,Γ均为一*/
double KalmanFilterX(double measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
  static double P_last=0.0000003169;   //上一次的预测误差
  static double P_mid;        //对预测误差的预测
  static double Kk;           //滤波增益系数
  
  static double Q=0.00000000002514;       //系统噪声         
  double R=0.0;      //测量噪声 
  static double IAE_st[50];    //记录的新息
  static double data=0.0;
  double Cr=0;                //新息的方差
  
  predict=act_value;
  
  /* 新息的方差计算 */
  memcpy(IAE_st,IAE_st+1,392);
  IAE_st[49]=measureData-predict;
  
  Cr=0;
  for(int i=0;i<50;i++)
  {
    Cr=Cr+IAE_st[i]*IAE_st[i];
  }
  Cr=Cr/50.0f;
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return 0.0;
  }
  else if(ignore==100){
    predict=data/99.0;
    //USART_OUT_F(predict);
    //USART_Enter();
  }else
    ignore=101;
  
  /* 预测本次的预测误差 */
  P_mid=P_last+Q;
  
  /* 计算系数，求得输出值 */
  Kk=P_mid/(P_mid+R);
  
  act_value=predict+Kk*(measureData-predict);
  
  /* 更新预测误差 */
  P_last=(1-Kk)*P_mid;
  
  /* 计算并调整系统噪声 */
  Q=Kk*Kk*Cr;
  
  /* 为提高滤波器的响应速度，减小滞后而设下的阈值 */
  if(Kk>0.5)
    act_value=measureData;
  
  return act_value;
}

/*算法中 H,φ,Γ均为一*/
double KalmanFilterY(double measureData)
{
   static double act_value=0;  //实际值
  double predict;             //预测值
  
  static double P_last=0.01;   //上一次的预测误差
  static double P_mid;        //对预测误差的预测
  static double Kk;           //滤波增益系数
  
  static double Q=0.003;       //系统噪声        
  double R=0.0;      //测量噪声 
  static double IAE_st[50];    //记录的新息
  static double data=0.0;
  double Cr=0;                //新息的方差
  //令预测值为上一次的真实值
  predict=act_value;
  
  /* 新息的方差计算 */
  memcpy(IAE_st,IAE_st+1,196);
  IAE_st[49]=measureData-predict;
  
  Cr=0;
  for(int i=0;i<50;i++)
  {
    Cr=Cr+IAE_st[i]*IAE_st[i];
  }
  Cr=Cr/50.0f;
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return 0.0;
  }
  else if(ignore==100){
    predict=data/99.0;
    //USART_OUT_F(predict);
    //USART_Enter();
  }else
    ignore=101;
  
  /* 预测本次的预测误差 */
  P_mid=P_last+Q;
  
  /* 计算系数，求得输出值 */
  Kk=P_mid/(P_mid+R);
  
  act_value=predict+Kk*(measureData-predict);
  
  /* 更新预测误差 */
  P_last=(1-Kk)*P_mid;
  
  /* 计算并调整系统噪声 */
  Q=Kk*Kk*Cr;
  
  /* 为提高滤波器的响应速度，减小滞后而设下的阈值 */
  if(Kk>0.5)
    act_value=measureData;
  
  return act_value;
}

