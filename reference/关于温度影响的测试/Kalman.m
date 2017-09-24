function [ Kk ] = Kalman( data,Q,R)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
P_last=0;
IAE_st(50)=0;
QQ(50)=Q;
QQ(length(data))=Q;
for j=1:49
    IAE_st(j)=data(j);
    result(j,1)=data(j);
end
act_value=data(49);
for j=50:length(data)
	predict=act_value;
	%相当于数组的左移
	for i=1:49
        IAE_st(i)=IAE_st(i+1);
    end
	IAE_st(50)=data(j)-predict;
	
	% 新息的方差计算 
	Cr=0;
	for i=1:50
		Cr=Cr+IAE_st(i)*IAE_st(i);
    end
	Cr=Cr/50.0;		%样本方差（不知道对不对，还要看原公式）
	
	% 预测本次的预测误差 
	P_mid(j)=P_last+QQ(j);
	
	% 计算系数，求得输出值 
	Kk(j)=P_mid(j)/(P_mid(j)+R);
	act_value=predict+Kk(j)*(data(j)-predict);
	
	% 更新预测误差 
	P_last=(1-Kk(j))*P_mid(j);
	
	% 计算并调整系统噪声 
	QQ(j+1)=Kk(j)*Kk(j)*Cr;

	% 为提高滤波器的响应速度，减小滞后而设下的阈值 
	if Kk(j)>0.5
		act_value=data(j);
    end
    result(j,1)=act_value;
end
end

