function [ result ] = kalman( data)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
P(49)=0.001;
Q=0.0000000015;
R=0.0006708655;
QQ(length(data))=Q;
for j=1:50
    IAE_st(j)=data(j);
    result(j,1)=data(j);
end
act_value=data(49);
% QQ(49)=0.000000008236918;
% P_last=0.000006478590639000001;
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
	Cr=Cr/50;		%样本方差（不知道对不对，还要看原公式）
	
	% 预测本次的预测误差 
% 	P_pre(j)=P(j-1)+QQ(j-1);
	P_pre(j)=P(j-1)+Q;
	
	% 计算系数，求得输出值 
	Kk(j)=P_pre(j)/(P_pre(j)+R);
	act_value=predict+Kk(j)*(data(j)-predict);
	
	% 更新预测误差 
	P(j)=(1-Kk(j))*P_pre(j);
	
	% 计算并调整系统噪声 
% 	QQ(j)=Kk(j)*Kk(j)*Cr;

	% 为提高滤波器的响应速度，减小滞后而设下的阈值 
	if Kk(j)>0.5
		act_value=data(j);
    end
    result(j,1)=act_value;
end
result(:,2)=Kk;
result(:,3)=QQ;
result(:,4)=P_pre;
end

