function [ result ] = kalman( data ,Q,R)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
P_last=0.01;
act_value=0;
for j=1:length(data)
	predict=act_value;

	% 预测本次的预测误差 
	P_mid=P_last+Q;
	
	% 计算系数，求得输出值 
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(data(j)-predict);
	
	% 更新预测误差 
	P_last=(1-Kk)*P_mid;

    result(j)=act_value;
end

end

