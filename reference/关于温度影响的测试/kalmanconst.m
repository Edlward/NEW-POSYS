function [ result ] = kalmanconst( data)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
Q=0.000000101;
R=0.006708655;
act_value=data(49);
PLast=0.000;
for j=1:length(data)
	predict=act_value;
	
	% 预测本次的预测误差 
%  	P_pre(j)=P(j-1)+QQ(j-1);
	P_pre(j)=PLast+Q;
	
	% 计算系数，求得输出值 
	Kk(j)=P_pre(j)/(P_pre(j)+R);
	act_value=predict+Kk(j)*(data(j)-predict);
	
	% 更新预测误差 
	PLast=(1-Kk(j))*P_pre(j);
	
	% 计算并调整系统噪声 
%  	QQ(j)=Kk(j)*Kk(j)*Cr;
% 	Q=Kk(j)*Kk(j)*Cr;

	% 为提高滤波器的响应速度，减小滞后而设下的阈值 
% 	if Kk(j)>0.5
% 		act_value=data(j);
%     end
    result(j,1)=act_value;
end
end

