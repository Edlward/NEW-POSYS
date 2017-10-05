function [ result ] = kalmanIst( data )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
N=100;
Q(N-1)=0.006708655;
R=0.006708655;
Q(length(data))=0;
for j=1:N
    IAE_st(j)=data(j);
    result(j,1)=data(j);
end
act_value=data(N-1);
P(N-1)=0.000;
for j=N:length(data)
	predict=act_value;
	%相当于数组的左移
	for i=1:(N-1)
        IAE_st(i)=IAE_st(i+1);
    end
	IAE_st(N)=data(j)-predict;
	Cr=0;
	for i=1:N
		Cr=Cr+IAE_st(i)*IAE_st(i);
    end
	Cr=Cr/N;		%样本方差（不知道对不对，还要看原公式）
	% 预测本次的预测误差 
 	P_pre(j)=P(j-1)+Q(j-1);
	% 计算系数，求得输出值 
	Kk(j)=P_pre(j)/(P_pre(j)+R);
	act_value=predict+Kk(j)*(data(j)-predict);
	
	% 更新预测误差 
	P(j)=(1-Kk(j))*P_pre(j);
	
	% 计算并调整系统噪声 
  	Q(j)=Kk(j)*Kk(j)*Cr;%+P(j)+P(j-1);

	% 为提高滤波器的响应速度，减小滞后而设下的阈值 
% 	if Kk(j)>0.5
% 		act_value=data(j);
%     end
    result(j,1)=act_value;
end
% result(:,2)=Kk;
% result(:,3)=QQ;
% result(:,4)=P_pre;

end

