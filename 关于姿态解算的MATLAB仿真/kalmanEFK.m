function [ result ] = kalmanEFK( data)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
R=0.003;
Q=0.003;
P_last=0.01;
t=[1:1:49]';
y=polyfit(t,data(1:49),1);%1x2的行向量
for j=1:50
    IAE_st(j)=data(j)-y(1)*j-y(2);
    result(j)=0;
end
for j=50:length(data)
    y=polyfit(t,data(j-49:j-1),1);%1x2的行向量
	predict=y(1)*50+y(2);
	%相当于数组的左移
	for i=1:49
        IAE_st(i)=IAE_st(i+1);
    end
	IAE_st(50)=data(j)-predict;
	
	% 新息的方差计算 
	Cr=sum(IAE_st(1:50).^2)/50;		%样本方差（不知道对不对，还要看原公式）
	
	% 预测本次的预测误差 
	P_mid=P_last+Q;
	
	% 计算系数，求得输出值 
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(data(j)-predict);
	
	% 更新预测误差 
	P_last=(1-Kk)*P_mid;
	
	% 计算并调整系统噪声 
 	Q=Kk*Kk*Cr;

	if Kk>0.5
		act_value=data(j);
    end
    result(j)=act_value;
end
end

