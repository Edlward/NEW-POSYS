function [ result ] = kalmanconst( data)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
Q=0.000000101;
R=0.006708655;
act_value=data(49);
PLast=0.000;
for j=1:length(data)
	predict=act_value;
	
	% Ԥ�Ȿ�ε�Ԥ����� 
%  	P_pre(j)=P(j-1)+QQ(j-1);
	P_pre(j)=PLast+Q;
	
	% ����ϵ����������ֵ 
	Kk(j)=P_pre(j)/(P_pre(j)+R);
	act_value=predict+Kk(j)*(data(j)-predict);
	
	% ����Ԥ����� 
	PLast=(1-Kk(j))*P_pre(j);
	
	% ���㲢����ϵͳ���� 
%  	QQ(j)=Kk(j)*Kk(j)*Cr;
% 	Q=Kk(j)*Kk(j)*Cr;

	% Ϊ����˲�������Ӧ�ٶȣ���С�ͺ�����µ���ֵ 
% 	if Kk(j)>0.5
% 		act_value=data(j);
%     end
    result(j,1)=act_value;
end
end

