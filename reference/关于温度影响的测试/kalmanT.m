function [ result ] = kalmanT( data)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
Q(1)=0.000000009957;
R=0.02;
Q(49)=Q(1);
for j=1:50
    IAE_st(j)=data(j);
    result(j,1)=data(j);
end
act_value=mean(data(1:49));
P(49)=0.00001505;
for j=50:length(data)
	predict=act_value;
	%�൱�����������
	for i=1:49
        IAE_st(i)=IAE_st(i+1);
    end
	IAE_st(50)=data(j)-predict;
	
	% ��Ϣ�ķ������ 
	Cr=0;
	for i=1:50
		Cr=Cr+IAE_st(i)*IAE_st(i);
    end
	Cr=Cr/50;		%���������֪���Բ��ԣ���Ҫ��ԭ��ʽ��
	
	% Ԥ�Ȿ�ε�Ԥ����� 
%  	P_pre(j)=P(j-1)+QQ(j-1);
	P_pre(j)=P(j-1)+Q(j-1);
	
	% ����ϵ����������ֵ 
	Kk(j)=P_pre(j)/(P_pre(j)+R);
	act_value=predict+Kk(j)*(data(j)-predict);
	
	% ����Ԥ����� 
	P(j)=(1-Kk(j))*P_pre(j);
	
	% ���㲢����ϵͳ���� 
%  	QQ(j)=Kk(j)*Kk(j)*Cr;
 	Q(j)=Kk(j)*Kk(j)*Cr;

	% Ϊ����˲�������Ӧ�ٶȣ���С�ͺ�����µ���ֵ 
% 	if Kk(j)>0.5
% 		act_value=data(j);
%     end
    result(j,1)=act_value;
end
result(:,2)=P;
result(:,3)=Q;

end

