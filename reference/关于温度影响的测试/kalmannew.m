function [ result ] = kalmannew( data)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
P(49)=0.00;
Q=0.006708655;
R=0.006708655;
QQ(length(data))=Q;
for j=1:50
    IAE_st(j)=data(j);
    result(j,1)=data(j);
end
act_value=data(49);
m=2;
QQ(49)=0.006708655;
% P_last=0.000006478590639000001;
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
        P_pre(j)=(P(j-1)^m+QQ(j-1)^m)^(1/m);
	% ����ϵ����������ֵ 
        Kk(j)=(P_pre(j)^m/(P_pre(j)^m+R^m))^(1/m);
	act_value=predict+Kk(j)*(data(j)-predict);
	
	% ����Ԥ����� 
        P(j)=((1-Kk(j))*(P_pre(j)^m))^(1/m);
	
	% ���㲢����ϵͳ���� 
	QQ(j)=Kk(j)*Kk(j)*Cr;

	% Ϊ����˲�������Ӧ�ٶȣ���С�ͺ�����µ���ֵ 
% 	if Kk(j)>0.5
% 		act_value=data(j);
%     end
    result(j,1)=act_value;
end
end

