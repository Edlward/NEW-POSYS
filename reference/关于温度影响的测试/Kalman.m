function [ result ] = Kalman( data,Q,R)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
P_last=0;
IAE_st(50)=0;
QQ(49)=Q;
QQ(length(data))=Q;
for j=1:49
    IAE_st(j)=data(j);
    result(j,1)=data(j);
end
act_value=data(49);
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
	Cr=Cr/50.0;		%���������֪���Բ��ԣ���Ҫ��ԭ��ʽ��
	
	% Ԥ�Ȿ�ε�Ԥ����� 
	P_mid(j)=P_last+QQ(j-1);
	
	% ����ϵ����������ֵ 
	Kk(j)=P_mid(j)/(P_mid(j)+R);
	act_value=predict+Kk(j)*(data(j)-predict);
	
	% ����Ԥ����� 
	P_last=(1-Kk(j))*P_mid(j);
	
	% ���㲢����ϵͳ���� 
	QQ(j)=Kk(j)*Kk(j)*Cr;

	% Ϊ����˲�������Ӧ�ٶȣ���С�ͺ�����µ���ֵ 
	if Kk(j)>0.5
		act_value=data(j);
    end
    result(j,1)=act_value;
end
result(:,2)=Kk;
result(:,3)=QQ;
end

