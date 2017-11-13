function [ result ] = kalman( data)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
R=0.003;
Q=0.003;
P_last=0.01;
for j=1:50
    IAE_st(j)=data(j)-mean(data(1:50));
    result(j)=0;
end
act_value=mean(data(1:50));
for j=50:length(data)
	predict=act_value;
	%�൱�����������
	for i=1:49
        IAE_st(i)=IAE_st(i+1);
    end
	IAE_st(50)=data(j)-predict;
	
	% ��Ϣ�ķ������ 
	Cr=sum(IAE_st(1:50).^2)/50;		%���������֪���Բ��ԣ���Ҫ��ԭ��ʽ��
	
	% Ԥ�Ȿ�ε�Ԥ����� 
	P_mid=P_last+Q;
	
	% ����ϵ����������ֵ 
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(data(j)-predict);
	
	% ����Ԥ����� 
	P_last=(1-Kk)*P_mid;
	
	% ���㲢����ϵͳ���� 
 	Q=Kk*Kk*Cr;

	if Kk>0.5
		act_value=data(j);
    end
    result(j)=act_value;
end
end

