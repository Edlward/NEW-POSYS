function [ result ] = kalman( data ,Q,R)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
P_last=0.01;
act_value=0;
for j=1:length(data)
	predict=act_value;

	% Ԥ�Ȿ�ε�Ԥ����� 
	P_mid=P_last+Q;
	
	% ����ϵ����������ֵ 
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(data(j)-predict);
	
	% ����Ԥ����� 
	P_last=(1-Kk)*P_mid;

    result(j)=act_value;
end

end

