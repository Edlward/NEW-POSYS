function [ result ] = kalmanIst( data )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
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
	%�൱�����������
	for i=1:(N-1)
        IAE_st(i)=IAE_st(i+1);
    end
	IAE_st(N)=data(j)-predict;
	Cr=0;
	for i=1:N
		Cr=Cr+IAE_st(i)*IAE_st(i);
    end
	Cr=Cr/N;		%���������֪���Բ��ԣ���Ҫ��ԭ��ʽ��
	% Ԥ�Ȿ�ε�Ԥ����� 
 	P_pre(j)=P(j-1)+Q(j-1);
	% ����ϵ����������ֵ 
	Kk(j)=P_pre(j)/(P_pre(j)+R);
	act_value=predict+Kk(j)*(data(j)-predict);
	
	% ����Ԥ����� 
	P(j)=(1-Kk(j))*P_pre(j);
	
	% ���㲢����ϵͳ���� 
  	Q(j)=Kk(j)*Kk(j)*Cr;%+P(j)+P(j-1);

	% Ϊ����˲�������Ӧ�ٶȣ���С�ͺ�����µ���ֵ 
% 	if Kk(j)>0.5
% 		act_value=data(j);
%     end
    result(j,1)=act_value;
end
% result(:,2)=Kk;
% result(:,3)=QQ;
% result(:,4)=P_pre;

end

