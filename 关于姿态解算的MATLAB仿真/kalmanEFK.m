function [ result ] = kalmanEFK( data,n,axis)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
if axis==1
R=0.001663456966435;
Q=0.001663456966435;
end
if axis==2
R=0.001184112746198;
Q=0.001184112746198;
end
if axis==3
R=0.000770426491094;
Q=0.000770426491094;
end
P_last=0.01;
t=[1:1:n-1]';
y=polyfit(t,data(1:n-1),1);%1x2��������
for j=1:n
    IAE_st(j)=data(j)-y(1)*j-y(2);
    result(j)=mean(data(1:n));
end
for j=n:length(data)
    y=polyfit(t,data(j-(n-1):j-1),1);%1x2��������
	predict=y(1)*n+y(2);
	%�൱�����������
	for i=1:n-1
        IAE_st(i)=IAE_st(i+1);
    end
	IAE_st(n)=data(j)-predict;
	
	% ��Ϣ�ķ������ 
	Cr=sum(IAE_st(1:n).^2)/n;		%���������֪���Բ��ԣ���Ҫ��ԭ��ʽ��
	
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

