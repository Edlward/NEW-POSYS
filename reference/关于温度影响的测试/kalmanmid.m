function [ result ] = kalmanmid( data,Q,R )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
P(49)=0.001;
QQ(length(data))=Q;
for j=1:50
    IAE_st(j)=data(j);
    result(j,1)=data(j);
end
act_value=data(49);
m=2;
% QQ(49)=0.000000008236918;
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
    if(j<5000)
        P_pre(j)=(P(j-1)^m+QQ(j-1)^m)^(1/m);
    else
        P_pre(j)=P(j-1)+QQ(j-1);
    end
	% ����ϵ����������ֵ 
    if(j<5000)
        Kk(j)=(P_pre(j)^m/(P_pre(j)^m+R^m))^(1/m);
    else
        Kk(j)=P_pre(j)/(P_pre(j)+R);
    end
	act_value=predict+Kk(j)*(data(j)-predict);
	
	% ����Ԥ����� 
    if(j<5000)
        P(j)=((1-Kk(j))*(P_pre(j)^m))^(1/m);
    else
        P(j)=(1-Kk(j))*P_pre(j);
    end
	
	% ���㲢����ϵͳ���� 
	QQ(j)=Kk(j)*Kk(j)*Cr;

	% Ϊ����˲�������Ӧ�ٶȣ���С�ͺ�����µ���ֵ 
% 	if Kk(j)>0.5
% 		act_value=data(j);
%     end
    result(j,1)=act_value;
end
result(:,2)=Kk;
result(:,3)=QQ;
result(:,4)=P_pre;

end

