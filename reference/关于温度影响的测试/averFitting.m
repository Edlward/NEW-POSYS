function [ result ] = averFitting( data6 )
%UNTITLED3 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

result=sum((data6(1:3000,1)-mean(data6(1:3000,1))).*(data6(1:3000,4)-mean(data6(1:3000,4))))/sum((data6(1:3000,1)-mean(data6(1:3000,1))).^2);
end

