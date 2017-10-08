function [ result ] = averFitting( data6 )
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明

result=sum((data6(1:3000,1)-mean(data6(1:3000,1))).*(data6(1:3000,4)-mean(data6(1:3000,4))))/sum((data6(1:3000,1)-mean(data6(1:3000,1))).^2);
end

