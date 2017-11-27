function [ result ] = square2_self( x,y,n )
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
y2=polyfit(x,y,n)%尝试不同的阶数拟合
% syms x f(x)
% f(x)=poly2sym(y2,x)%得到拟合出的表达式

end

