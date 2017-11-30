function [ angle ] = angleInt( static,motion,mode )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
if mode==1
    angle=sum((motion(:,3)-mean(static(:,3)))*0.005);
end
if mode==2
static=Kalman(static)
end

end

