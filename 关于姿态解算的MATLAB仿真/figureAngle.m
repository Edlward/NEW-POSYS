function [ angle ,temporaryAngle] = figureAngle( data )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明quaternion=[1 0 0 0];
dT=0.005;
% wx=data(:,1)/pi*180;
% wy=data(:,2)/pi*180;
% wz=data(:,3)/pi*180;
wx=data(:,1);
wy=data(:,2);
wz=data(:,3);
palstance=[wx';wy';wz'];
quaternion=[1 0 0 0];
for i=1:length(wx)
    quaternion=QuaternionInt(quaternion,palstance(:,i),dT);
    temporaryAngle(i,1:3)=Quaternion_to_Euler(quaternion);
    quaternion=Euler_to_Quaternion_old(temporaryAngle(i,1:3));
end
angle=Quaternion_to_Euler(quaternion);
end

