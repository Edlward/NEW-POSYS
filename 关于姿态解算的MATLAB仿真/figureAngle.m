function [ angle ] = figureAngle( data )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��quaternion=[1 0 0 0];
dT=0.005;
wx=data(:,1)/pi*180;
wy=data(:,2)/pi*180;
wz=data(:,3)/pi*180;
palstance=[wx';wy';wz'];
quaternion=[1 0 0 0];
for i=1:length(wx)
    quaternion=QuaternionInt(quaternion,palstance(:,i),dT);
end
angle=Quaternion_to_Euler(quaternion);
end

