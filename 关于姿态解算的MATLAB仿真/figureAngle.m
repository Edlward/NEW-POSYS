function [ angle ] = figureAngle( data,dT )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��quaternion=[1 0 0 0];
wx=data(:,1);
wy=data(:,2);
wz=data(:,3);
palstance=[wx';wy';wz'];
quaternion=[1 0 0 0];
for i=1:length(wx)
    quaternion=QuaternionInt(quaternion,palstance(:,i),dT);
end
angle=Quaternion_to_Euler(quaternion)
end

