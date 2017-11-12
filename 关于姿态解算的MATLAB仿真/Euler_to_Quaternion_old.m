function [ quaternion ] = Euler_to_Quaternion_old( Rad )
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
  Rad=Rad/180*pi;

  quaternion(0+1)=0.5*sqrt(1+cos(Rad(2))*cos(Rad(3))+cos(Rad(1))*cos(Rad(3))+cos(Rad(1))*cos(Rad(2))+sin(Rad(1))*sin(Rad(2))*sin(Rad(3)));
  if quaternion(0+1)==0.0
    quaternion(0+1)=1.0;
    quaternion(1+1)=0.0;
    quaternion(2+1)=0.0;
    quaternion(3+1)=0.0;
  else
    quaternion(1+1)=(sin(Rad(1))+sin(Rad(2))*sin(Rad(3))+sin(Rad(1))*cos(Rad(2))*cos(Rad(3)))/4/quaternion(0+1);
    quaternion(2+1)=(sin(Rad(2))*cos(Rad(3))-cos(Rad(2))*sin(Rad(3))*sin(Rad(1))+sin(Rad(2))*cos(Rad(1)))/4/quaternion(0+1);
    quaternion(3+1)=(-cos(Rad(2))*sin(Rad(3))+sin(Rad(2))*cos(Rad(3))*sin(Rad(1))-sin(Rad(3))*cos(Rad(1)))/4/quaternion(0+1);
  end

end

