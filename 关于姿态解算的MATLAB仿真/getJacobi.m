function [ dif_quarterion ] = getJacobi(quaternion,data)
%UNTITLED3 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��  
  dif_quarterion(0+1)=(-quaternion(1+1)*data(0+1) - quaternion(2+1)*data(1+1) - quaternion(3+1)*data(2+1))*0.5;
  dif_quarterion(1+1)=( quaternion(0+1)*data(0+1) + quaternion(2+1)*data(2+1) - quaternion(3+1)*data(1+1))*0.5;
  dif_quarterion(2+1)=( quaternion(0+1)*data(1+1) - quaternion(1+1)*data(2+1) + quaternion(3+1)*data(0+1))*0.5;
  dif_quarterion(3+1)=( quaternion(0+1)*data(2+1) + quaternion(1+1)*data(1+1) - quaternion(2+1)*data(0+1))*0.5;
end
