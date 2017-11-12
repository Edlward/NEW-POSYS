function [ quaternion ] = QuaternionInt( quaternion , palstance,dT)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明

 % 角度弧度转换 */
  palstance=palstance/180.0*pi;

  %根据其阶数不同改变angle的值*/
  for grade=1:1:4
    temp_quarterion=quaternion;
    switch grade
    case 1
      dif_quarterion_1=calculateK(temp_quarterion,palstance,dT);
    case 2
      temp_quarterion=temp_quarterion+dif_quarterion_1/2.0;
      dif_quarterion_2=calculateK(temp_quarterion,palstance,dT);
    case 3
      temp_quarterion=temp_quarterion+dif_quarterion_2/2.0;
      dif_quarterion_3=calculateK(temp_quarterion,palstance,dT);
    case 4
      temp_quarterion=temp_quarterion+dif_quarterion_3;
      dif_quarterion_4=calculateK(temp_quarterion,palstance,dT);
    end
  end
  quaternion=quaternion+1/6*(dif_quarterion_1+2.0*dif_quarterion_2+2.0*dif_quarterion_3+dif_quarterion_4);
end
