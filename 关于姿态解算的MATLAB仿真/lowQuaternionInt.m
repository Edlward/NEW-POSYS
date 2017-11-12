function [ quaternion ] = lowQuaternionInt(  quaternion, data,dT )
	persistent old_w;
    if isempty(old_w)
        old_w=[0;0;0];
	
	dif_quarterion_f(0+1)=(-quaternion(1+1)*old_w(1) - quaternion(2+1)*old_w(2) - quaternion(3+1)*old_w(3))*0.5;
	dif_quarterion_f(1+1)=( quaternion(0+1)*old_w(1) + quaternion(2+1)*old_w(3) - quaternion(3+1)*old_w(2))*0.5;
	dif_quarterion_f(2+1)=( quaternion(0+1)*old_w(2) - quaternion(1+1)*old_w(3) + quaternion(3+1)*old_w(1))*0.5;
	dif_quarterion_f(3+1)=( quaternion(0+1)*old_w(3) + quaternion(1+1)*old_w(2) - quaternion(2+1)*old_w(1))*0.5;
	
	med_quarterion(0+1)=quaternion(0+1)+dif_quarterion_f(0+1)*dT;
	med_quarterion(1+1)=quaternion(1+1)+dif_quarterion_f(1+1)*dT;
	med_quarterion(2+1)=quaternion(2+1)+dif_quarterion_f(2+1)*dT;
	med_quarterion(3+1)=quaternion(3+1)+dif_quarterion_f(3+1)*dT; 
  
	dif_quarterion_l(0+1)=(-med_quarterion(1+1)*data(1) - med_quarterion(2+1)*data(2) - med_quarterion(3+1)*data(3))*0.5;
	dif_quarterion_l(1+1)=( med_quarterion(0+1)*data(1) + med_quarterion(2+1)*data(3) - med_quarterion(3+1)*data(2))*0.5;
	dif_quarterion_l(2+1)=( med_quarterion(0+1)*data(2) - med_quarterion(1+1)*data(3) + med_quarterion(3+1)*data(1))*0.5;
	dif_quarterion_l(3+1)=( med_quarterion(0+1)*data(3) + med_quarterion(1+1)*data(2) - med_quarterion(2+1)*data(1))*0.5;
	
	
	quaternion(0+1)=quaternion(0+1)+0.5*(dif_quarterion_f(0+1)+dif_quarterion_l(0+1))*dT;
	quaternion(1+1)=quaternion(1+1)+0.5*(dif_quarterion_f(1+1)+dif_quarterion_l(1+1))*dT;
	quaternion(2+1)=quaternion(2+1)+0.5*(dif_quarterion_f(2+1)+dif_quarterion_l(2+1))*dT;
	quaternion(3+1)=quaternion(3+1)+0.5*(dif_quarterion_f(3+1)+dif_quarterion_l(3+1))*dT;
	old_w=data;

end

