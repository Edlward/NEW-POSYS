function [ dif_quarterion ] = calculateK(quaternion,data,dT)
  jacobi=getJacobi(quaternion,data);
  dif_quarterion(0+1)=jacobi(0+1)*dT;
  dif_quarterion(1+1)=jacobi(1+1)*dT;
  dif_quarterion(2+1)=jacobi(2+1)*dT;
  dif_quarterion(3+1)=jacobi(3+1)*dT; 
end

