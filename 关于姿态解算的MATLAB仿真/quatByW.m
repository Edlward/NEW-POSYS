quaternion=[1 0 0 0];
dT=0.005;
palstance=[[1 1 1];[2 2 2];[3 3 3]];
for i=1:3
    quaternion=QuaternionInt(quaternion,palstance(:,i),dT);
end
quaternion