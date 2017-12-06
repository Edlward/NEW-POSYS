b1=[2,2];b2=[3,5];b3=[5,6];b4=[7,3];%四个型值点的横纵坐标值 可以自定 
plot([b1(1),b2(1),b3(1),b4(1)],[b1(2),b2(2),b3(2),b4(2)],'o');%画出型值点 
hold on; 
N=[5 1 0 0;1 4 1 0;0 1 4 1;0 0 1 5]; 
P=N\(6*([b1;b2;b3;b4]));%P 是二维数组 存放控制点 求出 P 
xc1=P(1); %对应 6 个控制点 
xc2=P(2); 
xc3=P(3); 
xc4=P(4); 
yc1=P(5); 
yc2=P(6); 
yc3=P(7); 
yc4=P(8); 
plot([xc1,xc2,xc3,xc4],[yc1,yc2,yc3,yc4],'*')%画出控制点 
hold on; 
plot([xc1,xc2,xc3,xc4],[yc1,yc2,yc3,yc4])%连接控制点 
hold on; 
u=0:0.01:1; 
N1=1/6*(1-3*u+3*u.*u-u.*u.*u);%基函数 
N2=1/6*(4-6*u.*u+3*u.*u.*u); 
N3=1/6*(1+3*u+3*u.*u-3*u.*u.*u); 
N4=1/6*u.*u.*u; 
xp=xc1.*N1+xc1.*N2+xc2.*N3+xc3.*N4; 
yp=yc1.*N1+yc1.*N2+yc2.*N3+yc3.*N4; plot(xp,yp);%反算第一段 B 样条 
hold on; 
xp=xc1.*N1+xc2.*N2+xc3.*N3+xc4.*N4; 
yp=yc1.*N1+yc2.*N2+yc3.*N3+yc4.*N4; 
plot(xp,yp);%反算第二段 B 样条 
hold on; 
xp=xc2.*N1+xc3.*N2+xc4.*N3+xc4.*N4; 
yp=yc2.*N1+yc3.*N2+yc4.*N3+yc4.*N4; 
plot(xp,yp);%反算第三段 B 样条 
hold on;