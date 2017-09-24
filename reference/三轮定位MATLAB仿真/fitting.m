period=0.0001;
time=180;
d=231.45;
radius=50;
pos(1,1)=0;
pos(2,1)=0;
pos(3,1)=0;
result(1,1)=0;
result(2,1)=0;
result(3,1)=0;
f=@(xita)[-2/3*sin(xita),	-sqrt(3)/3*cos(xita)+1/3*sin(xita),		sqrt(3)/3*cos(xita)+1/3*sin(xita);
 2/3*cos(xita),		-sqrt(3)/3*sin(xita)-1/3*cos(xita),		sqrt(3)/3*sin(xita)-1/3*cos(xita);
 1/3/d,				1/3/d,									1/3/d							 ];
for i=1:time/period
sensorData(1,1)=i*period;
sensorData(2,1)=i*period*i*period*5;
sensorData(3,1)=i*period*i*period*3;
jacobi=f(pos(3,1))*sensorData;
jacobi=jacobi*period;
pos=pos+jacobi;
result(:,i)=pos;
end
result=result';