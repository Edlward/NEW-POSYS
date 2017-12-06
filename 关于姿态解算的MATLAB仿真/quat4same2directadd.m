[angle,temporaryAngle1]=angleInt(slowstatic,slowmotion,7);
plot(temporaryAngle1(:,3));
[angle,temporaryAngle]=angleInt(slowstatic,slowmotion,1);
for i=1:length(temporaryAngle)
if temporaryAngle(i)>180
temporaryAngle(i)=temporaryAngle(i)-360;
end
if temporaryAngle(i)<-180
temporaryAngle(i)=temporaryAngle(i)+360;
end
end
hold on
plot(-temporaryAngle,'r');title('红的是直接累加得到的')