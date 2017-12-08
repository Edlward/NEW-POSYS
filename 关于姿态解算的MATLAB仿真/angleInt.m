function [ angle ,temporaryAngle] = angleInt( static,motion,mode )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
if mode==1
    data=motion-mean(static);
    data(find(abs(data)<0.01))=0;
    angle=sum(data*0.005);
    for i=1:length(data)
        temporaryAngle(i)=sum(data(1:i)*0.005);
    end
%     for i=1:length(temporaryAngle)
%         if temporaryAngle(i)>180
%             temporaryAngle(i)=temporaryAngle(i)-360;
%         end
%         if temporaryAngle(i)<-180
%             temporaryAngle(i)=temporaryAngle(i)+360;
%         end
%     end
%     figure
%     plot(-temporaryAngle);
end
if mode==2
    data=[static;motion];
    for i=1:3
        static(:,i)=kalmanq(static(:,i),var(static(:,i)),var(static(:,i)));
        data(:,i)=kalmanq(data(:,i),var(static(:,i)),var(static(:,i)));
        data(:,i)= data(:,i)-static(end,i);
    end
    motion=data(length(static(:,1))+1:end,:);
    angle=figureAngle(motion);
end
if mode==3
    data=[static(:,3);motion(:,3)];
    static(:,3)=Kalman(static(:,3),3);
    data=Kalman(data,3);
    data= data-static(end,3);
    angle=sum(data(length(static(:,1))+1:end))*0.005;
end
if mode==4
    data=[static(:,3);motion(:,3)];
    static(:,3)=kalmanEFK(static(:,3),50,3);
    data=kalmanEFK(data,50,3);
    data= data-static(end,3);
    angle=sum(data(length(static(:,1))+1:end))*0.005;
end
if mode==5
    motion=motion(:,3)-mean(static(:,3));
    motionshift=[0;motion(1:end-1)];
    motion=(motion+motionshift)/2;
    angle=sum(motion*0.005);
end
if mode==6
    for i=1:3
        motion(:,i)=motion(:,i)-mean(static(:,i));
    end
    motion(:,1:2)=zeros(size(motion(:,1:2)));
    angle=figureAngle(motion);
end
if mode==7
    for i=1:3
        motion(:,i)=motion(:,i)-mean(static(:,i));
    end
    [angle,temporaryAngle]=figureAngle(motion);
end

if mode==8
    data=motion(:,3)-mean(static(end-end:end,3));
    data(find(abs(data)<0.01))=0;
    angle=sum(data*0.005);
    for i=1:length(data)
     temporaryAngle(i)=sum(data(1:i)*0.005);
    end
%     for i=1:length(temporaryAngle)
%         if temporaryAngle(i)>180
%            temporaryAngle(i)=temporaryAngle(i)-360;
%         end
%         if temporaryAngle(i)<-180
%            temporaryAngle(i)=temporaryAngle(i)+360;
%         end
%     end
%     %figure
%     %plot(-temporaryAngle);

end
end

