function [ result ] = posefigure( data)
%   gyroz1 gyroz2 gyroz3 轮子1 轮子2
%   此处显示详细说明
minvalue=0.01;
static=data(1:20860,3);
%概率论去除杂点
static(find(abs(static-mean(static))>3*sqrt(var(static))))=[];
motion=data(20861:end,:);
motion(:,3)=motion(:,3)-mean(static(end-8/0.002:end));
motion(find(abs(motion(:,3))<minvalue),3)=0;
index=0;
n=2;
for i=n:n:length(motion(:,3))
   index=index+1;
   if index==1
       temporaryAngle(1)=sum(motion(1:n,3)*0.002);
       angle(1)=temporaryAngle(1);
       vell1(1)=sum(motion(1:n,1));
       vell2(1)=sum(motion(1:n,2));
   else
       temporaryAngle(index)=temporaryAngle(index-1)+sum(motion(n*(index-1)+1:n*index,3)*0.002);
       angle(index)=(temporaryAngle(index)+temporaryAngle(index-1))/2;
       vell1(index)=sum(motion((index-1)*n+1:n*index,1));
       vell2(index)=sum(motion((index-1)*n+1:n*index,2));
   end
end
angle=angle';
result(1,1)=angle(end,end);
result(1,2)=figurePos(angle,vell1',vell2');
%angle=angle(end);
% posx=posx(end);
% posy=posy(end);
end

