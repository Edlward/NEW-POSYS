function [ result ] = figurePos( zangle,vell1,vell2)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
%这种方法精度还是差了点
%      a=0.3533/2/180*pi;
%     real=[ cos(a)/(2*cos(a)^2 - 1), sin(a)/(2*sin(a)^2 - 1);sin(a)/(2*sin(a)^2 - 1), cos(a)/(2*cos(a)^2 - 1)]*[vell1';vell2'];
%     real=real';
%     s=sqrt(vell1.^2+vell2.^2);
%     tanangle=atan2(real(:,1),real(:,2))/pi*180;
%     tanangle=tanangle+zangle;
%     sx=s.*cosd(tanangle);
%     sy=s.*sind(tanangle);
% 	for i=1:length(tanangle(:,1))
%         posx(i)=sum(sx(1:i));
%         posy(i)=sum(sy(1:i));
%     end
%     convert_X=posx*0.707106781186548+posy*0.707106781186548;
% 	convert_Y=posx*0.707106781186548-posy*0.707106781186548;
% 	
% 	posx=convert_X*0.038622517085838;
% 	posy=convert_Y*0.038651337725656;

% 
    a=0.3533/2/180*pi;
    real=[ cos(a)/(2*cos(a)^2 - 1), sin(a)/(2*sin(a)^2 - 1);sin(a)/(2*sin(a)^2 - 1), cos(a)/(2*cos(a)^2 - 1)]*[vell1';vell2'];
    vell1=real(1,:)';
    vell2=real(2,:)';
	add(:,1)=sin(zangle*0.017453292519943).*vell2+cos(zangle*0.017453292519943).*vell1;
	add(:,2)=cos(zangle*0.017453292519943).*vell2-sin(zangle*0.017453292519943).*vell1;

	for i=1:length(add(:,1))
        vell(i,1)=sum(add(1:i,1));
        vell(i,2)=sum(add(1:i,2));
    end
	
	convert_X=vell(:,1)*0.707106781186548+vell(:,2)*0.707106781186548;
	convert_Y=vell(:,1)*0.707106781186548-vell(:,2)*0.707106781186548;
	
	posx=convert_X*0.038622517085838;
	posy=convert_Y*0.038651337725656;
     %plot(posx,posy);
    result=sqrt(posx(end)^2+posy(end)^2);
end



