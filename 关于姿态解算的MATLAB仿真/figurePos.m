function [ posx posy ] = figurePos( zangle,vell1,vell2 )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
%     lastangle=[0;zangle(1:end-1)];
%     zangle=(zangle+lastangle)/2;
	add(:,1)=sin(-zangle*0.017453292519943).*vell2+cos(-zangle*0.017453292519943).*vell1;
	add(:,2)=cos(-zangle*0.017453292519943).*vell2+sin(-zangle*0.017453292519943).*vell1;

	for i=1:length(add(:,1))
        vell(i,1)=sum(add(1:i,1));
        vell(i,2)=sum(add(1:i,2));
    end
	
	vell=vell*1.000004752846005;
	
	convert_X=vell(:,1)*0.707106781186548+vell(:,2)*0.707106781186548;
	convert_Y=vell(:,1)*0.707106781186548-vell(:,2)*0.707106781186548;
	
	posx=convert_X*0.038622517085838;
	posy=convert_Y*0.038651337725656;
end

