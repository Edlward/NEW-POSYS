function [ angle ] = angleInt( static,motion,mode )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
if mode==1
    angle=sum((motion(:,3)-mean(static(:,3)))*0.005);
end
if mode==2
static=Kalman(static)
end

end

