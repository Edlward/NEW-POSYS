function [ resultAngle ] = angleInt2( data,mode)
%data组成为三个陀螺仪的三轴角速度，三个加速度计的三轴加速度
if mode==1
    for i=1:length(data)
       temporaryAngle(i)=sum(data(1:i)*0.005);
    end
end

end

