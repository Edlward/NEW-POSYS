function [ resultAngle ] = angleInt2( data,mode)
%data���Ϊ���������ǵ�������ٶȣ��������ٶȼƵ�������ٶ�
if mode==1
    for i=1:length(data)
       temporaryAngle(i)=sum(data(1:i)*0.005);
    end
end

end

