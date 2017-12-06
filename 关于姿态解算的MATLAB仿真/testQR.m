function [ angle ] = testQR( static,motion,Q,R )
    data=[static(:,3);motion(:,3)];
    static(:,3)=kalman(static(:,3),Q,R);
    data=kalman(data,Q,R);
    data= data-static(end,3);
    angle=sum(data(length(static(:,1))+1:end))*0.005;
end

