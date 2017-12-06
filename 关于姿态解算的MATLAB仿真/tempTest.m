i=1;
for Q=logspace(0.00000000001,0.1,100)
    for R=logspace(0.00000000001,0.1,100)
        angle(i)=testQR(slowstatic,slowmotion,Q,R);
        i=i+1;
    end
end
plot(angle)