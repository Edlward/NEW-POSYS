function [ result ] = checkChart( data,temp,step,chart )
%UNTITLED 其本质就是衡量建表的一致性   
wildData=BuildTemChart(data,temp,step);
if(min(wildData(:,1))<min(chart(:,1)))
minIndexChart=1;
minIndexWild=find(wildData==min(chart(:,1)));
else
minIndexWild=1;
minIndexChart=find(chart==min(wildData(:,1)));
end

if(max(wildData(:,1))<max(chart(:,1)))
maxIndexWild=length(wildData(:,1));
maxIndexChart=find(chart==max(wildData(:,1)));
else
maxIndexChart=length(chart(:,1));
maxIndexWild=find(wildData==max(chart(:,1)));
end

result1=wildData(minIndexWild:maxIndexWild,2);
result2=chart(minIndexChart:maxIndexChart,2);

result=mean(result1)-mean(result2);

end

