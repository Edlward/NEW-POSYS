function [ result ] = corrcoef_self( var1,var2 )
result=mean((var1-mean(var1)).*(var2-mean(var2)));
result=result./std(var1)./std(var2);
end

