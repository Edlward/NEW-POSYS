function [ result ] = sortByIndex( data,index )
    data(:,1)=round(data(:,1)*10)/10;
    data=sortrows(data,1);
    n=length(index);
   for j=1:n
     data(find(data(:,1)==index(j)),:)=[];
   end
    result=data;
end

