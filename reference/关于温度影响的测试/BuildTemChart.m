    function [ result ] = BuildTemChart( data,tem,step )
    %���ݣ��¶ȣ��������¶ȵĲ�����0.1,0.2�ȵȣ�
    if(length(data)~=length(tem))
    'error'
    end
    mdata(:,2)=data;
    mdata(:,1)=tem;
    mdata=sortrows(mdata,1);
    if(step~=0)
    minTem=ceil(mdata(1,1)*(1/step));
    maxTem=floor(mdata(length(mdata),1)*(1/step));
    mdata(:,1)=round(mdata(:,1)*(1/step))/(1/step);
    end
    %��ͬ�¶ȵ�ȡƽ������Ȼ��ɾ��������
    index=1;
    while(1)
    x=find(mdata==mdata(index,1));
    mdata(index,2)=mean(mdata(x,2));
    mdata(index+1:index+length(x)-1,:)=[];
    index=index+1;
        if(index>length(mdata))
        break;
        end
    end
    result=mdata;
    end

