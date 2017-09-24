%不同方法各自检验（不用滤波后的表检验为滤波数据的准确性）
dataWild={data1,data2,data3,data4,data5};
%未滤波%chart1是由初始数据五组温度零飘表cl
clear chart result
for order=1:4
    for dataPotv=1:5
        clear data 
        switch order
            case 1
                data=BuildTemChart(dataWild{dataPotv}(:,1),dataWild{dataPotv}(:,2),0.01);
            case 2
                data=BuildTemChart(dataWild{dataPotv}(:,1),dataWild{dataPotv}(:,4),0.01);
            case 3
                data=BuildTemChart(dataWild{dataPotv}(:,3),dataWild{dataPotv}(:,2),0.01);
            case 4
                data=BuildTemChart(dataWild{dataPotv}(:,3),dataWild{dataPotv}(:,4),0.01);
        end
        chart{dataPotv}{order}(:,1)=data(:,1);
        chart{dataPotv}{order}(:,2)=data(:,2);
    end
end

%result   被处理的数据  处理数据的数据  其用的不同方法
for dataNatv=1:5
    for dataPotv=1:5
        for way=1:4
            if (dataNatv~=dataPotv)
                switch way
                    case 1
                        result(way,dataNatv,dataPotv)=checkChart(dataWild{dataNatv}(:,1),dataWild{dataNatv}(:,2),0.01,chart{dataPotv}{way});
                    case 2
                        result(way,dataNatv,dataPotv)=checkChart(dataWild{dataNatv}(:,1),dataWild{dataNatv}(:,4),0.01,chart{dataPotv}{way});
                    case 3
                        result(way,dataNatv,dataPotv)=checkChart(dataWild{dataNatv}(:,3),dataWild{dataNatv}(:,2),0.01,chart{dataPotv}{way});
                    case 4
                        result(way,dataNatv,dataPotv)=checkChart(dataWild{dataNatv}(:,3),dataWild{dataNatv}(:,4),0.01,chart{dataPotv}{way});
                end
            else
                result(way,dataNatv,dataPotv)=0;
            end
        end
    end
end

clear score temp
score(5,4)=0;
for dataNatv=1:5
    i=0;
    figure
    for dataPotv=1:5
        if (dataNatv~=dataPotv)
        i=i+1;
        subplot(2,2,i);plot(result(:,dataNatv,dataPotv));title(['dataPotv' num2str(dataPotv)]);grid on
        temp=find(abs(result(:,dataNatv,dataPotv))==min(abs(result(:,dataNatv,dataPotv))));
        score(dataNatv,temp)=score(dataNatv,temp)+1;
        end
    end
end
score

%这两个表分别为两种处理方式的结果
%共四行，五列
%四行分别是原始数据滤波和非滤波的组合
%五列分别是五个表分别用其他四个表温飘表过滤后零飘值的平均数
