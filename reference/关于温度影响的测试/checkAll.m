%��ͬ�������Լ��飨�����˲���ı����Ϊ�˲����ݵ�׼ȷ�ԣ�
dataWild={data1,data2,data3,data4,data5};
%δ�˲�%chart1���ɳ�ʼ���������¶���Ʈ��cl
clear chart result
for order=1:4
    for dataPotv=1:5
        clear data 
        switch order
            case 1
                data=BuildTemChart(dataWild{dataPotv}(:,1),dataWild{dataPotv}(:,2),0.1);
            case 2
                data=BuildTemChart(dataWild{dataPotv}(:,1),dataWild{dataPotv}(:,4),0.1);
            case 3
                data=BuildTemChart(dataWild{dataPotv}(:,3),dataWild{dataPotv}(:,2),0.1);
            case 4
                data=BuildTemChart(dataWild{dataPotv}(:,3),dataWild{dataPotv}(:,4),0.1);
        end
        chart{dataPotv}{order}(:,1)=data(:,1);
        chart{dataPotv}{order}(:,2)=data(:,2);
    end
end

%result   �����������  �������ݵ�����  ���õĲ�ͬ����
for dataNatv=1:5
    for dataPotv=1:5
        for way=1:4
            if (dataNatv~=dataPotv)
                switch way
                    case 1
                        result(way,dataNatv,dataPotv)=checkChart(dataWild{dataNatv}(:,1),dataWild{dataNatv}(:,2),0.1,chart{dataPotv}{way});
                    case 2
                        result(way,dataNatv,dataPotv)=checkChart(dataWild{dataNatv}(:,1),dataWild{dataNatv}(:,4),0.1,chart{dataPotv}{way});
                    case 3
                        result(way,dataNatv,dataPotv)=checkChart(dataWild{dataNatv}(:,3),dataWild{dataNatv}(:,2),0.1,chart{dataPotv}{way});
                    case 4
                        result(way,dataNatv,dataPotv)=checkChart(dataWild{dataNatv}(:,3),dataWild{dataNatv}(:,4),0.1,chart{dataPotv}{way});
                end
            else
                result(way,dataNatv,dataPotv)=0;
            end
        end
    end
end
% 
% clear score temp
% score(5,4)=0;
% for dataNatv=1:5
%     i=0;
%     figure
%     for dataPotv=1:5
%         if (dataNatv~=dataPotv)
%         i=i+1;
%         subplot(2,2,i);plot(result(:,dataNatv,dataPotv));title(['dataPotv' num2str(dataPotv)]);grid on
%         temp=find(abs(result(:,dataNatv,dataPotv))==min(abs(result(:,dataNatv,dataPotv))));
%         score(dataNatv,temp)=score(dataNatv,temp)+1;
%         end
%     end
% end
% score
    for i=1:4
        subplot(2,2,i);plot(chart{i}{2}(:,1),chart{i}{2}(:,2));grid on
    end
%��������ֱ�Ϊ���ִ���ʽ�Ľ��
%�����У�����
%���зֱ���ԭʼ�����˲��ͷ��˲������
%���зֱ��������ֱ��������ĸ�����Ʈ����˺���Ʈֵ��ƽ����



