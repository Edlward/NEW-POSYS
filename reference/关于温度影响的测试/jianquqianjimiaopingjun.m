function [ output_args ] = jianquqianjimiaopingjun( data,jieshu,qianjimiao )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
% (mean(data(jieshu*200:end))-mean(data(jieshu*200-qianjimiao*200:jieshu*200)))
 sum((mean(data(jieshu*200:end))-mean(data(jieshu*200-qianjimiao*200:jieshu*200)))*0.005*length(data(jieshu*200:end)))
% sum((mean(data(jieshu*200:end)))*0.005*length(data(jieshu*200:end)))
end

