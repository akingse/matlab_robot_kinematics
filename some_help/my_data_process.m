%% initialize
clc; close all; clear all;
format shortg; format compact;

%% matlab��������
% filename = 'ming.png';
% A = importdata(filename);
% image(A);
delimiterIn   = ' '; % �ַ��ָ���
headerlinesIn = 2;   % �ļ�ͷ��������ȥ���ļ�ͷ���������������
filename='C:\Users\Akingse\Documents\MATLAB\code\date2.csv';
A = importdata(filename, delimiterIn, headerlinesIn); %�ļ����뺯��
% A
% ��ȡ����
data       = A.data;
textdata   = A.textdata;
colheaders = A.colheaders;


% ����	˵��
% A = importdata(filename)	�����ݴ��ļ�������ʾ���ļ��м��ص����� A ��
% A = importdata(��-pastespecial��)	��ϵͳ������������ݣ������Ǵ��ļ��������ݡ�
% A = importdata(___, delimiterIn)	�� delimiterIn ����Ϊ ASCII �ļ����ļ���������������е��зָ��������Խ� delimiterIn �������﷨�е��κ��������һ��ʹ�á�
% A = importdata(___, delimiterIn, headerlinesIn)	�� ASCII �ļ����ļ����������������ݣ����� lineheaderlinesIn+1 ��ʼ��ȡ�������ݡ�
% A, delimiterOut, headerlinesOut] = importdata(___)	�ڷָ�������з��ؼ�⵽�ķָ����ַ�����ʹ��ǰ���﷨�е��κ�����������headerlinesOut �м�⵽�ı���������

%% ��������ȡ����ȡͬһֵ�Ķ�Ӧ���ֵ

filename='C:\Users\Akingse\Documents\MATLAB\code\date2.csv';
% filename='1.csv'; %����·����Ŀ¼��
a=csvread(filename);
len=length(a); %


a(1,1)=round(a(1,1));
b(1,1:2)=a(1,1:2);
j=2;
for i=2:len %
    a(i,1)=round(a(i,1));
    if a(i,1)~=a(i-1,1)
        b(j,1:2)=a(i,1:2);
        j=j+1;
    end
    if a(i,2)>b(j-1,2)
        b(j-1,2)=a(i,2);
    end
end
b
% csvwrite('1-1.csv',b);
    
%% ɸѡ
A=2*rand(3,3)-1
B=Filter_N(A)


function T = Filter_N(T) %���˺���,��ά�ϳ�һά
    k=1;
    [line,column]=size(T);
    for i=1:line
        for j=1:column
            if T(i,j)>=0 %�Զ���ɸѡ����
                T1(k)=T(i,j);
                k=k+1;
            end
        end
    end
    T=T1;
end

% ����ʽ��ֵ������ʽ������ϣ�
% for k=1:n2
%     z(:,k)=polyval(polyfit(X,A(:,k),1),X);           %ÿ�о�����С���˷�ֱ�߻ع���ϸ߳̾���
%     Z(:,k)=A(:,k)-polyval(polyfit(X,A(:,k),1),X);    %������б�Ⱥ�ƫ�������ÿ�и߳�
% end







