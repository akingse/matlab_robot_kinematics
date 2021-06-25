%% initialize
clc; close all; clear all;
format shortg; format compact;

%% matlab导入数据
% filename = 'ming.png';
% A = importdata(filename);
% image(A);
delimiterIn   = ' '; % 字符分隔符
headerlinesIn = 2;   % 文件头的行数，去掉文件头，保留后面的数据
filename='C:\Users\Akingse\Documents\MATLAB\code\date2.csv';
A = importdata(filename, delimiterIn, headerlinesIn); %文件导入函数
% A
% 提取数据
data       = A.data;
textdata   = A.textdata;
colheaders = A.colheaders;


% 函数	说明
% A = importdata(filename)	将数据从文件名所表示的文件中加载到数组 A 中
% A = importdata(’-pastespecial’)	从系统剪贴板加载数据，而不是从文件加载数据。
% A = importdata(___, delimiterIn)	将 delimiterIn 解释为 ASCII 文件、文件名或剪贴板数据中的列分隔符。可以将 delimiterIn 与上述语法中的任何输入参数一起使用。
% A = importdata(___, delimiterIn, headerlinesIn)	从 ASCII 文件、文件名或剪贴板加载数据，并从 lineheaderlinesIn+1 开始读取数字数据。
% A, delimiterOut, headerlinesOut] = importdata(___)	在分隔符输出中返回检测到的分隔符字符，并使用前面语法中的任何输入参数检测headerlinesOut 中检测到的标题行数。

%% 四舍五入取整，取同一值的对应最大值

filename='C:\Users\Akingse\Documents\MATLAB\code\date2.csv';
% filename='1.csv'; %放在路径根目录下
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
    
%% 筛选
A=2*rand(3,3)-1
B=Filter_N(A)


function T = Filter_N(T) %过滤函数,多维合成一维
    k=1;
    [line,column]=size(T);
    for i=1:line
        for j=1:column
            if T(i,j)>=0 %自定义筛选规则
                T1(k)=T(i,j);
                k=k+1;
            end
        end
    end
    T=T1;
end

% 多项式求值，多项式曲线拟合；
% for k=1:n2
%     z(:,k)=polyval(polyfit(X,A(:,k),1),X);           %每列经过最小二乘法直线回归拟合高程矩阵
%     Z(:,k)=A(:,k)-polyval(polyfit(X,A(:,k),1),X);    %经过倾斜度和偏置误差处理的每列高程
% end







