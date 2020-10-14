%% initialize
clc; close all; clear all;
format shortg; format compact;

%% 函数
syms x
syms a b c d

y=inline('a*x.^3+b*x.^2+c*x+d'); %内联函数
f(x)=a*x.^3+b*x.^2+c*x+d;  % inline function:

% @函数 function_handle

fun=@(x) a*x^2+b*x+c;
fun(1)
%% 数据处理
% 模除法和舍入
% mod	除后的余数（取模运算）
% rem	除后的余数
% idivide	带有舍入选项的整除
% ceil	朝正无穷大四舍五入
% fix	朝零四舍五入
% floor	朝负无穷大四舍五入
% round	四舍五入为最近的小数或整数



%% 精度控制函数
% matlab控制运算精度函数digits(A)和vpa(B)
digits(6);
sqrt(2)
a=vpa(sqrt(2)) %联合控制

a=int8(pi)
a=single(pi)
% 查看数据类型函数是class
class(a)
class(pi)


%% 随机函数

a(1:4)=[0,1,2,3];
a(5:8)=a(1:4);
a(9:16)=a(1:8);
Data = a;
Data1 = [];

digits(10)
Data = 100*(2*rand(unidrnd(100),1)-1);
for i = 1:size(Data,1)
    for j = 1:size(Data,2)
        if vpa(Data(i,j) > 0)
            %Data(i,j) = [];
            Data1 = [Data1,Data(i,j)];
        else
            continue;
        end
    end
end
% disp(Data1)

%% 因式分解
% 变量符号操作
% expand();% 表达式展开（乘=>加）
% factor()% 因式分解（加=>乘）
% simplify()

% Z=1+2*i;
% X=conj(Z),real(Z),imag(Z)
% format long;format short;format rat;%永久内部运算精度设置
% vpa(pi,4);
% digits(5);vpa(pi)%只识别总位数
% sprintf('%3.3f',pi) 
% fprintf('%3.3f\n',pi)
% sqrt(2),2^(0.5)
% 
% syms x a t h n;% 微积分
% D1=limit(sin(x)/x,x,0)%f(x):x=>0%<=>limit(sin(x)/x)
% D2=diff(exp(x^2),'x',1)%df(x)/dx%diff(exp(x^2))%differential
% D3=int(2*x*exp(x^2),x,0,1)%∫f(x)dx|a->b%integral
% D4=fourier()%积分变换
% funtool
% pretty(collect(eqlift-eqright,th8))

