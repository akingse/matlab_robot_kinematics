%% initialize
clc; close all; clear all;
format shortg; format compact;

%% ����
syms x
syms a b c d

y=inline('a*x.^3+b*x.^2+c*x+d'); %��������
f(x)=a*x.^3+b*x.^2+c*x+d;  % inline function:

% @���� function_handle

fun=@(x) a*x^2+b*x+c;
fun(1)
%% ���ݴ���
% ģ����������
% mod	�����������ȡģ���㣩
% rem	���������
% idivide	��������ѡ�������
% ceil	�����������������
% fix	������������
% floor	�����������������
% round	��������Ϊ�����С��������



%% ���ȿ��ƺ���
% matlab�������㾫�Ⱥ���digits(A)��vpa(B)
digits(6);
sqrt(2)
a=vpa(sqrt(2)) %���Ͽ���

a=int8(pi)
a=single(pi)
% �鿴�������ͺ�����class
class(a)
class(pi)


%% �������

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

%% ��ʽ�ֽ�
% �������Ų���
% expand();% ���ʽչ������=>�ӣ�
% factor()% ��ʽ�ֽ⣨��=>�ˣ�
% simplify()

% Z=1+2*i;
% X=conj(Z),real(Z),imag(Z)
% format long;format short;format rat;%�����ڲ����㾫������
% vpa(pi,4);
% digits(5);vpa(pi)%ֻʶ����λ��
% sprintf('%3.3f',pi) 
% fprintf('%3.3f\n',pi)
% sqrt(2),2^(0.5)
% 
% syms x a t h n;% ΢����
% D1=limit(sin(x)/x,x,0)%f(x):x=>0%<=>limit(sin(x)/x)
% D2=diff(exp(x^2),'x',1)%df(x)/dx%diff(exp(x^2))%differential
% D3=int(2*x*exp(x^2),x,0,1)%��f(x)dx|a->b%integral
% D4=fourier()%���ֱ任
% funtool
% pretty(collect(eqlift-eqright,th8))

