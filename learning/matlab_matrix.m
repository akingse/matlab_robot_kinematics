%% initialize
clc; close all; clear all;
format shortg; format compact;
% ��ѧ����������ѧ����Ҫ��ʱ����

% scalar | vector | matrix | multi_array
% ����   |  ʸ��  |  ����  |  ��ά����

%% ����operate
% zeros(3),%0 %����
% ones(3,4),1 һ��E 
% eye(3) ,��λ��

% x1=inv(x);%��
% û�а��棬bansuiX=inv(x)*det(x);
% x2=rank(x);%��
% x3=(x)';%ת��
% x4=trace(x);��
% det(x);%����ʽ��ֵ
% poly()%��������ʽ
% null(A,'r')%��ζ���ʽ�������ϵ
% diag()%�Խ���
% x5=eig(x0);%����ֵ
% [v,D]=eig(x0);%��������������ֵ�Խ���

%% ������
% zeros(3),ones(3,4),eye(3)%0 1 E %����һ�󣬵�λ��
% x1=inv(x);%��
% û�а��棬bansuiX=inv(x)*det(x);
% x2=rank(x);%��
% x3=(x)';%ת��
% x4=trace(x);��
% det(x);%����ʽ��ֵ
% poly()%��������ʽ
% null(A,'r')%��ζ���ʽ�������ϵ
% diag()%�Խ���
% x5=eig(x0);%����ֵ
% [v,D]=eig(x0);%��������������ֵ�Խ���
% a=x*y*z
% a=hadmard 
% b=hankel
% z4=rand(4,4)%0-1random
% z5=randn(4,4)%����1����̬�ֲ�
% z6=hilb(3)
% z7=magic(10)
% z8=(z7)'
%% ����

%{

eps	������Ծ���
flintmax	�����ʽ�������������
i	������λ
j	������λ
Inf	��������ֵ��Ϊ Inf ������
% pi	Բ���ܳ�����ֱ���ı���
NaN	��������ֵ��Ϊ NaN ������
isfinite	ȷ����Щ����Ԫ��Ϊ����
isinf	ȷ����Щ����Ԫ��Ϊ����ֵ
isnan	ȷ����Щ����Ԫ��Ϊ NaN
compan	�������
gallery	���Ծ���
hadamard	Hadamard ����
hankel	Hankel ����
hilb	Hilbert ����
invhilb	Hilbert ����������
magic	�÷�����
pascal	��˹������
rosser	���ͶԳ�����ֵ��������
toeplitz	�������ľ���
vander	Vandermonde ����
wilkinson	Wilkinson ������ֵ���Ծ���

%}
% X = Inf �����������ı�����ʾ����������̫���������޷���ʾΪ������ʱ���� 1/0 �� log(0)������᷵�� Inf��
% 
% ����˫���ȣ�Inf ��ʾ���� realmax �����֡����ڵ����ȣ�Inf ��ʾ���� realmax('single') �����֡�
% X = NaN ���ء�����ֵ���ı�����ʾ��ʽ�����������δ�������ֵ������� 0/0 �� 0*Inf�������㷵�� NaN��


%% �ھ��� ������ѧ����
% 9.1 �������
% + - * / \ ^ '  %�������
% .* ./ .\ .^ .'  %�������
% A=[1 2;1 3];
% B=[1 1;1 2]; 
% C=[3 5;4 7];
% A\C,inv(A)*C
% A/C,A*inv(C)
% inv(C\A)==A\C
% syms x y a b c;
% A=[1 y;x 3];
% B=[1 x;1 y]; 
% C=[3 5;4 7];
% [a,b]=solve(A*B==C,x,y)
% syms x y m n a b;
% [a b]=solve(x==m*cos(a)+n*cos(a+b),y==m*sin(a)+n*sin(a+b),a,b)
% a=[1 0;0 2];
% b=[1;3];
% syms x1 x2;
% y=a*[x1;x2]+b;
% [x1 x2]=solve(y(1),y(2),x1,x2)
% syms x1 x2 x3 x4;
% a=[1 0;0 2];
% b=[1 1;2 3];
% y=a*[x1 x2;x3 x4]-b;
% [x1,x2,x3,x4]=solve(y,x1,x2,x3,x4);
% s=solve(y,x1,x2,x3,x4);
% s.x1
% s.x2
% syms x;
% eqn = sin(x) == 1;%solx = solve(eqn,x)
% [solx, params, conds] = solve(sin(x)==0, x, 'return',1)
% syms x y a b c;%ָ����
% a=2;
% y=a^x;y=exp(x)%e=exp(1)
% y=log(x);%log(x)=>ln(x)
% y=x^a;
% aEb;%��ѧ������a*10^b;



%% �������ɺ���
% a=hadmard  %Hadamard ����
% b=hankel
% z4=rand(4,4)%[0-1] random
% z5=randn(4,4)%����1����̬�ֲ�
% z6=hilb(3)
% z7=magic(10)
% z8=(z7)'


% a=pascal(3);
% a=[1 1 1;1 2 3;1 3 6];
% b=magic(3);
% b=[8 1 6;3 5 7;4 9 2];
% c=a*b,d=a.*b
% x=1:1:5
% y=logspace(1,2,10)
% y=linspace(1,10,10)






