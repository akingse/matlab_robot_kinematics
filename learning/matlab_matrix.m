%% initialize
clc; close all; clear all;
format shortg; format compact;
% 现学现用现用现学，需要及时处理。

% scalar | vector | matrix | multi_array
% 标量   |  矢量  |  矩阵  |  多维数组

%% 矩阵operate
% zeros(3),%0 %零阵
% ones(3,4),1 一阵，E 
% eye(3) ,单位阵；

% x1=inv(x);%逆
% 没有伴随，bansuiX=inv(x)*det(x);
% x2=rank(x);%秩
% x3=(x)';%转置
% x4=trace(x);迹
% det(x);%行列式的值
% poly()%特征多项式
% null(A,'r')%齐次多项式方程组解系
% diag()%对角线
% x5=eig(x0);%特征值
% [v,D]=eig(x0);%特征向量，特征值对角阵

%% 矩阵函数
% zeros(3),ones(3,4),eye(3)%0 1 E %零阵，一阵，单位阵；
% x1=inv(x);%逆
% 没有伴随，bansuiX=inv(x)*det(x);
% x2=rank(x);%秩
% x3=(x)';%转置
% x4=trace(x);迹
% det(x);%行列式的值
% poly()%特征多项式
% null(A,'r')%齐次多项式方程组解系
% diag()%对角线
% x5=eig(x0);%特征值
% [v,D]=eig(x0);%特征向量，特征值对角阵
% a=x*y*z
% a=hadmard 
% b=hankel
% z4=rand(4,4)%0-1random
% z5=randn(4,4)%方差1的正态分布
% z6=hilb(3)
% z7=magic(10)
% z8=(z7)'
%% 函数

%{

eps	浮点相对精度
flintmax	浮点格式的最大连续整数
i	虚数单位
j	虚数单位
Inf	创建所有值均为 Inf 的数组
% pi	圆的周长与其直径的比率
NaN	创建所有值均为 NaN 的数组
isfinite	确定哪些数组元素为有限
isinf	确定哪些数组元素为无限值
isnan	确定哪些数组元素为 NaN
compan	伴随矩阵
gallery	测试矩阵
hadamard	Hadamard 矩阵
hankel	Hankel 矩阵
hilb	Hilbert 矩阵
invhilb	Hilbert 矩阵的逆矩阵
magic	幻方矩阵
pascal	帕斯卡矩阵
rosser	典型对称特征值测试问题
toeplitz	托普利茨矩阵
vander	Vandermonde 矩阵
wilkinson	Wilkinson 的特征值测试矩阵

%}
% X = Inf 返回正无穷大的标量表示。当运算结果太大以至于无法表示为浮点数时，如 1/0 或 log(0)，运算会返回 Inf。
% 
% 对于双精度，Inf 表示大于 realmax 的数字。对于单精度，Inf 表示大于 realmax('single') 的数字。
% X = NaN 返回“非数值”的标量表示形式。如果运算有未定义的数值结果，如 0/0 或 0*Inf，则运算返回 NaN。


%% 第九章 符号数学计算
% 9.1 运算符号
% + - * / \ ^ '  %矩阵操作
% .* ./ .\ .^ .'  %数组操作
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
% syms x y a b c;%指对幂
% a=2;
% y=a^x;y=exp(x)%e=exp(1)
% y=log(x);%log(x)=>ln(x)
% y=x^a;
% aEb;%科学计数，a*10^b;



%% 矩阵生成函数
% a=hadmard  %Hadamard 矩阵
% b=hankel
% z4=rand(4,4)%[0-1] random
% z5=randn(4,4)%方差1的正态分布
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






