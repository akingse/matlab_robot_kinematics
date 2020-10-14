%% initialize
clc; close all; clear all;
format shortg; format compact;

%% Mathematics
% Equation solving, formula simplification, calculus, linear algebra, and more
% Symbolic Math Toolbox? provides capabilities for a variety of mathematical tasks, including differentiation, integration, simplification, transforms, 
% linear algebra operations, and equation solving. The emphasis is on performing computations analytically, as well as using high-precision numerical computations.

%% 多项式方程
p=[1 -3 2];%代表多项式 x^2-3x+2
q=[1 1];
% x=roots(p)%
% k=conv(p,q)%
% f=deconv(p,q)%
% polyval(p,3)%

% solve函数
syms a b c x;
eqn = a*x^2 + b*x + c == 0;
solx = solve(eqn, x) %-(b +- (b^2 - 4*a*c)^(1/2))/(2*a)
solb = solve(eqn, b) %-(a*x^2 + c)/x

% fplot(cos(x));
% hold on
% grid on
% fplot(-sin(x));
% title('Both sides of equation cos(x) = -sin(x)')
% legend('cos(x)','-sin(x)','Location','best','AutoUpdate','off')

%% 微分 q = integral(fun,xmin,xmax)
syms x y;
y1=sin(3*x+2);
y2=log(exp(3*x)+2);
y3=exp(3*x)*cos(x);

% dy1=[char(diff(y1,x)),'dx'] %符号积分
% dy2=[char(diff(y2,x)),'dx'];
% dy3=[char(diff(y3,x)),'dx'];
dy1=diff(y1,x) %求导




%% 积分
fun = @(x) exp(-x.^2).*log(x).^2;
% 计算 x=0 至 x=Inf 的积分。
q = integral(fun,0,Inf)



%% 函数
f=inline('sin(x)','x')  %内联函数
% f(2*x)
% f(pi/2)
% f(pi)


syms  x y;
f1(x)
f1(pi/6)
f2(x)
subs(f2(x),pi/6)
% subs(f2(x),x,pi/6)%subs(f2(x),old,new)

function [y1]=f1(x)
y1=sin(x);
end
function [y2]=f2(x)
y2=diff(f1(x));
% y2=f1(x)+1;
end



