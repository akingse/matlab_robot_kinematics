%% initialize
clc; close all; clear all;
format shortg; format compact;

% x=-5:0.01:5;
% fun_image3(x,1,2,3,4);
% fun_image4(x,-1,2,3,4,5);


%% 随机参数的四次方程
a=1;    b=0;    c=-12;    d=14;  e=0;%14  16  18
a=(2*rand(1)-1); 
b=(2*rand(1)-1);
c=(2*rand(1)-1);
d=(2*rand(1)-1);
e=(2*rand(1)-1);
x=Cartesian4(a,b,c,d,e)
% x=Shengjin3(a,b,c,d)
% x=Cardano3(a,b,c,d)
% x=FViete3(a,b,c,d)
% x=Ferrari40(a,b,c,d,e)



%% 数值法
%{ 设定步长搜索
a = 10*(2*rand(1)-1);
b = 10*(2*rand(1)-1);
c = 10*(2*rand(1)-1);
d = 10*(2*rand(1)-1);
e = 10*(2*rand(1)-1);
A=[a b c d e];
theta = -pi:0.001:pi;
x=[];% syms x
j=1;
for i = 1:length(theta)
    y(i) = a*cos(theta(i))^2+b*sin(theta(i))^2+c*cos(theta(i))*sin(theta(i))+d*cos(theta(i))+e*sin(theta(i));
    w(i) = 0;

    if abs(y(i))< 0.02 %1E-1 %绝对值，科学计数
        x(j)=i; %数组连续赋值
        j=j+1;
    end
end
% plot(theta,y);
% hold on;
% plot(theta,w);
% an=x/length(theta)
%}


%% image
function y = fun_image3(x,a,b,c,d)
% f = @(x) x^2;  f(10);  %简单定义函数
%     a=1;    b=0;    c=12;    d=16;
    figure(1);
    y=a*x.^3+b*x.^2+c*x+d;
    plot(x,y);
    hold on;
    plot(x,0*x);
end
function y = fun_image4(x,a,b,c,d,e)
%     a=1;    b=0;    c=12;    d=16;
    figure(2);
    y=a*x.^4+b*x.^3+c*x.^2+d.*x+e;
    plot(x,y);
    hold on;
    plot(x,0*x);

end
function mn = N_zero(mn)
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<1E-10  
                mn(i,j)=0;
            end
        end
    end
end 



%% 一元四次
function x=Cartesian4(a,b,c,d,e)
    % 笛卡尔法，待定系数法
    b=b/a;  c=c/a;  d=d/a;  e=e/a;  b1=b; %保留变量b
    % T=x^4+b*x^3+c*x^2+d*x+e;  %a==1 %x=(y-b/4); %变量代换
    % syms b c d e y;   T=(y-b/4)^4+b*(y-b/4)^3+c*(y-b/4)^2+d*(y-b/4)+e;
    % collect(T,y) % y^4+(c-(3*b^2)/8)*y^2+(b^3/8-(c*b)/2+d)*y-(3*b^4)/256+(c*b^2)/16-(d*b)/4+e
    p=(c-(3*b^2)/8); 
    q=(b^3/8-(c*b)/2+d);
    r=-(3*b^4)/256+(c*b^2)/16-(d*b)/4+e;
    % T=y^4+p*y^2+q*y+r;  %b==0
    % y^4+p*y^2+q*y+r==(y^2+k*y+m)*(y^2+k*y+n)
    % syms k m n;  T=(y^2+k*y+m)*(y^2-k*y+n);
    % collect(T,y) % y^4+p*y^2+q*y+r==y^4+(-k^2+m+n)*y^2+(k*n-k*m)*y+m*n
    % r=((k^3+c*k)^2-d^2)/(4*k^2);  %需要求解 k
    % syms p q r k;  T=((k^3+p*k)^2-q^2)-(4*k^2)*r;
    % collect(T,k) % k^6+2*p*k^4+(p^2-4*r)*k^2-q^2  % 令x=k^2;
    % a*x^3+b*x^2+c*x+d==0  %解一元三次方程
    if q==0
        if r==0  % m==0 || n==0
            x(1)=-b1/4;  x(2)=x(1);
            x(3)=sqrt(p)-b1/4;  x(4)=-sqrt(p)-b1/4;
        else  %k==0
            m=(p+sqrt(p^2-4*r))/2;
            n=(p-sqrt(p^2-4*r))/2;
            x(1)=sqrt(m)-b1/4;  x(2)=-sqrt(m)-b1/4;  
            x(3)=sqrt(n)-b1/4;  x(4)=-sqrt(n)-b1/4;  
        end
    else
        a=1;  b=2*p;  c=p^2-4*r;  d=-q^2;
        x=Cardano30(a,b,c,d);
        k=sqrt(x(1)); % k~=0
        m=(k^3+p*k-q)/(2*k);
        n=(k^3+p*k+q)/(2*k);
        % x(1)=y(1)-b1/4;
        x(1)=(-k+sqrt(k^2-4*m))/2-b1/4; %a==1
        x(2)=(-k-sqrt(k^2-4*m))/2-b1/4;
        x(3)=(k+sqrt(k^2-4*n))/2-b1/4;
        x(4)=(k-sqrt(k^2-4*n))/2-b1/4;
    end
end

%% 
function x=Cartesian41(a,b,c,d,e)
    % 笛卡尔法，简化版
    b=b/a;  c=c/a;  d=d/a;  e=e/a;  b1=b; 
    p=(c-(3*b^2)/8);
    q=(b^3/8-(c*b)/2+d);
    r=-(3*b^4)/256+(c*b^2)/16-(d*b)/4+e;
    a=1;  b=2*p;  c=p^2-4*r;  d=-q^2;
    x=Cardano31(a,b,c,d);
    k=sqrt(x(1)); % k~=0
    m=(k^3+p*k-q)/(2*k);
    n=(k^3+p*k+q)/(2*k);
    x(1)=(-k+sqrt(k^2-4*m))/2-b1/4; %a==1
    x(2)=(-k-sqrt(k^2-4*m))/2-b1/4;
    x(3)=(k+sqrt(k^2-4*n))/2-b1/4;
    x(4)=(k-sqrt(k^2-4*n))/2-b1/4;
    j=1; z=[];
    for i=1:length(x) % 筛选实数解
        if isreal(x(i))
            z(j)=x(i);
            j=j+1;
        end
    end
    x=z;
end
function x=Ferrari40(a,b,c,d,e)
    % 费拉里法， 一元四次方程
    % T=a*x^4+b*x^3+c*x^2+d*x+e; %一般式 a0
    b1=b/a;  c1=c/a;  d1=d/a;  e1=e/a; %a1=1
    % T1: x^4+b1*x^3+c1*x^2+d1*x+e1==0  % a==1
    % T2: x^4+b1*x^3==-c1*x^2-d1*x-e1  % 移项
    % T3: (x^2+1/2*b1*x)^2==(1/4*b1^2-c1)*x^2-d1*x-e1  % 等式两边同时加上 (1/2*b1*x)^2
    % T4: (x^2+1/2*b1*x+y/2)^2==(1/4*b1^2-c1+y)*x^2+(1/2*b1*y-d1)*x+(1/4*y^2-e1) % 等式两边同时加上 (x^2+1/2*b1*x)*y+1/4*y^2 
    % A^2=B^2;  % T4式中的y是一个参数。当T4式中的x为原方程的根时，不论y取何值T4式恒成立。
    % syms b1 c1 d1 e1 y;delta=(1/2*b1*y-d1)^2-4*(1/4*b1^2-c1+y)*(1/4*y^2-e1);  %Δ=b^2-4ac==0 判别式
    % collect(delta,y) % delta=-y^3+c1*y^2+(4*e1-b1*d1)*y+d1^2-e*(4*c1-b1^2); %化简为一般式
    a=-1; %delta专属abcd
    b=c1;
    c=(4*e1-b1*d1);
    d=d1^2-e1*(4*c1-b1^2);
    y=Cardano30(a,b,c,d);
    y=y(1);
    a2=(1/4*b1^2-c1+y);  %B^2方程参数abc
    b2=(1/2*b1*y-d1); 
    c2=(1/4*y^2-e1);
    % B==(1/4*b1^2-c1+y)*x^2+(1/2*b1*y-d1)*x+(1/4*y^2-e1)
    % (x^2+1/2*b1*x+y/2)^2==a2*x^2+b2*x+c2==(sqrt(a)*x+sign(b)*sqrt(c))^2 % b的符号判定
    % x^2+(b1/2±sqrt(a2))+(y/2±sqrt(c2))==0 %同正同负
    b=b1/2+sqrt(a2); 
    % c=y/2+b2/2/sqrt(a2); %b^2=4ac; %a2~=0
    c=y/2+sign(b2)*sqrt(c2);
    x(1)=(-b+sqrt(b^2-4*c))/2; % a=1;
    x(2)=(-b-sqrt(b^2-4*c))/2;
    b=b1/2-sqrt(a2);
    c=y/2-sign(b2)*sqrt(c2);
    x(3)=(-b+sqrt(b^2-4*c))/2; 
    x(4)=(-b-sqrt(b^2-4*c))/2;
end
function x=Ferrari41(a,b,c,d,e)
    %费拉里法简化版
    % T=a*x^4+b*x^3+c*x^2+d*x+e; %一般式
    % T=x^4+b*x^3+c*x^2+d*x+e; 
    b1=b/a;  c1=c/a;  d1=d/a;  e1=e/a; 
    % T=8*y^3-4*c1*y^2+(2*b1*d1-8*e1)*y+e1*(4*c1-b1^2)-d1^2; 
    a=8;
    b=-4*c1;
    c=(2*b1*d1-8*e1);
    d=e1*(4*c1-b1^2)-d1^2; 
    y=Cardano3(a,b,c,d);
    y=y(1);
    m=sqrt(8*y+b1^2-4*c1); %判断 m==0
    n=b1*y-d1;
    a=2;
    b=b1+m;
    c=2*(y+n/m);
    x(1)=(-b+sqrt(b^2-4*a*c))/4;
    x(2)=(-b-sqrt(b^2-4*a*c))/4;
    a=2;
    b=b1-m;
    c=2*(y-n/m);
    x(3)=(-b+sqrt(b^2-4*a*c))/4;
    x(4)=(-b-sqrt(b^2-4*a*c))/4;
end

%% 一元三次
function x=Shengjin3(a,b,c,d)
    % 盛金公式
    % 范盛金. 一元三次方程的新求根公式与新判别法[J]. 海南师范学院学报,1989,2(2):91-98.
    % a*x^3+b*x^2+c*x+d==0  %一元三次方程一般式
    % x^3+b*x^2+c*x+d==0  % a==1   
    A=b^2-3*a*c;  N_zero(A);
    B=b*c-9*a*d; %加上无穷小判定函数
    C=c^2-3*b*d;
    delta=B^2-4*A*C;  N_zero(delta);  %总判别式
    if delta>0  %方程有一个实根和一对共轭复根。
        Y1=A*b+3*a*(-B+sqrt(B^2-4*A*C))/2;
        Y2=A*b+3*a*(-B-sqrt(B^2-4*A*C))/2;
        y1=nthroot(Y1,3); %由于matlab内部计算法则
        y2=nthroot(Y2,3); %开三次根号
        % x1=(-b-Y1^(1/3)-Y2^(1/3))/3; 
        x(1)=(-b-y1-y2)/(3*a);
        x(2)=(-b+(1/2)*(y1+y2)+(sqrt(3)/2)*(y1-y2)*1i)/(3*a);
        x(3)=(-b+(1/2)*(y1+y2)-(sqrt(3)/2)*(y1-y2)*1i)/(3*a);
    elseif delta==0  %方程有三个实根，其中有一个二重根。
        if A==0 % 此时 A==B==0
            x(1)=-c/b; % 三连等
            x(2)=x(1);  
            x(3)=x(1);
        else
            K=B/A; %A~=0
            x(1)=-b/a+K;
            x(2)=-K/2;  
            x(3)=x(2);
        end
    else % delta<0  %方程有三个不相等的实根。
        theta=acos((2*A*b-3*a*B)/(2*sqrt(A^3)))/3;
        x(1)=(-b-2*sqrt(A)*cos(theta))/3/a;
        x(2)=(-b+sqrt(A)*(cos(theta)+sqrt(3)*sin(theta)))/3/a;
        x(3)=(-b+sqrt(A)*(cos(theta)-sqrt(3)*sin(theta)))/3/a;
    end
end



function x=Cardano30(a,b,c,d)
    % 卡尔丹公式
    % a*x^3+b*x^2+c*x+d==0  %一般式
    b=b/a;  c=c/a;  d=d/a;  a=1;  %同除a
    % T=(y-b/3)^3+b*(y-b/3)^2+c*(y-b/3)+d  % x=y-b/3;  %变量代换
    % collect(T,y) % y^3+(-b^2/3+c)*y+(2*b^3)/27-(c*b)/3+d
    p=-b^2/3+c;  %c
    q=(2*b^3)/27-(c*b)/3+d;  %d
    % y^3+c*y+d==0  %b==0，特殊情况
    delta=(q/2)^2+(p/3)^3;  %delta 判定，或正或负
    w=(-1+sqrt(3)*1i)/2; %为增强稳定性，将i替换为1i
    if delta>=0 %一个实根和两个复根
        m=nthroot(-q/2+sqrt(delta),3); %只显示实数根函数
        n=nthroot(-q/2-sqrt(delta),3);
    else
        m=(-q/2+sqrt(delta))^(1/3);  % delta通解算法
        n=(-q/2-sqrt(delta))^(1/3);
    end
    y1=m+n;  %实数根
    y2=w*m+w^2*n;  %可实可虚  
    y3=w^2*m+w*n;
    x(1)=y1-b/3;
    x(2)=y2-b/3;
    x(3)=y3-b/3;
end
function x=FViete3(a,b,c,d) %仅当 delta<0 时
    % 法国数学家韦达(F.Viete)，利用三角恒等式给出了不可约情形的方程的根
    % a*x^3+b*x^2+c*x+d==0  %一般式
    % y^3+c*y+d==0  %b==0，特殊情况
    b=b/a;  c=c/a;  d=d/a;  a=1;  %同除a
    p=-b^2/3+c;  %(3*a*c-b^2)/(3*a^2);
    q=(2*b^3)/27-(c*b)/3+d;  %(27*a^2*d-9*a*b*c+2*b^3)/(27*a^2);
    delta=(q/2)^2+(p/3)^3;
    if delta<0
        r=sqrt((-p/3)^3);
        theta=acos(-q/2/r)/3;   
        if (isreal(r))
            R=nthroot(r,3);
        else
            R=r^(1/3);
        end

        x(1)=2*R*cos(theta)-b/3;
        x(2)=2*R*cos(theta+2*pi/3)-b/3;
        x(3)=2*R*cos(theta-2*pi/3)-b/3;
    else
    end
end
%{
% check = t_go.^4 + 6*t_go.^2 - 60*t_go + 36   %检验所求解的正确性
% 1  0  c  d  e
a = 6;  b = -60;  c = 36;

alpha = -a^2/12 - c;
beta = -a^3/108 + a*c/3 - b^2/8;
delta = alpha^3/27 + beta^2/4;
Z = nthroot(-beta/2+sqrt(delta),3) + nthroot(-beta/2-sqrt(delta),3);
eta = Z - 5*a/6;   %用卡尔丹法求解三次方程一定存在的那个实数解eta
h = a + eta;
xi = sqrt(2*eta+a);
zeta = -b/(2*sqrt(2*eta+a));
t_go(1) = (xi + sqrt(xi^2 - 4*(h-zeta)))/2;
t_go(2) = (xi - sqrt(xi^2 - 4*(h-zeta)))/2;
t_go(3) = (-xi + sqrt(xi^2 - 4*(h+zeta)))/2;
t_go(4) = (-xi - sqrt(xi^2 - 4*(h+zeta)))/2;

%}
%{
function x=Cardano30(a,b,c,d) % 测试专用
p=(3*a*c-b^2)/(3*a^2);
q=(27*a^2*d-9*a*b*c+2*b^3)/(27*a^3);
delta=(q/2)^2+(p/3)^3;
m=nthroot(-q/2+sqrt(delta),3);
n=nthroot(-q/2-sqrt(delta),3);
x=m+n-b/(3*a);
end
%}
%{
% matlab 三次开方规则
% sqrt(r)*(cos(phi/2) + 1i*sin(phi/2)) 二次开根
% -8=8(cosπ+isinπ) 复数解
% ?√（-8）=2[cos(π+2kπ)/3+isin(π+2kπ)/3] k=0,1,2 默认0
%  (-8)^(1/3)=(-1)^(1/3)*(8)^(1/3)=2*(exp(i*2*pi))^(1/3) =2*(exp(i*pi*n/3))(n=0,1,2) 
% k=0:2;  3*exp((1+2*k)*pi/3*i)
% nthroot(X,N)==(X)^(1/N)  %只显示实数根
roots([1 0 0 -8]) %转换成方程形式，求出所有根的列表
sign(-8)*abs(-8^(1/3))
%}

