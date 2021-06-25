%% initialize
clc; close all; clear all;
format shortg; format compact;

% x=-5:0.01:5;
% fun_image3(x,1,2,3,4);
% fun_image4(x,-1,2,3,4,5);


%% ����������Ĵη���
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



%% ��ֵ��
%{ �趨��������
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

    if abs(y(i))< 0.02 %1E-1 %����ֵ����ѧ����
        x(j)=i; %����������ֵ
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
% f = @(x) x^2;  f(10);  %�򵥶��庯��
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



%% һԪ�Ĵ�
function x=Cartesian4(a,b,c,d,e)
    % �ѿ�����������ϵ����
    b=b/a;  c=c/a;  d=d/a;  e=e/a;  b1=b; %��������b
    % T=x^4+b*x^3+c*x^2+d*x+e;  %a==1 %x=(y-b/4); %��������
    % syms b c d e y;   T=(y-b/4)^4+b*(y-b/4)^3+c*(y-b/4)^2+d*(y-b/4)+e;
    % collect(T,y) % y^4+(c-(3*b^2)/8)*y^2+(b^3/8-(c*b)/2+d)*y-(3*b^4)/256+(c*b^2)/16-(d*b)/4+e
    p=(c-(3*b^2)/8); 
    q=(b^3/8-(c*b)/2+d);
    r=-(3*b^4)/256+(c*b^2)/16-(d*b)/4+e;
    % T=y^4+p*y^2+q*y+r;  %b==0
    % y^4+p*y^2+q*y+r==(y^2+k*y+m)*(y^2+k*y+n)
    % syms k m n;  T=(y^2+k*y+m)*(y^2-k*y+n);
    % collect(T,y) % y^4+p*y^2+q*y+r==y^4+(-k^2+m+n)*y^2+(k*n-k*m)*y+m*n
    % r=((k^3+c*k)^2-d^2)/(4*k^2);  %��Ҫ��� k
    % syms p q r k;  T=((k^3+p*k)^2-q^2)-(4*k^2)*r;
    % collect(T,k) % k^6+2*p*k^4+(p^2-4*r)*k^2-q^2  % ��x=k^2;
    % a*x^3+b*x^2+c*x+d==0  %��һԪ���η���
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
    % �ѿ��������򻯰�
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
    for i=1:length(x) % ɸѡʵ����
        if isreal(x(i))
            z(j)=x(i);
            j=j+1;
        end
    end
    x=z;
end
function x=Ferrari40(a,b,c,d,e)
    % �����﷨�� һԪ�Ĵη���
    % T=a*x^4+b*x^3+c*x^2+d*x+e; %һ��ʽ a0
    b1=b/a;  c1=c/a;  d1=d/a;  e1=e/a; %a1=1
    % T1: x^4+b1*x^3+c1*x^2+d1*x+e1==0  % a==1
    % T2: x^4+b1*x^3==-c1*x^2-d1*x-e1  % ����
    % T3: (x^2+1/2*b1*x)^2==(1/4*b1^2-c1)*x^2-d1*x-e1  % ��ʽ����ͬʱ���� (1/2*b1*x)^2
    % T4: (x^2+1/2*b1*x+y/2)^2==(1/4*b1^2-c1+y)*x^2+(1/2*b1*y-d1)*x+(1/4*y^2-e1) % ��ʽ����ͬʱ���� (x^2+1/2*b1*x)*y+1/4*y^2 
    % A^2=B^2;  % T4ʽ�е�y��һ����������T4ʽ�е�xΪԭ���̵ĸ�ʱ������yȡ��ֵT4ʽ�������
    % syms b1 c1 d1 e1 y;delta=(1/2*b1*y-d1)^2-4*(1/4*b1^2-c1+y)*(1/4*y^2-e1);  %��=b^2-4ac==0 �б�ʽ
    % collect(delta,y) % delta=-y^3+c1*y^2+(4*e1-b1*d1)*y+d1^2-e*(4*c1-b1^2); %����Ϊһ��ʽ
    a=-1; %deltaר��abcd
    b=c1;
    c=(4*e1-b1*d1);
    d=d1^2-e1*(4*c1-b1^2);
    y=Cardano30(a,b,c,d);
    y=y(1);
    a2=(1/4*b1^2-c1+y);  %B^2���̲���abc
    b2=(1/2*b1*y-d1); 
    c2=(1/4*y^2-e1);
    % B==(1/4*b1^2-c1+y)*x^2+(1/2*b1*y-d1)*x+(1/4*y^2-e1)
    % (x^2+1/2*b1*x+y/2)^2==a2*x^2+b2*x+c2==(sqrt(a)*x+sign(b)*sqrt(c))^2 % b�ķ����ж�
    % x^2+(b1/2��sqrt(a2))+(y/2��sqrt(c2))==0 %ͬ��ͬ��
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
    %�����﷨�򻯰�
    % T=a*x^4+b*x^3+c*x^2+d*x+e; %һ��ʽ
    % T=x^4+b*x^3+c*x^2+d*x+e; 
    b1=b/a;  c1=c/a;  d1=d/a;  e1=e/a; 
    % T=8*y^3-4*c1*y^2+(2*b1*d1-8*e1)*y+e1*(4*c1-b1^2)-d1^2; 
    a=8;
    b=-4*c1;
    c=(2*b1*d1-8*e1);
    d=e1*(4*c1-b1^2)-d1^2; 
    y=Cardano3(a,b,c,d);
    y=y(1);
    m=sqrt(8*y+b1^2-4*c1); %�ж� m==0
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

%% һԪ����
function x=Shengjin3(a,b,c,d)
    % ʢ��ʽ
    % ��ʢ��. һԪ���η��̵��������ʽ�����б�[J]. ����ʦ��ѧԺѧ��,1989,2(2):91-98.
    % a*x^3+b*x^2+c*x+d==0  %һԪ���η���һ��ʽ
    % x^3+b*x^2+c*x+d==0  % a==1   
    A=b^2-3*a*c;  N_zero(A);
    B=b*c-9*a*d; %��������С�ж�����
    C=c^2-3*b*d;
    delta=B^2-4*A*C;  N_zero(delta);  %���б�ʽ
    if delta>0  %������һ��ʵ����һ�Թ������
        Y1=A*b+3*a*(-B+sqrt(B^2-4*A*C))/2;
        Y2=A*b+3*a*(-B-sqrt(B^2-4*A*C))/2;
        y1=nthroot(Y1,3); %����matlab�ڲ����㷨��
        y2=nthroot(Y2,3); %�����θ���
        % x1=(-b-Y1^(1/3)-Y2^(1/3))/3; 
        x(1)=(-b-y1-y2)/(3*a);
        x(2)=(-b+(1/2)*(y1+y2)+(sqrt(3)/2)*(y1-y2)*1i)/(3*a);
        x(3)=(-b+(1/2)*(y1+y2)-(sqrt(3)/2)*(y1-y2)*1i)/(3*a);
    elseif delta==0  %����������ʵ����������һ�����ظ���
        if A==0 % ��ʱ A==B==0
            x(1)=-c/b; % ������
            x(2)=x(1);  
            x(3)=x(1);
        else
            K=B/A; %A~=0
            x(1)=-b/a+K;
            x(2)=-K/2;  
            x(3)=x(2);
        end
    else % delta<0  %��������������ȵ�ʵ����
        theta=acos((2*A*b-3*a*B)/(2*sqrt(A^3)))/3;
        x(1)=(-b-2*sqrt(A)*cos(theta))/3/a;
        x(2)=(-b+sqrt(A)*(cos(theta)+sqrt(3)*sin(theta)))/3/a;
        x(3)=(-b+sqrt(A)*(cos(theta)-sqrt(3)*sin(theta)))/3/a;
    end
end



function x=Cardano30(a,b,c,d)
    % ��������ʽ
    % a*x^3+b*x^2+c*x+d==0  %һ��ʽ
    b=b/a;  c=c/a;  d=d/a;  a=1;  %ͬ��a
    % T=(y-b/3)^3+b*(y-b/3)^2+c*(y-b/3)+d  % x=y-b/3;  %��������
    % collect(T,y) % y^3+(-b^2/3+c)*y+(2*b^3)/27-(c*b)/3+d
    p=-b^2/3+c;  %c
    q=(2*b^3)/27-(c*b)/3+d;  %d
    % y^3+c*y+d==0  %b==0���������
    delta=(q/2)^2+(p/3)^3;  %delta �ж���������
    w=(-1+sqrt(3)*1i)/2; %Ϊ��ǿ�ȶ��ԣ���i�滻Ϊ1i
    if delta>=0 %һ��ʵ������������
        m=nthroot(-q/2+sqrt(delta),3); %ֻ��ʾʵ��������
        n=nthroot(-q/2-sqrt(delta),3);
    else
        m=(-q/2+sqrt(delta))^(1/3);  % deltaͨ���㷨
        n=(-q/2-sqrt(delta))^(1/3);
    end
    y1=m+n;  %ʵ����
    y2=w*m+w^2*n;  %��ʵ����  
    y3=w^2*m+w*n;
    x(1)=y1-b/3;
    x(2)=y2-b/3;
    x(3)=y3-b/3;
end
function x=FViete3(a,b,c,d) %���� delta<0 ʱ
    % ������ѧ��Τ��(F.Viete)���������Ǻ��ʽ�����˲���Լ���εķ��̵ĸ�
    % a*x^3+b*x^2+c*x+d==0  %һ��ʽ
    % y^3+c*y+d==0  %b==0���������
    b=b/a;  c=c/a;  d=d/a;  a=1;  %ͬ��a
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
% check = t_go.^4 + 6*t_go.^2 - 60*t_go + 36   %������������ȷ��
% 1  0  c  d  e
a = 6;  b = -60;  c = 36;

alpha = -a^2/12 - c;
beta = -a^3/108 + a*c/3 - b^2/8;
delta = alpha^3/27 + beta^2/4;
Z = nthroot(-beta/2+sqrt(delta),3) + nthroot(-beta/2-sqrt(delta),3);
eta = Z - 5*a/6;   %�ÿ�������������η���һ�����ڵ��Ǹ�ʵ����eta
h = a + eta;
xi = sqrt(2*eta+a);
zeta = -b/(2*sqrt(2*eta+a));
t_go(1) = (xi + sqrt(xi^2 - 4*(h-zeta)))/2;
t_go(2) = (xi - sqrt(xi^2 - 4*(h-zeta)))/2;
t_go(3) = (-xi + sqrt(xi^2 - 4*(h+zeta)))/2;
t_go(4) = (-xi - sqrt(xi^2 - 4*(h+zeta)))/2;

%}
%{
function x=Cardano30(a,b,c,d) % ����ר��
p=(3*a*c-b^2)/(3*a^2);
q=(27*a^2*d-9*a*b*c+2*b^3)/(27*a^3);
delta=(q/2)^2+(p/3)^3;
m=nthroot(-q/2+sqrt(delta),3);
n=nthroot(-q/2-sqrt(delta),3);
x=m+n-b/(3*a);
end
%}
%{
% matlab ���ο�������
% sqrt(r)*(cos(phi/2) + 1i*sin(phi/2)) ���ο���
% -8=8(cos��+isin��) ������
% ?�̣�-8��=2[cos(��+2k��)/3+isin(��+2k��)/3] k=0,1,2 Ĭ��0
%  (-8)^(1/3)=(-1)^(1/3)*(8)^(1/3)=2*(exp(i*2*pi))^(1/3) =2*(exp(i*pi*n/3))(n=0,1,2) 
% k=0:2;  3*exp((1+2*k)*pi/3*i)
% nthroot(X,N)==(X)^(1/N)  %ֻ��ʾʵ����
roots([1 0 0 -8]) %ת���ɷ�����ʽ��������и����б�
sign(-8)*abs(-8^(1/3))
%}

