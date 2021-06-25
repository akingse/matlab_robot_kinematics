%% initialize
clc; close all; clear all;
format shortg; format compact;
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
% Q(Quartic equation with one variable)意思是一元四次方程，此m脚本为一元四次方程解法；
% 这是第一次关于将两个六轴函数合在一起的尝试，只是这个版本的Virtual6函数有点问题，
% 应该是中间手算部分出了(0)*(equ)，导致出现了一元四次方程，一开始没有注意，实际上也能算出来但是走了远路，
% 中间化简部分，可以用syms符号运算，使用 collet() simplify() 函数。% 所以啊，你永远可以相信计算机！计算机是永远嘀神；
% 公式太多，文字太少，计算太复杂，每次当我回过头来想要重新运用这些代码的时候，都要花很多时间来审视这些代码；
带有Q一元四次方程的脚本版本，已经没有什么参考价值了；
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}


%% main

syms x0 y0 z0;  % 设定远心点
x0=-300;  
y0=-300;
z0=300;
% x0=-820;  y0=-180  ;z0=0;
Prcm=[x0 y0 z0];
Trcm=transl(x0,y0,z0)*trotx(pi/2);  % T0→Tv
Tircm=invtrot(trotx(pi/2))*invtrans(transl(x0,y0,z0));

T=rpy2tr(1,1,1);T(1,4)=-400;T(2,4)=-400;T(3,4)=-400;
% T %基于T0的final位姿
Tv=Tircm*T   %虚拟轴部分模型的4*4位姿矩阵[noap]获取

qv8=ikine_Virtual(Tv)  %8*9 or 8*6


j=1;
for i=1:8 % 获取8组关节角，筛选 90<th5<90，筛选d7>0
    if abs(qv8(i,5))<pi/2 && qv8(i,7)>0
        qv(j,1:9)=qv8(i,1:9);
        j=j+1;
    end
end


T4 = DH_forward(qv(1,4),0,0,pi/2);
T5 = DH_forward(qv(1,5),0,0,-pi/2);
T6 = DH_forward(qv(1,6),0,0,0);
Tr=Trcm*T4*T5*T6   %实体轴部分模型的4*4位姿矩阵[noap]获取
qr=ikine_reality(Tr)
%% function_Virtual
function th=ikine_Virtual(T)
% function th=Virtual6(T)
    a8=50;a9=50;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    % th9
    Px=px-a9*nx;
    Py=py-a9*ny;
    Pz=pz-a9*nz;    
    Pn=(Px*az-Pz*ax)*ny+(Pz*ay-Py*az)*nx+(Py*ax-Px*ay)*nz;
    Po=(Px*az-Pz*ax)*oy+(Pz*ay-Py*az)*ox+(Py*ax-Px*ay)*oz;
    at=a8*nz*Pn;
    bt=a8*oz*Po;
    ct=-a8*(oz*Pn+nz*Po);
    dt=-Pz*Pn;
    et=Pz*Po;
    a=(ct^2+(at-bt)^2);
    b=(2*ct*et+2*dt*(at-bt));
    c=(-ct^2+dt^2+et^2+2*bt*(at-bt));
    d=(2*bt*dt-2*ct*et);
    e=bt^2-et^2;
    x=Ferrari41(a,b,c,d,e);
%     x=real(x)
    lenx=length(x);
    if lenx==2;
        thx9(1:2)=acos(x); 
        thx9(3:4)=-thx9(1:2);
    else
        thx9(1:4)=acos(x);
        thx9(5:8)=-thx9(1:4);
    end
    Tri=at*cos(thx9).^2+bt*sin(thx9).^2+ct*cos(thx9).*sin(thx9)+dt*cos(thx9)+et*sin(thx9);
    j=1;
    for i=1:2*lenx % cos()sin()筛选
        if N_zero(Tri(i))==0
            the9(j)=thx9(i);
            j=j+1;
        end
    end

    % th8
    Rx= nx*cos(the9)-ox*sin(the9);  Rx=real(Rx);
    Ry= ny*cos(the9)-oy*sin(the9);  Ry=real(Ry);
    Rz= nz*cos(the9)-oz*sin(the9);  Rz=real(Rz);
    if lenx==2;
        m8=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx)); 
        n8=(Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz)); 
        the8(1:2)=atan2(m8,n8);
        the8(3:4)=the8(1:2)+pi;
        the9(3:4)=the9(1:2);
    else
        m8=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx));
        n8=(Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz));
        the8(1:4)=atan2(m8,n8);
        the8(5:8)=the8(1:4)+pi;
        the9(5:8)=the9(1:4);
    end
    % d7
    M3=(az*cos(the8)+sin(the8).*(nz*cos(the9)-oz*sin(the9)));
    N3=(Pz-a8*(nz*cos(the9)-oz*sin(the9))); 
    j=1;
    if lenx==4
        for i=1:8 % 非零筛选
            if N_zero(M3(i)) && N_zero(N3(i))
                th8(j)=the8(i);
                th9(j)=the9(i);
                j=j+1;
            end
        end
    else
        th8=the8;
        th9=the9;    
    end
    d7(1:4)=(pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9)))./(az*cos(th8)+sin(th8).*(nz*cos(th9)-oz*sin(th9)));
    d7(5:8)=d7(1:4);
    
    % th5 th6 th4
    n5=az*cos(th8) + sin(th8).*(nz*cos(th9)-oz*sin(th9));
    th5(1:4)=acos(n5);
    th5(5:8)=-th5(1:4);
    
    th8(5:8)=th8(1:4);
    th9(5:8)=th9(1:4);
    if n5==1
        n46=-ax*sin(th8) + cos(th8).*(nx*cos(th9)-ox*sin(th9));
        m46=-ay*sin(th8) + cos(th8).*(ny*cos(th9)-oy*sin(th9));
        th46=atan2(m46,n46);
        th4(1:8)=0;
        th6=th46;
    else
        n4=-(ax*cos(th8)+sin(th8).*(nx*cos(th9)-ox*sin(th9)))./sin(th5);
        m4=-(ay*cos(th8)+sin(th8).*(ny*cos(th9)-oy*sin(th9)))./sin(th5);
        th4=atan2(m4,n4);
        n6=(-az*sin(th8)+cos(th8).*(nz*cos(th9)-oz*sin(th9)))./sin(th5);
        m6=(-oz*cos(th9)-nz*sin(th9))./sin(th5);
        th6=atan2(m6,n6);
    end
    
    for i=1:8
        th(i,9)=th9(i);
        th(i,8)=In_pi(th8(i));
        th(i,7)=d7(i); %*pi/180;
        th(i,5)=th5(i);
        th(i,4)=th4(i);
        th(i,6)=th6(i);
    end
end
%% function_reality
function th=ikine_reality(T)
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    % th1
    m1=d6*ay-py;
    n1=d6*ax-px;
    if (d4^2<=m1^2+n1^2)
        k1=d4/sqrt(m1^2+n1^2);
        th1(1)=atan2(m1,n1)-atan2(k1,sqrt(1-k1^2));
        th1(2)=atan2(m1,n1)-atan2(k1,-sqrt(1-k1^2));
    else
        th1(1:2)=NaN;
    end
    % th5
    k5=ax*sin(th1)-ay*cos(th1);
    th5(1:2)=acos(k5);
    th5(3:4)=-th5(1:2);
    % th6
    n6=ny*cos(th1)-nx*sin(th1);
    m6=oy*cos(th1)-ox*sin(th1);
    n6=N_zero(n6);
    m6=N_zero(m6);
    if m6(1)==0 && n6(1)==0 %Singularity①
        th6(1)=0; %or th6=read(theta_6);
        th6(2)=atan2(m6(2),-n6(2)); % ?
    else
        th6(1:2)=atan2(m6,-n6);
        th6(3:4)=th6(1:2)-pi;
    end
    
    % th3 % th2
    th1(3:4)=th1(1:2);
    m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)));
    n23=pz-d1-d6*az+d5*(nz*sin(th6)+oz*cos(th6)); 
    m23=N_zero(m23);
    n23=N_zero(n23);
    k3=(m23.^2+n23.^2-a2^2-a3^2)/(2*a2*a3);
    k3=K_one(k3);
    th3(1:4)=acos(k3);
    th3(5:8)=-th3(1:4);
    k2=(m23.^2+n23.^2+a2^2-a3^2)./sqrt((2*a2*m23).^2+(2*a2*n23).^2);
    k2=K_one(k2);
    th2(1:4)=atan2(2*n23*a2,2*m23*a2)-acos(k2);
    th2(5:8)=atan2(2*n23*a2,2*m23*a2)+acos(k2);
    % th4
    m234=-sin(th6).*(nx*cos(th1)+ny*sin(th1))-cos(th6).*(ox*cos(th1)+oy*sin(th1));
    n234=nz*sin(th6)+oz*cos(th6);
    m234=N_zero(m234);
    n234=N_zero(n234);
    th234(1:4)=atan2(m234,n234);
    th234(5:8)=th234(1:4);
    th4=th234-th3-th2;
    % sort out;
    th1(5:8)=th1(1:4);
    th5(5:8)=th5(1:4);
    th6(5:8)=th6(1:4);
    for i=1:8
        th(i,1)=th1(i);
        th(i,2)=th2(i);
        th(i,3)=th3(i);
        th(i,4)=In_pi(th4(i));
        th(i,5)=th5(i);
        th(i,6)=In_pi(th6(i));
    end
end
%% linear_function
function x=Ferrari41(a,b,c,d,e)  % 费拉里法
    b1=b/a;  c1=c/a;  d1=d/a;  e1=e/a; %a1=1
    a=-1;  b=c1;  c=(4*e1-b1*d1);  d=d1^2-e1*(4*c1-b1^2);
    y=Cardano31(a,b,c,d); % y=y(1);
    a2=(1/4*b1^2-c1+y);
    b2=(1/2*b1*y-d1); 
    c2=(1/4*y^2-e1);
    b=b1/2+sqrt(a2);
    c=y/2+sign(b2)*sqrt(c2);
    xc(1)=(-b+sqrt(b^2-4*c))/2; % a=1;
    xc(2)=(-b-sqrt(b^2-4*c))/2;
    b=b1/2-sqrt(a2);
    c=y/2-sign(b2)*sqrt(c2);
    xc(3)=(-b+sqrt(b^2-4*c))/2;
    xc(4)=(-b-sqrt(b^2-4*c))/2;
    j=1; 
    for i=1:length(xc) % 筛选实数解
        if isreal(xc(i))
            x(j)=xc(i);
            j=j+1;
        end
    end
end
function x=Cardano31(a,b,c,d)  % 卡尔丹公式
    b=b/a;  c=c/a;  d=d/a; % a=1;
    p=-b^2/3+c; 
    q=(2*b^3)/27-(c*b)/3+d;
    delta=(q/2)^2+(p/3)^3;  %delta 判定
    if delta>=0
        m=nthroot(-q/2+sqrt(delta),3); 
        n=nthroot(-q/2-sqrt(delta),3);
    else
        m=(-q/2+sqrt(delta))^(1/3);
        n=(-q/2-sqrt(delta))^(1/3);
    end
    x=m+n-b/3;
end
%% kinetic
function T = forward_kine49(theta) % virtual六轴专用函数，正运动位姿获取
    d=[0 0 0 0 0 0 0 0 0]; 
    a=[0 0 0 0 0 0 0 50 50]; 
    alpha=[0 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(0,theta(7),a(7),alpha(7)); 
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T4*T5*T6*T7*T8*T9;
end
function T = DH_forward(theta,d,a,alpha) 
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end
function T = DH_inverse(theta,d,a,alpha)
    T=invtrot(trotx(alpha))*invtrans(transl(a,0,0))*invtrot(trotz(theta))*invtrans(transl(0,0,d));
end
function [Ti]=invtrot(T)
    for i=1:2
        for j=i+1:3
            temp=T(i,j);
            T(i,j)=T(j,i);
            T(j,i)=temp;
        end
    end
    Ti=T;
end
function [Ti]=invtrans(T)
    for i=1:3
        T(i,4)=-T(i,4);
    end
    Ti=T;
end
%% standard
function mn = N_zero(mn)
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<1E-6
                mn(i,j)=0;
            end
        end
    end
end   
function theta = In_pi(theta)
    while (abs(theta)>pi)
        if (theta>pi)
            theta=theta-2*pi;
        elseif (theta<-pi)
            theta=theta+2*pi;
        end
    end
end
function k = K_one(k) %之前多虑了，解arccos()时，大部分情况下理论证明有 |k|<=1
% 否则，应该是超出了模型的工作空间；
    S=size(k);
    for i=1:S(1)
        for j=1:S(2)
            if abs(k(i,j))>1
                k(i,j)=NaN;
            end
        end
    end
end
% 
