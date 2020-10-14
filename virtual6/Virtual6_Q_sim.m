%% initialize
clc; close all; clear all;
format shortg; format compact;

%% initial Virtual6

% d7=400;
% theta_deg=[0 0 0 10 20 30 d7*180/pi 50 60]
% theta=theta_deg/180*pi;
theta=[0 0 0 pi/2*(2*rand(1)-1) pi/2*(2*rand(1)-1) pi*(2*rand(1)-1) 10 pi*(2*rand(1)-1) pi*(2*rand(1)-1)];
% theta=[0  0  0  -30.937      -89.429       -24.31       5729.6      -87.366       71.545]/180*pi;  % x4jie
% theta_deg=theta*180/pi
% theta_deg=[0 0 0 10 20 30 40*180/pi 50 60]
% theta=theta_deg/180*pi;

T=forward_kine49(theta);
% T=[       0.5403      0.70807      0.45465         -100
%      -0.84147      0.45465      0.29193         -100
%             0      -0.5403      0.84147          100
%             0            0            0            1]
% T=rpy2tr(1,1,0);T(1,4)=-100;T(2,4)=-100;T(3,4)=500;
theta=ikine_Virtual(T);
theta_deg=theta*180/pi

for i=1:8
%     eval(['Q',num2str(i),'=','vpa(forward_kine49(theta(i,1:9))-T,6)']);
    T8=forward_kine49(theta(i,1:9))-T;
    eval(['Q',num2str(i),'=','vpa(sum(sum(T8)),6)']);
end

%% function
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
    Pn=Px*(az*ny-ay*nz)+Py*(ax*nz-az*nx)+Pz*(ay*nx-ax*ny);
    Po=Px*(az*oy-ay*oz)+Py*(ax*oz-az*ox)+Pz*(ay*ox-ax*oy);
    
    A=a8*nz*Pn;
    B=a8*oz*Po;
    C=-a8*(oz*Pn+nz*Po);
    D=-Pz*Pn;
    E=Pz*Po;
    a=(C^2+(A-B)^2);
    b=(2*C*E+2*D*(A-B));
    c=(-C^2+D^2+E^2+2*B*(A-B));
    d=(2*B*D-2*C*E);
    e=B^2-E^2;
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
    Tri=A*cos(thx9).^2+B*sin(thx9).^2+C*cos(thx9).*sin(thx9)+D*cos(thx9)+E*sin(thx9);
    j=1;
    for i=1:2*lenx % cos()sin()筛选
        if N_zero(Tri(i))==0
            the9(j)=thx9(i);
            j=j+1;
        end
    end
    Tri=Pz - a8*(nz*cos(the9)-oz*sin(the9));
    j=1;
    if lenx==4 % 筛去(Pz - Rz*a8)==0的部分
        for i=1:4 
            if N_zero(Tri(i))~=0
                th9(j)=the9(i);
                j=j+1;
            end
        end
    else
        th9=the9;
    end
    % th8
    Rx= nx*cos(th9)-ox*sin(th9);
    Ry= ny*cos(th9)-oy*sin(th9);
    Rz= nz*cos(th9)-oz*sin(th9);
    m8=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx)); 
    n8=(Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz)); 
    th8(1:2)=atan2(m8,n8);
    th8(3:4)=th8(1:2)+pi;
    th9(3:4)=th9(1:2);
    % d7
    Mz=(az*cos(th8)+sin(th8).*(nz*cos(th9)-oz*sin(th9))); 
    Nz=(Pz-a8*(nz*cos(th9)-oz*sin(th9))); 
    d7(1:4)=Nz./Mz;
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
        th(i,7)=d7(i); %*pi/180;  %被动转角度
        th(i,5)=th5(i);
        th(i,4)=th4(i);
        th(i,6)=th6(i);
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

%% kinetic_function
function T = forward_kine49(theta) % virtual六轴专用函数，正运动位姿获取
    d=[0 0 0 0 0 0 0 0 0]; %syms  a8 a9 d7; %     a8=50;a9=50; %d7=0;
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
function T = DH_forward(theta,d,a,alpha) % 正运动函数
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end



