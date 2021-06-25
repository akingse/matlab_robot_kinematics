%% initialize
clc; close all; clear all;
format shortg; format compact;
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
常用函数区
包括运动学函数，正向运动学的标准DH变换，旋转和平移的正逆矩阵；
基础的位姿描述方法之间的转换，
逆运动里的三角函数参数的数据标准化处理；

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

% 测试
%直线插补

  T=[3^0.5/2 0 -0.5  -45;
     -0.5 0 -3^0.5/2 -77.94;
      0   1   0    0;
      0   0   0    1 ]
T1=[1 0 0 200;
   0 1 0 200;
   0 0 1 200;
   0 0 0 1]
P=tr2pose(T)
P1=tr2pose(T1)
T=pose2tr(P)


% P1=[100 100 100 pi 0 0];
% P2=[200 200 100 pi 0 0];

% for i=1:100
%     for j=1:6
%         P(i,j)=P1(j)+i*(P2(j)-P1(j))/100;
%     end
% end
% 
% for i=1:100
%     T=pose2tr(P(i,:));
%     Q=inverse_kine(T);%*180/pi;
%     Q=Q(1,:);
%     forward_kine(Q)
% end
% T1=pose2tr(P1);
% T2=pose2tr(P2);
% Q1=inverse_kine(T1)*180/pi;
% Q2=inverse_kine(T2)*180/pi;
%
    % T=angvec2tr(2*pi*rand(1),[rand(1) rand(1) rand(1)]);
    % T(1,4)=636;T(2,4)=546;T(3,4)=546
% T=rpy2tr(0,0,0);
% T=rpy2tr(-pi/3,-pi/3,pi/3);
% T(1,4)=-400;T(2,4)=400;T(3,4)=490
% T=rpy2tr(0,0,0);% T(1,4)=-400;T(2,4)=400;T(3,4)=490
% T=rpy2tr(-pi/2,pi,-pi/2);
% T(1,4)=-730;T(2,4)=0;T(3,4)=90

%% functions
function T = forward_kine(theta)
    d1=90;d4=90;d5=90;d6=90; d7=0;
    a2=-420;a3=-400;a8=50;a9=50;
    d=[d1 0 0 d4 d5 d6 d7 0 0];
    a=[0 a2 a3 0 0 0 0 a8 a9];
    alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T=T1*T2*T3*T4*T5*T6;
end



% mdh =
% [            cos(theta),           -sin(theta),           0,             a]
% [ cos(alpha)*sin(theta), cos(alpha)*cos(theta), -sin(alpha), -d*sin(alpha)]
% [ sin(alpha)*sin(theta), sin(alpha)*cos(theta),  cos(alpha),  d*cos(alpha)]
% [                     0,                     0,           0,             1]
% sdh =
% [ cos(theta), -cos(alpha)*sin(theta),  sin(alpha)*sin(theta), a*cos(theta)]
% [ sin(theta),  cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)]
% [          0,             sin(alpha),             cos(alpha),            d]
% [          0,                      0,                      0,            1]


%标准型变换公式 T_standard %无法自动处理高阶无穷小
function T = SDH_forward(theta,d,a,alpha) 
    T=[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
    sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
    0,sin(alpha),cos(alpha),d;
    0,0,0,1];
end

function T = SDH_inverse(theta,d,a,alpha) % 逆运动函数
    T=[      cos(theta),             sin(theta),          0,            -a;
 -cos(alpha)*sin(theta),  cos(alpha)*cos(theta), sin(alpha), -d*sin(alpha);
  sin(alpha)*sin(theta), -sin(alpha)*cos(theta), cos(alpha), -d*cos(alpha);
                   0,                   0,                0,             1];
end
function T = DH_forward(theta,d,a,alpha) %T_standard(theta,d,a,alpha)
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end


function T = DH_inverse(theta,d,a,alpha) %DH_inverse()
    T=invtrot(trotx(alpha))*invtrans(transl(a,0,0))*invtrot(trotz(theta))*invtrans(transl(0,0,d));
end

function Ti=invtrot(T)
    for i=1:2
        for j=i+1:3
            temp=T(i,j);
            T(i,j)=T(j,i);
            T(j,i)=temp;
        end
    end
    Ti=T;
end
function Ti=invtrans(T)
    for i=1:3
        T(i,4)=-T(i,4);
    end
    Ti=T;
end

%如果全要自定义
% 在MATLAB中的syms推导中，最好使用Toolbox系统函数，把自定义的函数注释掉，否则对化简不利；
% note them to avoid infinitesimal;
function T=transl(x,y,z) %平移矩阵
    T=[1 0 0 x;
    0 1 0 y;
    0 0 1 z;
    0 0 0 1];
end
 %自定义函数，无法自动处理高阶无穷小
% function T=trotx(alpha) %旋转矩阵x
%     T=[ 1 0 0 0;
%         0 cos(alpha) -sin(alpha) 0;
%         0 sin(alpha) cos(alpha) 0;
%         0 0 0 1];
% end
function T=trotx(theta)
    T=[1 0 0 0;
        0 cos(theta) -sin(theta) 0;
        0 sin(theta) cos(theta) 0;
        0 0 0 1];
end
function T=trotz(theta)
    T=[cos(theta) -sin(theta) 0 0;
        sin(theta) cos(theta) 0 0;
        0 0 1 0;
        0 0 0 1];
end

function Ti=troty(theta)
    Ti=[cos(theta) 0 sin(theta) 0;
        0 1 0 0;
        -sin(theta) 0 cos(theta) 0;
        0 0 0 1];
end


%% pose trans
function T=pose2tr(P) %位姿表达形式转换
    k=sqrt(P(4)^2+P(5)^2+P(6)^2);
    if k==0 
        T=eye(4);  
    else    
        v=[P(4) P(5) P(6)]/k;
        T=angvec2tr(k,v);
    end
    T(1:3,4)=P(1:3);
end

function P=tr2pose(T)
    [theta,v]=tr2angvec(T);
    P(4:6)=theta*v;
    P(1:3)=T(1:3,4);
end

%% Standardized
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
% 
function k = K_one(k) 
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
function theta = In_pi(theta)
    while (abs(theta)>pi)
        if (theta>pi)
            theta=theta-2*pi;
        elseif (theta<-pi)
            theta=theta+2*pi;
        end
    end
end
