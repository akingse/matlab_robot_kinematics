%% initialize
clc; close all; clear all;
format shortg; format compact;

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
8自由度模型的求解
重新建模，同样分为Actual6和Virtual6；Actual6的alpha6不同，Virtual6的关节编号不同；
由于alpha6不同，在求逆解的时候，可以为T_A6右乘一个inv(rotx(pi/2)),也可以单独写一个函数；
原六轴：T_A=T1*T2*T3*T4*T5*T60*trotx(pi/2)
新六轴：T_A=T1*T2*T3*T4*T5*T6

本脚本仅为alpha6的实体六轴求解，进行符号推导；

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

%% DOF8 model 
theta=[0 0 0 0 0 0 0 0]; 
d=[90 0 0 90 90 800 0 0]; 
a=[0 -420 -400 0 0 0 50 50];
alpha=[pi/2 0 0 pi/2 -pi/2 pi/2 -pi/2 0]; 
offset=[0 0 0 0 0 0 0 0];

sigma=0;mdh=0;  %旋转0移动1 %标准0改进1
% Li = Link([theta(i) d(i) a(i) alpha(i) sigma offset(i)],mdh);
L(1) = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
L(2) = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
L(3) = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
L(4) = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
L(5) = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
L(6) = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);
L(7) = Link([theta(7) d(7) a(7) alpha(7) sigma offset(7)],mdh);
L(8) = Link([theta(8) d(8) a(8) alpha(8) sigma offset(8)],mdh);

robot=SerialLink([L(1) L(2) L(3) L(4) L(5) L(6) L(7) L(8)],'name','DOF8');%   
% robot.display();
% figure('NumberTitle', 'off', 'Name', 'DOF8');
% robot.teach(); 
% % 设定初始化位姿
% theta=[0 -60 120 -150 -90 0 90 0]/180*pi; 
robot.teach(theta); 


%% 联立等式组
% Actual alpha6 version,Symbolic derivation
syms th1 th2 th3 th4 th5 th6;
theta=[th1 th2 th3 th4 th5 th6]
T=forward_kine_sym(theta);

% 为了获取逆解等式组，令 T2*T3*T4*T5==T1i*T_end*T6i
T2345=forward_kine_25(theta);
% 正负号不要动
%[cos(th2+th3+th4)*cos(th5),-sin(th2+th3+th4),-cos(th2+th3+th4)*sin(th5),a3*cos(th2+th3)+a2*cos(th2)+d5*sin(th2+th3+th4)]
%[sin(th2+th3+th4)*cos(th5),cos(th2+th3+th4),-sin(th2+th3+th4)*sin(th5),a3*sin(th2+th3)+a2*sin(th2)-d5*cos(th2+th3+th4)]
%[sin(th5),0,cos(th5),d4]

T1q6=forward_kine_1q6(theta);
%[sin(th6)*(ax*cos(th1)+ay*sin(th1))+cos(th6)*(nx*cos(th1)+ny*sin(th1)),sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ax*cos(th1)+ay*sin(th1)),ox*cos(th1)+oy*sin(th1),px*cos(th1)-d6*(ox*cos(th1)+oy*sin(th1))+py*sin(th1)]
%[nz*cos(th6)+az*sin(th6),nz*sin(th6)-az*cos(th6),oz,pz-d1-d6*oz]
%[-sin(th6)*(ay*cos(th1)-ax*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1)),cos(th6)*(ay*cos(th1)-ax*sin(th1))-sin(th6)*(ny*cos(th1)-nx*sin(th1)),ox*sin(th1)-oy*cos(th1),d6*(oy*cos(th1)-ox*sin(th1))-py*cos(th1)+px*sin(th1)]


%{
% cos(th2+th3+th4)*cos(th5)=sin(th6)*(ax*cos(th1)+ay*sin(th1))+cos(th6)*(nx*cos(th1)+ny*sin(th1))
% sin(th2+th3+th4)*cos(th5)=nz*cos(th6)+az*sin(th6)
                   sin(th5)=-sin(th6)*(ay*cos(th1)-ax*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1))  ①

-sin(th2+th3+th4)=sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ax*cos(th1)+ay*sin(th1))  ②
 cos(th2+th3+th4)=nz*sin(th6)-az*cos(th6)                                                ③
                0=cos(th6)*(ay*cos(th1)-ax*sin(th1))-sin(th6)*(ny*cos(th1)-nx*sin(th1))  ④

% -cos(th2+th3+th4)*sin(th5)=ox*cos(th1)+oy*sin(th1)  
% -sin(th2+th3+th4)*sin(th5)=oz                       
                  cos(th5)=ox*sin(th1)-oy*cos(th1)  ⑤

a2*cos(th2)+a3*cos(th2+th3)+d5*sin(th2+th3+th4)=px*cos(th1)-d6*(ox*cos(th1)+oy*sin(th1))+py*sin(th1)  ⑥
a2*sin(th2)+a3*sin(th2+th3)-d5*cos(th2+th3+th4)=pz-d1-d6*oz                                           ⑦
                                             d4=d6*(oy*cos(th1)-ox*sin(th1))-py*cos(th1)+px*sin(th1)  ⑧
---------------------------------------------------------------------------
th1
d4=d6*(oy*cos(th1)-ox*sin(th1))-py*cos(th1)+px*sin(th1)  ⑧

th6
0=cos(th6)*(ay*cos(th1)-ax*sin(th1))-sin(th6)*(ny*cos(th1)-nx*sin(th1))  ④

th5
sin(th5)=-sin(th6)*(ay*cos(th1)-ax*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1))  ①
cos(th5)=ox*sin(th1)-oy*cos(th1)  ⑤


th2 th3 th4
-sin(th2+th3+th4)=sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ax*cos(th1)+ay*sin(th1))  ②
 cos(th2+th3+th4)=nz*sin(th6)-az*cos(th6)                                                ③
a2*cos(th2)+a3*cos(th2+th3)+d5*sin(th2+th3+th4)=px*cos(th1)-d6*(ox*cos(th1)+oy*sin(th1))+py*sin(th1)  ⑥
a2*sin(th2)+a3*sin(th2+th3)-d5*cos(th2+th3+th4)=pz-d1-d6*oz                                           ⑦

%}
%% main
q_deg=[10 20 30 40 50 60]
q=q_deg/180*pi;
T = forward_kine(q)
Q=ikine_Actual_a(T);
Q_deg=Q*180/pi
for i=1:8
    T8=forward_kine(Q(i,1:6))-T;
    T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4))
end


%% alpha6版，逆解函数
function th=ikine_Actual_a(T) %注意，d6为800
    d1=90;d4=90;d5=90;d6=800;a2=-420;a3=-400;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);

% th1
% d4=d6*(oy*cos(th1)-ox*sin(th1))-py*cos(th1)+px*sin(th1)  ⑧
% d4=(d6*oy-py)*cos(th1)-(d6*ox-px)*sin(th1)
% d4/r1=cos(phi1+th1)
    m1=d6*ox-px;
    n1=d6*oy-py;
    k1=d4/sqrt(m1^2+n1^2);
    if (k1<=1) %工作空间判定
            th1(1)=acos(k1)-atan2(m1,n1);
            th1(2)=-acos(k1)-atan2(m1,n1);
    else
        th1(1:2)=NaN;
    end

% th6
% 0=cos(th6)*(ay*cos(th1)-ax*sin(th1))-sin(th6)*(ny*cos(th1)-nx*sin(th1))  ③
% tan(th6)=sin(th6)/cos(th6)=(ay*cos(th1)-ax*sin(th1))/(ny*cos(th1)-nx*sin(th1))
    m6=ay*cos(th1)-ax*sin(th1);  m6=N_zero(m6);
    n6=ny*cos(th1)-nx*sin(th1);  n6=N_zero(n6);
    th6(1:2)=atan(m6./n6);
%     if  (m6(1)==0 && n6(1)==0)
%         th6(1)=0; %奇异判定1
%     end
%     if  (m6(2)==0 && n6(2)==0)
%         th6(2)=0;
%     end
    th6(3:4)=th6(1:2)+pi;

% th5
% cos(th5)=ox*sin(th1)-oy*cos(th1)  ④
% sin(th5)=-sin(th6)*(ay*cos(th1)-ax*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1)) ①
    th1(3:4)=th1(1:2);
    m5=-sin(th6).*(ay*cos(th1)-ax*sin(th1))-cos(th6).*(ny*cos(th1)-nx*sin(th1));
    n5=ox*sin(th1)-oy*cos(th1);
    th5=atan2(m5,n5);


% -------------------------------------------------------------------------
% th2 th3 th4
% -sin(th2+th3+th4)=sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ax*cos(th1)+ay*sin(th1))  ①
%  cos(th2+th3+th4)=nz*sin(th6)-az*cos(th6)                                                ②
% a2*cos(th2)+a3*cos(th2+th3)+d5*sin(th2+th3+th4)=px*cos(th1)-d6*(ox*cos(th1)+oy*sin(th1))+py*sin(th1)  ⑤
% a2*sin(th2)+a3*sin(th2+th3)-d5*cos(th2+th3+th4)=pz-d1-d6*oz                                           ⑥

% a2*cos(th2)+a3*cos(th2+th3)=px*cos(th1)-d6*(ox*cos(th1)+oy*sin(th1))+py*sin(th1)+d5*(sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ax*cos(th1)+ay*sin(th1)))
% a2*sin(th2)+a3*sin(th2+th3)=pz-d1-d6*oz+d5*(nz*sin(th6)-az*cos(th6))
%                           m=px*cos(th1)+py*sin(th1)-d6*(ox*cos(th1)+oy*sin(th1))+d5*(sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ax*cos(th1)+ay*sin(th1)))
%                           n=pz-d1-d6*oz+d5*(nz*sin(th6)-az*cos(th6))
    
    m=px*cos(th1)+py*sin(th1)-d6*(ox*cos(th1)+oy*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))-cos(th6).*(ax*cos(th1)+ay*sin(th1)));
    n=pz-d1-d6*oz+d5*(nz*sin(th6)-az*cos(th6));
    mn=m.^2+n.^2; 
    for i=1:4
        if (a2-a3)^2<=mn(i) && mn(i)<=(a2+a3)^2
            k2=(m(i)^2+n(i)^2+a2^2-a3^2)/sqrt((2*a2*m(i))^2+(2*a2*n(i))^2);
            th2(i)=atan2(n(i)*a2,m(i)*a2)-acos(k2);
            th2(i+4)=atan2(n(i)*a2,m(i)*a2)+acos(k2);
            k3=(m(i)^2+n(i)^2-a2^2-a3^2)/(2*a2*a3);
            th3(i)=acos(k3);
            th3(i+4)=-th3(i);
        else
            th2(i)=NaN;
            th2(i+4)=NaN;
            th3(i)=NaN;
            th3(i+4)=NaN;
        end
    end
% th4 
    m234=-sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ax*cos(th1)+ay*sin(th1));
    n234=nz*sin(th6)-az*cos(th6);
    th234=atan2(m234,n234);
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



%% function
function T = forward_kine(theta)% 正运动
    d1=90;d4=90;d5=90;d6=800;a2=-420;a3=-400;
    d=[d1 0 0 d4 d5 d6];
    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 pi/2]; %alpha6
    
    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T=T1*T2*T3*T4*T5*T6;
end

function T = forward_kine_sym(theta)% 正运动
    syms d1 d4 d5 d6 a2 a3; 
    d=[d1 0 0 d4 d5 d6];
    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 pi/2]; %alpha6
    
    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T=simplify(T1*T2*T3*T4*T5*T6);
end

function T = forward_kine_25(theta)% 
    syms d1 d4 d5 d6 a2 a3; 
    d=[d1 0 0 d4 d5 d6];
    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 pi/2]; %alpha6

    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T=simplify(T2*T3*T4*T5);
end

function T = forward_kine_1q6(theta)%
    global py;
    syms d1 d4 d5 d6 a2 a3 nx ox ax px ny oy ay py nz oz az pz;
    T_end=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];
    syms d1 d4 d5 d6 a2 a3; 
    d=[d1 0 0 d4 d5 d6];
    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 pi/2]; %alpha6

    T1i=DH_inverse(theta(1),d(1),a(1),alpha(1)); 
    T6i=DH_inverse(theta(6),d(6),a(6),alpha(6));
    T=simplify(T1i*T_end*T6i);
end

% ---------------------------------------------------------------------
function T = DH_forward(theta,d,a,alpha) % 正运动函数
% T=transZ(0,0,d)*trotZ(theta)*transX(a,0,0)*trotX(alpha);
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end
function T = DH_inverse(theta,d,a,alpha) % 逆运动函数
    T=invtrot(trotx(alpha))*invtrans(transl(a,0,0))*invtrot(trotz(theta))*invtrans(transl(0,0,d));
end

function Ti=invtrot(T) %旋转矩阵，逆矩阵
    for i=1:2
        for j=i+1:3
            temp=T(i,j);
            T(i,j)=T(j,i);
            T(j,i)=temp;
        end
    end
    Ti=T;
end
function Ti=invtrans(T) %平移矩阵，逆矩阵
    for i=1:3
        T(i,4)=-T(i,4);
    end
    Ti=T;
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
function theta = In_pi(theta)
    while (abs(theta)>pi)
        if (theta>pi)
            theta=theta-2*pi;
        elseif (theta<-pi)
            theta=theta+2*pi;
        end
    end
end

