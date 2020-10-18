%% initialize
clc; close all; clear all;
format shortg; format compact;
%% main Reality6
% q_deg=[0 -90 0 -90 0 0];
q_deg=[10 20 30 40 50 60]
q=q_deg/180*pi;
q=[pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1)];
T=fkine_UR5(q,6)
Q=ikine_UR5(T);
Q_deg=Q*180/pi %print()
for i=1:8
%     eval(['Q',num2str(i),'=','fkine_UR5(Q(i,1:6),6)-T']);
end

%% function1
function th=ikine_UR5(T)
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    %d1=127.3;d4=164;d5=116;d6=92;a2=-612;a3=-573; %UR10
    
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
        %奇异点 singularity 预自定义
    sing_th1=0;
    sing_th5=0;
    sing_th6=0;
    
%% th1
    m1=d6*ay-py;
    n1=d6*ax-px;
    if (d4^2<=m1^2+n1^2) %工作空间，盲区判定
%       if m1==0&&n1=0 奇异判定
%         th1(1:2)=sing_th1;
%         k1=d4/sqrt(m1^2+n1^2);
%         th1(1)=acos(-k1)-atan2(m1,n1);
%         th1(2)=-acos(-k1)-atan2(m1,n1);
        
        k1=d4/sqrt(m1^2+n1^2);
        th1(1)=atan2(m1,n1)-atan2(k1,sqrt(1-k1^2));
        th1(2)=atan2(m1,n1)-atan2(k1,-sqrt(1-k1^2));
    else
        th1(1:2)=NaN;
    end

    
%% th5 无奇异
    k5=ax*sin(th1)-ay*cos(th1);
    th5(1:2)=acos(k5); %尽量用arccos()，默认为正，加负号即可。
    th5(3:4)=-th5(1:2);
   
%% th6
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
%{
    % th6    
    m6=oy*cos(th1)-ox*sin(th1);
    n6=ny*cos(th1)-nx*sin(th1);
    n6=N_zero(n6);
    m6=N_zero(m6);
    
%     th6(1:2)=atan(-m6./n6); %atan强行限定(-pi/2,pi/2)内，不适合使用。否则数据需要重新排序
%     th6(3:4)=th6(1:2)+pi;
    th6(1:2)=atan2(m6,-n6);%atan2(-m6,n6)+pi
    th6(3:4)=th6(1:2)+pi;
%     if m6(1)==0 && n6(1)==0 %Singularity①
%         th6(1)=0; %or th6=read(theta_6);
%         th6(2)=atan2(m6(2),-n6(2)); % ?
%     else
%         th6(1:2)=atan2(m6,-n6);
%         th6(3:4)=th6(1:2)-pi;
%}
    
%% th3 th2
    th1(3:4)=th1(1:2);
    m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)));
    n23=pz-d1-d6*az+d5*(nz*sin(th6)+oz*cos(th6)); 
%     m23=N_zero(m23);
%     n23=N_zero(n23);
% if (a2^2-a3^2)^2<=m23.^2+n23.^2 && m23.^2+n23.^2<=(a2^2-a3^2)^2
% end

    k3=(m23.^2+n23.^2-a2^2-a3^2)/(2*a2*a3);
%     k3=K_one(k3); 无奇异
    th3(1:4)=acos(k3);
    th3(5:8)=-th3(1:4);
    
    k2=(m23.^2+n23.^2+a2^2-a3^2)./sqrt((2*a2*m23).^2+(2*a2*n23).^2);
%     k2=K_one(k2); 无奇异
    th2(1:4)=atan2(2*n23*a2,2*m23*a2)-acos(k2);
    th2(5:8)=atan2(2*n23*a2,2*m23*a2)+acos(k2);
%% th4 无奇异
    m234=-sin(th6).*(nx*cos(th1)+ny*sin(th1))-cos(th6).*(ox*cos(th1)+oy*sin(th1));
    n234=nz*sin(th6)+oz*cos(th6);
    m234=N_zero(m234);
    n234=N_zero(n234);
    th234(1:4)=atan2(m234,n234);
    th234(5:8)=th234(1:4);
    th4=th234-th3-th2;
    
%% sort out;
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
%% Function functions
function T = fkine_UR5(theta,n) %forward_kine()
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    %d1=127.3;d4=164;d5=116;d6=92;a2=-612;a3=-573; %UR10
    d=[d1 0 0 d4 d5 d6];    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 0];
    for i=1:6
      Te(1:4,1:4,i)=DH_forward(theta(i),d(i),a(i),alpha(i));
    end
    T=eye(4); 
    for j=1:n
        T=T*Te(1:4,1:4,j);
    end 
end

function T = DH_forward(theta,d,a,alpha)
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end
%
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

% function T=transl(x,y,z) % note them to avoid infinitesimal;
%   T=[1 0 0 x;0 1 0 y;0 0 1 z;0 0 0 1];
% end
% function T=trotx(theta)
%   T=[1 0 0 0;0 cos(theta) -sin(theta) 0;0 sin(theta) cos(theta) 0;0 0 0 1];
% end
% function T=trotz(theta)
%   T=[cos(theta) -sin(theta) 0 0;sin(theta) cos(theta) 0 0;0 0 1 0;0 0 0 1];
% end
%% function1 
% 关节角二维数组版本，其实一维数组就可以了
function th=ikine_UR5_2x2(T)
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    % th1
    m1=d6*ay-py;
    n1=d6*ax-px;
    if (d4^2<=m1^2+n1^2)
        k1=d4/sqrt(m1^2+n1^2);
        th1(1,1)=atan2(m1,n1)-atan2(k1,sqrt(1-k1^2));
        th1(1,2)=atan2(m1,n1)-atan2(k1,-sqrt(1-k1^2));
    else
        th1(1,1:2)=NaN;
    end
    % th5
    k5=ax*sin(th1)-ay*cos(th1);
    th5(1,1:2)=acos(k5);
    th5(2,1:2)=-th5(1,1:2);
    % th6
    n6=ny*cos(th1)-nx*sin(th1);
    m6=oy*cos(th1)-ox*sin(th1);
    n6=N_zero(n6);
    m6=N_zero(m6);
    if m6(1,1)==0 && n6(1,1)==0 %Singularity①
        th6(1:2,1)=0; %or th6=read(theta_6);
        th6(1,2)=atan2(m6(1,2),-n6(1,2));
        th6(2,2)=th6(1,2)-pi;
    else
        th6(1,1:2)=atan2(m6,-n6);
        th6(2,1:2)=th6(1,1:2)-pi;
    end
%     th1(2,1:2)=th1(1,1:2);
    m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)));
    n23=pz-d1-d6*az+d5*(nz*sin(th6)+oz*cos(th6)); 
    m23=N_zero(m23);
    n23=N_zero(n23);
    % th3
    k3=(m23.^2+n23.^2-a2^2-a3^2)/(2*a2*a3);
    k3=K_one(k3);
    th3(1:2,1:2)=atan2(sqrt(1-k3.^2),k3);
    th3(3:4,1:2)=-th3(1:2,1:2);
    % th2
    k2=(m23.^2+n23.^2+a2^2-a3^2)./sqrt((2*a2*m23).^2+(2*a2*n23).^2);
    k2=K_one(k2);
    th2(1:2,1:2)=atan2(k2,sqrt(1-k2.^2))-atan2(2*a2*m23,2*a2*n23);
    th2(3:4,1:2)=atan2(k2,-sqrt(1-k2.^2))-atan2(2*a2*m23,2*a2*n23);
    % th4
    m234=-sin(th6).*(nx*cos(th1)+ny*sin(th1))-cos(th6).*(ox*cos(th1)+oy*sin(th1));
    n234=nz*sin(th6)+oz*cos(th6);
    m234=N_zero(m234);
    n234=N_zero(n234);
    th234(1:2,1:2)=atan2(m234,n234);
    th234(3:4,1:2)=th234(1:2,1:2);
    th4=th234-th3-th2;
    % sort out;
    th5(3:4,1:2)=th5(1:2,1:2);
    th6(3:4,1:2)=th6(1:2,1:2);
    for i=1:2 
        for j=1:4
            th(4*(i-1)+j,1)=In_pi(th1(1,i)); %th1;
            th(4*(i-1)+j,2)=In_pi(th2(j,i)); %th2;
            th(4*(i-1)+j,3)=N_zero(th3(j,i)); %th3;
            th(4*(i-1)+j,4)=In_pi(th4(j,i)); %th4;
            th(4*(i-1)+j,5)=N_zero(th5(j,i)); %th5;
            th(4*(i-1)+j,6)=In_pi(th6(j,i)); %th6;
        end
    end
end

