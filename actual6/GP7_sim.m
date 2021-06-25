%% initialize
clc; close all; clear all;
format shortg; format compact;

% main
theta_deg=[10 20 30 40 50 60]
% theta_deg=[90 90 90 90 90 90]
% theta_deg=[180*(rand*2-1) 180*(rand*2-1) 180*(rand*2-1) 180*(rand*2-1) 180*(rand*2-1) 180*(rand*2-1)]
theta=theta_deg/180*pi;
T=fkine_GP7(theta,6)
Q_deg=ikine_GP7(T)
th=Q_deg*pi/180;
for i=1:8
%     eval(['Q',num2str(i),'=','N_zero(forward_16(th(i,1:6))-T)']);
end

%% ikine
function theta_deg=ikine_GP7(T)
    d1=100;d4=-440;d6=-80;a1=40;a2=460;a3=40;
    d=[d1 0 0 d4 0 d6];    a=[a1 a2 a3 0 0 0];
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    
    % th1
    n1=N_zero(d6*ax+px);
    m1=N_zero(d6*ay+py);
    if m1==0 && n1==0 %Singularity¢Ù
        th1(1,1)=0;
        th1(1,2)=pi;
    else
        th1(1,1)=atan2(m1,n1);
        th1(1,2)=th1(1,1)-pi;
    end
    % th2
    m23=-a1+d6*(ax*cos(th1)+ay*sin(th1))+px*cos(th1)+py*sin(th1); 
    n23=-d6*az-(pz-d1);
    k2=(m23.^2+n23^2+a2^2-a3^2-d4^2)./(2*a2*sqrt(m23.^2+n23.^2));
%     k2=K_one(k2); ÎÞÐèÅÐ¶Ï
    th2(1,1:2)=atan2(k2,sqrt(1-k2.^2))-atan2(m23,n23); 
    th2(2,1:2)=atan2(k2,-sqrt(1-k2.^2))-atan2(m23,n23);
    % th3
    k3=(m23.^2+n23^2-a2^2-a3^2-d4^2)/(2*a2*sqrt(a3^2+d4^2));
%     k3=K_one(k3); 
    th3(1,1:2)=atan2(a3,d4)-atan2(k3,-sqrt(1-k3.^2));
    th3(2,1:2)=atan2(a3,d4)-atan2(k3,sqrt(1-k3.^2));
    %----------------------------------------------------------------------
    % th6
    m6=tan(th2-th3).*(ox*cos(th1)+oy*sin(th1))+oz;  
    n6=tan(th2-th3).*(nx*cos(th1)+ny*sin(th1))+nz;  
    m6=N_zero(m6);
    n6=N_zero(n6);
    th6(1:2,1:2)=atan2(m6,n6);
    th6(3:4,1:2)=th6(1:2,1:2)-pi;
    % if sin(th5)==0;%Singularity¢Ú
    if m6(1,1)==0&&n6(1,1)==0 
        th6(1:2:3,1)=0;
    end
    % th5
    th3(3:4,1:2)=th3(1:2,1:2);
    th2(3:4,1:2)=th2(1:2,1:2);
    m5=(sin(th6).*(ox*cos(th1)+oy*sin(th1))+cos(th6).*(nx*cos(th1)+ny*sin(th1))).*sin(th2-th3)+(nz*cos(th6)+oz*sin(th6)).*cos(th2-th3);
    n5=(-ax*cos(th1)-ay*sin(th1)).*sin(th2-th3)+(-az)*cos(th2-th3);
    th5=atan2(m5,n5); 
    % th4
    n4=sin(th6).*(ny*cos(th1)-nx*sin(th1))-cos(th6).*(oy*cos(th1)-ox*sin(th1));
    n4=N_zero(n4);
    th23=N_zero(th2-th3);
    if th23(1,1)==0||th23(2,1)==0%when sin(th2-th3)==0,using equation 6;
        m4=(cos(th6).*(ox*cos(th1)+oy*sin(th1))-sin(th6).*(nx*cos(th1)+ny*sin(th1)))./(cos(th2-th3));
    else
        m4=(nz*sin(th6)-oz*cos(th6))./(sin(th2-th3));
    end
    th4=atan2(m4,n4);
    % sort out;------------------------------------------------------------
    for i=1:2 
        for j=1:4
            th(4*(i-1)+j,1)=In_pi(th1(1,i)); %th1;
            th(4*(i-1)+j,2)=In_pi(th2(j,i)); %th2;
            th(4*(i-1)+j,3)=In_pi(th3(j,i)); %th3;
            th(4*(i-1)+j,6)=In_pi(th6(j,i)); %th6;
            th(4*(i-1)+j,5)=In_pi(th5(j,i)); %th5;
            th(4*(i-1)+j,4)=In_pi(th4(j,i)); %th4;
        end
    end
    theta_deg=th*180/pi;
end

%% function1
function T = fkine_GP7(theta,n)
    d1=100;d4=-440;d6=-80;a1=40;a2=460;a3=40;
    d=[d1 0 0 d4 0 d6];    a=[a1 a2 a3 0 0 0];
    alpha=[-pi/2 pi -pi/2 pi/2 -pi/2 pi];
    for i=1:6
      Te(:,:,i)=DH_forward(theta(i),d(i),a(i),alpha(i));
    end
    T=eye(4); 
    for j=1:n
        T=T*Te(:,:,j);
    end 
end

function T = DH_forward(theta,d,a,alpha)
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end

%------------------------------------------------
function mn = N_zero(mn)
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<2E-6
                mn(i,j)=0;
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

