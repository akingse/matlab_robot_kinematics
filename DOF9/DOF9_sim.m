%% initialize
clc; close all; clear all;
format shortg; format compact;
%2020_09版 文本代码
%% model
theta=[0 0 0 0 0 0 0 0 0]; %绕Z 
d=[90 0 0 90 90 90 0 0 0]; %沿Z 
a=[0 -420 -400 0 0 0 0 50 50]; %沿X  % a=[0 420 400 0 0 0]; 
alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0]; %绕X
offset=[0 0 0 0 0 0 0 0 0]; %初始偏移角%offset=[0 -pi/2 0 -pi/2 0 0 0 pi/2 0]; bug找到了
sigma=0; %旋转0移动1
mdh=0; %标准0改进1
% Li = Link([theta(i) d(i) a(i) alpha(i) sigma offset(i)],mdh);
L(1) = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
L(2) = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
L(3) = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
L(4) = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
L(5) = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
L(6) = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);
L(7) = Link([theta(7) d(7) a(7) alpha(7) 1 offset(7)], mdh);  L(7).qlim = [0 1000];
L(8) = Link([theta(8) d(8) a(8) alpha(8) sigma offset(8)],mdh); 
L(9) = Link([theta(9) d(9) a(9) alpha(9) sigma offset(9)],mdh);
robot_dof9=SerialLink(L,'name','DOF9');%   
% robot_dof9.display();% figure('NumberTitle', 'off', 'Name', 'DOF9');
% robot_dof9.teach(); 
% syms rx ry rz xi yi zi; %旋转矢量，远心点位置


%% 获取设定正向的末端位姿
% syms th1 th2 th3 th4 th5 th6 th8 th9 d7; % syms d1 d4 d5 d6 a2 a3 a8 a9;
% theta=[th1 th2 th3 th4 th5 th6 d7 th8 th9]; %关节轴变量；
% T=forward_kine9(theta); simplify(T)
% Tmain=equation(theta)

theta_rand=[2*rand(1)-1 2*rand(1)-1 2*rand(1)-1 pi*(2*rand(1)-1) pi*(2*rand(1)-1)/2 pi*(2*rand(1)-1) 500*(1+rand(1)) pi*(2*rand(1)-1) pi*(2*rand(1)-1)]
% theta_rand=[-0.93923     -0.58306    -0.090068       -2.342      -1.5436       1.4268       677.06       1.7621       -0.398]

T_RCM=transl(-600+theta_rand(1),theta_rand(2),100+theta_rand(3)) %指定RCM远心点位置 %限定范围的，随机RCM远心点
T_AV=trotx(pi/2); %DOF9到Virtual6之间的基坐标系变换；
T_49=forward_kine49(theta_rand); %T_49==T_V6
T_end=T_RCM*T_AV*T_49  %获取DOF9末端任意位姿
% T_tcp %工具中心点 的英文名称为“Tool Central Point”，简写为“TCP”

%% 求逆3+6
T=invtrot(T_AV)*invtrans(T_RCM)*T_end;  %T49
thv=ikine_Virtual3(T) %theta_virtual

% %% 求逆6+6
% T=invtrot(T_AV)*invtrans(T_RCM)*T_end;  %T49
% thv=ikine_Virtual6(T) %theta_virtual
 
% T=T_end*inv(T9)*inv(T8)*inv(T7);
T1=T_end*inverse_kine987(thv(1,1:9));   %T16
T2=T_end*inverse_kine987(thv(2,1:9)); %从virtual分别获得两组，分别对应actual4组，实际上有4*8组，这里做了条件筛选；
tha1=ikine_Actual6(T1);%theta_actual
tha2=ikine_Actual6(T2);
    theta(1:4,1:6)=tha1;
    theta(5:8,1:6)=tha2;
    for i=1:4 % 区别于变量名整体赋值，索引逐个赋值
        theta(i,7:9)=thv(1,7:9);
        theta(4+i,7:9)=thv(2,7:9);
    end
%     theta_deg=theta*180/pi;
    theta_deg(1:8,1:6)=theta(:,1:6)*180/pi;
    theta_deg(1:8,7)=theta(:,7);
    theta_deg(1:8,8:9)=theta(:,8:9)*180/pi;
    theta_deg
%     a = theta_deg;
%     n = 3; %保留位数
%     b = a*10^n;% 先化成整数，
%     a = round(b); %s四舍五入
%     need_num = a/10^n; %再转成小数
%     need_str = num2str(need_num)
%    
%% 验证
for i=1:4
%     eval(['Q',num2str(i),'=','vpa(forward_kine49(theta(i,1:9))-T,6)']);
    T8=forward_kine16(theta(i,1:6))-T_RCM;
    T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4));
    equ(i)=T8p;
%     eval(['Q',num2str(i),'=','T8p']); 
end
    equ
for i=1:8
%     eval(['Q',num2str(i),'=','vpa(forward_kine49(theta(i,1:9))-T,6)']);
    T8=forward_kine19(theta(i,1:9))-T_end;
    T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4));
%     eval(['Q',num2str(i),'=','T8p']); 
    equ(i)=T8p;
end
    equ

% robot_dof9.display();% figure('NumberTitle', 'off', 'Name', 'DOF9');
% robot_dof9.teach(theta(8,1:9)); 

%% ikine_Virtual3(T)
function th=ikine_Virtual3(T)
    a8=50;a9=50;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    %th9
    Px=px-a9*nx;
    Py=py-a9*ny;
    Pz=pz-a9*nz;
    Pm=(Pz*ay-Py*az)*nx+(Px*az-Pz*ax)*ny+(Py*ax-Px*ay)*nz;    Pm=N_zero(Pm);
    Pn=(Pz*ay-Py*az)*ox+(Px*az-Pz*ax)*oy+(Py*ax-Px*ay)*oz;    Pn=N_zero(Pn);
    th9(1)=atan(Pm/Pn);
    th9(2)=th9(1)+pi;
	%th8
    Rx=nx*cos(th9)-ox*sin(th9); 
    Ry=ny*cos(th9)-oy*sin(th9); 
    Rz=nz*cos(th9)-oz*sin(th9);  
%     m81=ax*(Pz-a8*Rz)-az*(Px-a8*Rx);
%     n81=Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz);
%     m81=N_zero(m81);
    m82=ay*(Pz-a8*Rz)-az*(Py-a8*Ry);
    n82=Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz);
    m82=N_zero(m82);
    th8(1:2)=atan(m82./n82);
    th8(3:4)=th8(1:2)+pi;
    th9(3:4)=th9(1:2);
    th8=In_pi(th8);
    %d7
    Mz=az*cos(th8) + sin(th8).*(nz*cos(th9)-oz*sin(th9));
    Nz=pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9));
    Mz=N_zero(Mz);
    Nz=N_zero(Nz);
    d7(1:4)=Nz./Mz;
    
    i=1; %取d7大于0的解
    for j=1:4
        if d7(j)>0
            th(i,9)=In_pi(th9(j));
            th(i,8)=In_pi(th8(j));
            th(i,7)=d7(j);
            i=i+1;
        end
    end
end


%% ikine_Virtual6(T)
function th=ikine_Virtual6(T)
    a8=50;a9=50;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    %th9
    Px=px-a9*nx;
    Py=py-a9*ny;
    Pz=pz-a9*nz;
    Pm=(Pz*ay-Py*az)*nx+(Px*az-Pz*ax)*ny+(Py*ax-Px*ay)*nz;
    Pn=(Pz*ay-Py*az)*ox+(Px*az-Pz*ax)*oy+(Py*ax-Px*ay)*oz;
    Pm=N_zero(Pm);
    Pn=N_zero(Pn);
    th9(1)=atan(Pm/Pn);
    th9(2)=th9(1)+pi;
	%th8
    Rx=nx*cos(th9)-ox*sin(th9); 
    Ry=ny*cos(th9)-oy*sin(th9); 
    Rz=nz*cos(th9)-oz*sin(th9);  
%     m81=ax*(Pz-a8*Rz)-az*(Px-a8*Rx);
%     n81=Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz);
%     m81=N_zero(m81);
    m82=ay*(Pz-a8*Rz)-az*(Py-a8*Ry);
    n82=Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz);
    m82=N_zero(m82);
    th8(1:2)=atan(m82./n82);
    th8(3:4)=th8(1:2)+pi;
    th9(3:4)=th9(1:2);
    th8=In_pi(th8);
    %d7
    Mz=az*cos(th8) + sin(th8).*(nz*cos(th9)-oz*sin(th9));
    Nz=pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9));
    Mz=N_zero(Mz);
    Nz=N_zero(Nz);
    d7(1:4)=Nz./Mz;
    %th5
    n5=az*cos(th8) + sin(th8).*(nz*cos(th9)-oz*sin(th9));
    th5(1:4)=acos(n5);
    th5(5:8)=-th5(1:4);
    %th4
    th9(5:8)=th9(1:4);
    th8(5:8)=th8(1:4);
    s=sign(sin(th5));
    n4=-ax*cos(th8)-sin(th8).*(nx*cos(th9)-ox*sin(th9));
    m4=-ay*cos(th8)-sin(th8).*(ny*cos(th9)-oy*sin(th9));
    th4=atan2(m4./s,n4./s);
    %th6
    n6=-az*sin(th8)+cos(th8).*(nz*cos(th9)-oz*sin(th9));
    m6=-oz*cos(th9)-nz*sin(th9);
    th6=atan2(m6./s,n6./s);
    
    i=1; %取d7大于0的解
    for j=1:4
        if d7(j)>0
            th(i,9)=In_pi(th9(j));
            th(i,8)=In_pi(th8(j));
            th(i,7)=d7(j);
            th(i,5)=th5(j);
            th(i,4)=th4(j);
            th(i,6)=th6(j);
            i=i+1;
        end
    end
end

%% ikine_Actual6(T)
function th=ikine_Actual6(T)
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    
% th1
    m1=px-d6*ax;
    n1=py-d6*ay;
    if (d4^2<=m1^2+n1^2) %th1盲区判定
        k1=d4/sqrt(m1^2+n1^2);
        th1(1)=acos(-k1)-atan2(m1,n1);
        th1(2)=-acos(-k1)-atan2(m1,n1);
    else
        th1(1:2)=NaN;
    end
    % th5
    k5=ax*sin(th1)-ay*cos(th1);
    th5(1:2)=acos(k5); 
    th5(3:4)=-th5(1:2);
    % th6    
    m6=oy*cos(th1)-ox*sin(th1);
    n6=ny*cos(th1)-nx*sin(th1);
    n6=N_zero(n6);
    m6=N_zero(m6);
    th6(1:2)=atan2(m6,-n6);%atan2(-m6,n6)+pi
    th6(3:4)=th6(1:2)+pi;

    % th23
    th1(3:4)=th1(1:2);
    m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)));
    n23=pz-d1-d6*az+d5*(nz*sin(th6)+oz*cos(th6)); 
    % th2
    k2=(m23.^2+n23.^2+a2^2-a3^2)./sqrt((2*a2*m23).^2+(2*a2*n23).^2);
    th2(1:4)=atan2(n23*a2,m23*a2)-acos(k2);
    th2(5:8)=atan2(n23*a2,m23*a2)+acos(k2);
    % th3
    k3=(m23.^2+n23.^2-a2^2-a3^2)/(2*a2*a3);
    th3(1:4)=acos(k3);
    th3(5:8)=-th3(1:4);
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

    i=1;%th2取小于0的部分，向上。
    for j=1:8
        if In_pi(th2(j))<0
            th(i,1)=In_pi(th1(j));
            th(i,2)=In_pi(th2(j));
            th(i,3)=th3(j);
            th(i,4)=In_pi(th4(j));
            th(i,5)=th5(j);
            th(i,6)=In_pi(th6(j));
            i=i+1;
        end
    end
end
  

%% forward9  
% 9轴部分和虚拟六轴模型不一样，需要单独设定
function T = forward_kine19(theta)% 九轴
%     syms d1 d4 d5 d6 a2 a3 a8 a9 d7;
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
    T7=DH_forward(0,theta(7),a(7),alpha(7)); 
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T1*T2*T3*T4*T5*T6*T7*T8*T9;
end
function T = forward_kine16(theta) %前实体六轴
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

function T = forward_kine49(theta)% 虚拟六轴
    a8=50;a9=50; %d7=0;
    d=[0 0 0 0 0 0 0 0 0];      
    a=[0 0 0 0 0 0 0 a8 a9]; 
    alpha=[0 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(0,theta(7),a(7),alpha(7));
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T4*T5*T6*T7*T8*T9;
end


function T = inverse_kine987(theta)
    d=[90 0 0 90 90 90 0 0 0]; 
    a=[0 -420 -400 0 0 0 0 50 50];
    alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
    T9i=DH_inverse(theta(9),d(9),a(9),alpha(9));
    T8i=DH_inverse(theta(8),d(8),a(8),alpha(8));
    T7i=DH_inverse(0,theta(7),a(7),alpha(7));
    T=T9i*T8i*T7i;
end 

%% function
function T = DH_forward(theta,d,a,alpha) % 正运动函数
% T=transZ(0,0,d)*trotZ(theta)*transX(a,0,0)*trotX(alpha);
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end
function T = DH_inverse(theta,d,a,alpha) % 逆运动函数
    T=invtrot(trotx(alpha))*invtrans(transl(a,0,0))*invtrot(trotz(theta))*invtrans(transl(0,0,d));
end

function [Ti]=invtrot(T) %旋转矩阵，逆矩阵
    for i=1:2
        for j=i+1:3
            temp=T(i,j);
            T(i,j)=T(j,i);
            T(j,i)=temp;
        end
    end
    Ti=T;
end
function [Ti]=invtrans(T) %平移矩阵，逆矩阵
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
    S=size(theta);
    for i=1:S(1)
        for j=1:S(2)
                while (abs(theta(i,j))>pi)
                    if (theta(i,j)>pi)
                        theta(i,j)=theta(i,j)-2*pi;
                    elseif (theta(i,j)<-pi)
                        theta(i,j)=theta(i,j)+2*pi;
                    end
                end
        end
    end
end


