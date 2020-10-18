%% setting
clc;clear;close all;
format short g;format compact;

%%
theta=[0 0 0 0 0 0]; %绕Z 
% d=[0 0 -1000 0 0 0]; %沿Z 
d=[0 0 0 0 0 0]; %沿Z 
a=[0 0 0 0 50 50]; %沿X  % a=[0 420 400 0 0 0]; 
alpha=[pi/2 -pi/2 0 pi/2 -pi/2 0]; %绕X
offset=[0 0 0 0 0 0];  
L(1) = Link([theta(1) d(1) a(1) alpha(1) 0 offset(1)],0);
L(2) = Link([theta(2) d(2) a(2) alpha(2) 0 offset(2)],0);
L(3) = Link([theta(3) d(3) a(3) alpha(3) 0 offset(3)],0);
L(4) = Link([theta(4) d(4) a(4) alpha(4) 1 offset(4)],0);  L(4).qlim = [0 2000];
L(5) = Link([theta(5) d(5) a(5) alpha(5) 0 offset(5)],0); 
L(6) = Link([theta(6) d(6) a(6) alpha(6) 0 offset(6)],0);
robot_V6=SerialLink( L,'name','V6'); 
% robot_V6.display(); %DH表
% robot_V6.teach(theta); 
% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|          0|          0|     1.5708|          0|
% |  2|         q2|          0|          0|    -1.5708|          0|
% |  3|         q3|          0|          0|          0|          0|
% |  4|          0|         q4|          0|     1.5708|          0|
% |  5|         q5|          0|         50|    -1.5708|          0|
% |  6|         q6|          0|         50|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+


%% initial Virtual6
% theta_deg=[0 0 0 10 20 30 40*180/pi 50 60]
% theta=theta_deg/180*pi;
% theta=[  0            0            0         2.41       0.8579      -1.1062       863.26      0.60959      -1.4757];
% theta=[0 0 0 pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) 1000*rand(1) pi*(2*rand(1)-1) pi*(2*rand(1)-1)];
theta=[0  0  0  -30.937      -89.429       -24.31       5729.6      -87.366       71.545]/180*pi;  % x4jie
theta_deg=theta*180/pi

T=forward_kine49(theta);
% T=rpy2tr(1,1,0);T(1,4)=-100;T(2,4)=-100;T(3,4)=500;
theta=ikine_Virtual(T);
theta*180/pi

for i=1:8
%     eval(['Q',num2str(i),'=','vpa(forward_kine49(theta(i,1:9))-T,6)']);
    T8=forward_kine49(theta(i,1:9))-T;
    T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4))
%     eval(['Q',num2str(i),'=','T8p']); %
   
end

% robot_V6.teach(theta(8,4:9));  %输出仿真图像
% robot_V6.teach(theta(2,4:9)); 

%% 输出保留3位的表格数据
theta_deg(1:8,4:6)=theta(:,4:6)*180/pi;
theta_deg(1:8,7)=theta(:,7);
theta_deg(1:8,8:9)=theta(:,8:9)*180/pi;
% theta_deg

a = theta_deg;
n = 3; %保留位数
b = a*10^n;% 先化成整数，
a = round(b); %s四舍五入
need_num = a/10^n; %再转成小数
% need_str = num2str(need_num)


%% function new algorithm
function th=ikine_Virtual(T)
    a8=50;a9=50;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
% th9
    Px=px-a9*nx;
    Py=py-a9*ny;
    Pz=pz-a9*nz;    
% 当且仅当 ，d7=0/s5=0才能用
% delta=Pz/(a8*sqrt(nz^2+oz^2))
% th9(1,1)=acos(Pz/(a8*sqrt(nz^2+oz^2)))-atan2(oz,nz);
% th9(1,2)=-acos(Pz/(a8*sqrt(nz^2+oz^2)))-atan2(oz,nz);
% th9=In_pi(th9); th9*180/pi

% 0==Pm*cos(th9)-Pn*sin(th9);
    Pm=(Pz*ay-Py*az)*nx+(Px*az-Pz*ax)*ny+(Py*ax-Px*ay)*nz;
    Pn=(Pz*ay-Py*az)*ox+(Px*az-Pz*ax)*oy+(Py*ax-Px*ay)*oz;
    th9(1)=atan(Pm/Pn);
%     th9(1)=atan2(Pm,Pn);
    th9(2)=th9(1)+pi;
    th9=In_pi(th9);%     th90=th9*180/pi

%% th8
    Rx=nx*cos(th9)-ox*sin(th9); 
    Ry=ny*cos(th9)-oy*sin(th9); 
    Rz=nz*cos(th9)-oz*sin(th9);  
    m81=ax*(Pz-a8*Rz)-az*(Px-a8*Rx);
    n81=Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz);
    m82=ay*(Pz-a8*Rz)-az*(Py-a8*Ry);
    n82=Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz);
%     eq81=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx)).*cos(th8)-(Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz)).*sin(th8)
%     eq82=(ay*(Pz-a8*Rz)-az*(Py-a8*Ry)).*cos(th8)-(Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz)).*sin(th8)
    th8(1:2)=atan(m82./n82);
    th8(3:4)=th8(1:2)+pi; %theta8有4个值；
    
%     th82(1:2)=atan(m82./n82);
%     th82(3:4)=th8(1:2)+pi;
    th9(3:4)=th9(1:2);
    th8=In_pi(th8);
%     th80=th8*180/pi
%     th82=In_pi(th82);th820=th82*180/pi
%% d7
%     Mx=ax*cos(th8) + sin(th8).*(nx*cos(th9)-ox*sin(th9))
%     Nx=px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9))
%     d71=Nx./Mx
%     My=ay*cos(th8) + sin(th8).*(ny*cos(th9)-oy*sin(th9))
%     Ny=py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9))
%     d72=Ny./My
    Mz=az*cos(th8) + sin(th8).*(nz*cos(th9)-oz*sin(th9));
    Nz=pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9));
    d7(1:4)=Nz./Mz;
    d7(5:8)=d7(1:4);
% eqx=d7.*(ax*cos(th8) + sin(th8).*(nx*cos(th9)-ox*sin(th9))) - (px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9)));
% eqy=d7.*(ay*cos(th8) + sin(th8).*(ny*cos(th9)-oy*sin(th9))) - (py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9)));
% eqz=d7.*(az*cos(th8) + sin(th8).*(nz*cos(th9)-oz*sin(th9))) - (pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9)));
% sum(eqx)
% sum(eqy)
% sum(eqz)
% 奇异点
    % 无穷量(Inf) %1/0
    % 不定值量(NaN) %0/0


%{
    if Mx==0&&Nx==0
        if My==0&&Ny==0
            d7=Mz./Nz;
        else
            d7=My./Ny;
        end
    else
        d7=Mz./Nz;
    end
    %}
    
%% th5
    n5=az*cos(th8) + sin(th8).*(nz*cos(th9)-oz*sin(th9));
    th5(1:4)=acos(n5);
    th5(5:8)=-th5(1:4);
%     th5*180/pi
%     eq51=cos(th5)-az*cos(th8) - sin(th8).*(nz*cos(th9)-oz*sin(th9));sum(eq51);
%     eq52=d7.*cos(th5) - (pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9)));    sum(eq52)

%  th4
    th9(5:8)=th9(1:4);
    th8(5:8)=th8(1:4);
    s=sign(sin(th5));
%     n4=-(ax*cos(th8)+sin(th8).*(nx*cos(th9)-ox*sin(th9)))./sin(th5);
%     m4=-(ay*cos(th8)+sin(th8).*(ny*cos(th9)-oy*sin(th9)))./sin(th5);
    n4=-ax*cos(th8)-sin(th8).*(nx*cos(th9)-ox*sin(th9));
    m4=-ay*cos(th8)-sin(th8).*(ny*cos(th9)-oy*sin(th9));
    th4=atan2(m4./s,n4./s);
%   th6
%     n6=(-az*sin(th8)+cos(th8).*(nz*cos(th9)-oz*sin(th9)))./sin(th5);
%     m6=(-oz*cos(th9)-nz*sin(th9))./sin(th5);
    n6=-az*sin(th8)+cos(th8).*(nz*cos(th9)-oz*sin(th9));
    m6=-oz*cos(th9)-nz*sin(th9);
    th6=atan2(m6./s,n6./s);
%% 
    for i=1:8
        th(i,9)=th9(i);
        th(i,8)=th8(i);
        th(i,7)=d7(i);%+1000; %*pi/180;  %被动转角度
        th(i,5)=th5(i);
        th(i,4)=th4(i);
        th(i,6)=th6(i);
    end
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

%% kinetic_function
% function T = forward_kine456(theta) % virtual六轴专用函数，正运动位姿获取
%     d=[0 0 0 0 0 0 0 0 0];     %syms  a8 a9 d7;
% %     a8=50;a9=50; %d7=0;
%     a=[0 0 0 0 0 0 0 50 50]; 
%     alpha=[0 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
%     T4=DH_forward(theta(4),d(4),a(4),alpha(4));
%     T5=DH_forward(theta(5),d(5),a(5),alpha(5));
%     T6=DH_forward(theta(6),d(6),a(6),alpha(6));
%     T7=DH_forward(0,theta(7),a(7),alpha(7)); 
%     T=T4*T5*T6*T7;
% end

function T = forward_kine49(theta) % virtual六轴专用函数，正运动位姿获取
    %syms  a8 a9 d7;
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
function T = DH_forward(theta,d,a,alpha) % 正运动函数
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end

