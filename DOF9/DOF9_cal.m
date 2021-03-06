%% initialize
clc; close all; clear all;
format shortg; format compact;

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%2020_09版 文本代码
九自由度，需要两个参数，RCM远心点和O9末端位姿
获取正向末端位姿有两种方法；
1，与六轴一样，设定9个关节的参数，比如使用随机函数；
T_end=T_RCM*T_AV*T_49 
2，ROS仿真中，根据手动设置合适的位置，设定RCM远心点，设定末端位姿（以好demo仿真，好截图为原则）；
获得逆解也有两种方法，通过正向运动学的等式推导，可以容易的发现等式之间的T矩阵关系
    1，使用参数 d7 th8 th9
    T_V=inv(T_AV)*inv(T_RCM)*T_end；
    T_R=T_end*inv(T9)*inv(T8)*inv(T7)；写一个函数获取T9T8T7逆乘
    2，使用参数 th4 th5 th6；
    T_V=inv(T_AV)*inv(T_RCM)*T_end；
    T_R=T_RCM*Tv4*Tv5*Tv6；
    显然第一种方法T7*T8*T9更加简洁高效，使用更少的参数和计算；

在ROS中，进行简单的轨迹规划和圆弧插补；
为了获得可靠位姿点，需要设定一个优雅的灵活工作空间范围，
ROS模型版本DH数据
d=[90 0 0 100 100 100 0 0 0]; 
a=[0 -420 -400 0 0 0 0 50 50]; 
T_RCM=transl(-500,-90,17.321);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
目前程序调试方法比较low，快捷键注释或加分号终止输出；
但是作为MATLAB的调试程序，需要大量的中间数据输出，这种方法是最简洁直观的了，但是需要对程序足够熟悉；
好吧，这只是调试程序，正式版的发布程序必须正式对输入输出做出明确规定，对函数严格要求；
编程要考虑程序的健壮性；全面性，精确性，最简性；

% 大论文最终数据
d=[90 0 0 90 90 90 0 0 0]; 
a=[0 -420 -400 0 0 0 0 50 50]; 
alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
theta_rand=[-0.93923     -0.58306    -0.090068       -2.342      -1.5436       1.4268       677.06       1.7621       -0.398]
T_RCM=transl(-600+theta_rand(1),theta_rand(2),100+theta_rand(3));
T_AV=trotx(pi/2);
T_49=forward_kine49(theta_rand); %T_49==T_V6
T_end=T_RCM*T_AV*T_49 

关键数据
T_RCM
T_end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

%% 获取设定正向的末端位姿
theta_rand=[2*rand(1)-1 2*rand(1)-1 2*rand(1)-1 pi*(2*rand(1)-1) pi*(2*rand(1)-1)/2 pi*(2*rand(1)-1) 500*(1+rand(1)) pi*(2*rand(1)-1) pi*(2*rand(1)-1)];
theta_rand=[-0.93923     -0.58306    -0.090068       -2.342      -1.5436       1.4268       677.06       1.7621       -0.398]

T_RCM=transl(-600+theta_rand(1),theta_rand(2),100+theta_rand(3)) %指定RCM远心点位置 %限定范围的，随机RCM远心点
% the_rounding(T_RCM)
T_AV=trotx(pi/2); %DOF9到Virtual6之间的基坐标系变换；
T_49=forward_kine49(theta_rand); %T_49==T_V6
T_end=T_RCM*T_AV*T_49  %获取DOF9末端任意位姿
% T_tcp %工具中心点 的英文名称为“Tool Central Point”，简写为“TCP”；后放弃tcp命名，使用T_end表示末端位姿；
% T_temp=forward_kine16(theta_rand)
% T_rcm=[1 0 0 T_temp(1,4);0 1 0 T_temp(2,4);0 0 1 T_temp(3,4);0 0 0 1]


%% 求9DOF逆解
T=invtrot(T_AV)*invtrans(T_RCM)*T_end;  %T49
thv=ikine_Virtual_2(T); %theta_virtual %两组解，d7>0
% thv1=ikine_Virtual_4(T) %单独获取虚拟六轴部分的解
% the_rounding(thv1*180/pi)
% for i=1:4
%     forward_kine456(thv1(i,:))%验证证明，不同的th4 th5 th6乘积相同；
% end

% T=T_end*inv(T9)*inv(T8)*inv(T7);
T1=T_end*inverse_kine987(thv(1,1:9));   %T16
T2=T_end*inverse_kine987(thv(2,1:9)); %从virtual分别获得两组，分别对应actual4组，实际上有4*8组，这里做了条件筛选；


%% toolbox仿真验证图
theta=[0 0 0 0 0 0 0 0 0];
d=[90 0 0 90 90 90 0 0 0]; %沿Z 
a=[0 -420 -400 0 0 0 0 50 50]; %沿X  % a=[0 420 400 0 0 0]; 
alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0]; %绕X
offset=[0 0 0 0 0 0 0 0 0];
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
robot=SerialLink([L(1) L(2) L(3) L(4) L(5) L(6) L(7) L(8) L(9)],'name','DOF9');%   

% 4+4组
%     tha1=ikine_Actual_4(T1);%theta_actual
%     tha2=ikine_Actual_4(T2);
%     theta(1:4,1:6)=tha1;
%     theta(5:8,1:6)=tha2;
%     for i=1:4 % 区别于变量名整体赋值，索引逐个赋值
%         theta(i,7:9)=thv(1,7:9);
%         theta(4+i,7:9)=thv(2,7:9);
%     end
%     theta_deg(1:8,1:6)=theta(:,1:6)*180/pi;%转为角度制
%     theta_deg(1:8,7)=theta(:,7); % 单独处理d7    
%     theta_deg(1:8,8:9)=theta(:,8:9)*180/pi;

% 8+8组
    tha1=ikine_Actual_8(T1);%theta_actual
    tha2=ikine_Actual_8(T2);
    theta(1:8,1:6)=tha1;
    theta(9:16,1:6)=tha2;
    for i=1:8 % 区别于变量名整体赋值，索引逐个赋值
        theta(i,7:9)=thv(1,7:9);
        theta(8+i,7:9)=thv(2,7:9);
    end
    theta;
    theta_deg(1:16,1:6)=theta(:,1:6)*180/pi;%转为角度制
    theta_deg(1:16,7)=theta(:,7); % 单独处理d7    
    theta_deg(1:16,8:9)=theta(:,8:9)*180/pi;
    theta_deg  

%   用toolbox输出仿真图像，手动编号；
%     i=8+8
%     figure(i);
%     robot.teach(theta(i,:)); 
%     forward_kine19(theta(i,1:9))
    
% 输出角度数据
% the_rounding(theta_deg)%转化为字符串，对数据进行处理，四舍五入，保留3位小时
% tha1=ikine_Actual_8(T1);%theta_actual
% the_rounding(tha1*180/pi)
% tha2=ikine_Actual_8(T2);
% the_rounding(tha1*180/pi)
    
%% 正解验证
for i=1:4 %验证RCM坐标
%     eval(['Q',num2str(i),'=','vpa(forward_kine49(theta(i,1:9))-T,6)']);
    T8=forward_kine16(theta(i,1:6))-T_RCM;
    T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4));
    equ_rcm(i)=T8p;
%     eval(['Q',num2str(i),'=','T8p']); 
end
    equ_rcm
for i=1:16 %验证9DOF末端坐标
%     eval(['Q',num2str(i),'=','vpa(forward_kine49(theta(i,1:9))-T,6)']);
    T8=forward_kine19(theta(i,1:9))-T_end;
    T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4));
%     eval(['Q',num2str(i),'=','T8p']); 
    equ_nine(i)=T8p;
end
    equ_nine

%     for i=1:8 
%         robot_dof9.teach(theta(i,1:9)); 
%         hold on
%     end

%% ROS的正向末端位姿获取
% T_RCM=transl(-600+theta_rand(1),theta_rand(2),100+theta_rand(3)) %指定RCM远心点位置 %限定范围的，随机RCM远心点
fprintf("this is ROS \n");
T_RCM=transl(-510,-100,7.3205);
T_AV=trotx(pi/2); %DOF9到Virtual6之间的基坐标系变换
T_R=T_RCM*T_AV;
Ti_R=invtrot(T_AV)*invtrans(T_RCM);

% theta_rand=[2*rand(1)-1 2*rand(1)-1 2*rand(1)-1 pi*(2*rand(1)-1) pi*(2*rand(1)-1)/2 pi*(2*rand(1)-1) 500*(1+rand(1)) pi*(2*rand(1)-1) pi*(2*rand(1)-1)]
theta_rand=[-0.93923     -0.58306    -0.090068       -2.342      -1.5436       1.4268       677.06       1.7621       -0.398];%
theta_rand=[-10.037	-51.166	112.351	80.953	161.093	-28.039	677.060*180/pi	100.961	-22.804]*pi/180;
forward_kine19(theta_rand);

theta_rand*180/pi;
T_V6=forward_kine49(theta_rand);%T_V6==T_49
T_end=T_RCM*T_AV*T_V6;  %获取DOF9末端任意位姿
T_end=forward_kine19(theta_rand);

% T_tcp %工具中心点 的英文名称为“Tool Central Point”，简写为“TCP”
% theta=[0,-30,60,-120,-90,0,100*180/pi,90,0]*pi/180 %home_pose下的RCM点位置；
% theta=[0,-60,120,-150,-90,0,100*180/pi,90,0]*pi/180 
% forward_kine16(theta_rand)
% theta=[0,-60,120,-150,-90,0,100*180/pi,90,0]*pi/180;
% forward_kine19(theta)% [-510,-100,-192.679]
% forward_kine16(theta)% RCM==[-510,-100,7.3205]
% T_rcm是xyz三个坐标点
% T_end=[
%     0, 1, 0, -510;
%     0, 0, -1, -100;
%     -1, 0, 0, -192.68;
%     0, 0, 0, 1]

%% 求逆3+6

% T_V6==T4i*T5i*T6i*T7*T8*T9==invtrot(T_AV)*invtrans(T_RCM)*T_end -->d7 th8 th9
T_V6=Ti_R*T_end;%invtrot(T_AV)*invtrans(T_RCM)*T_end;  %T49
thv=ikine_Virtual_2(T_V6); %theta_virtual
for i=1:4 % 区别于变量名整体赋值，索引逐个赋值
    theta(i,7:9)=thv(1,7:9);
    theta(4+i,7:9)=thv(2,7:9);
end

% %% 求逆6+6
% thv=ikine_Virtual6(T) %theta_virtual

% T_A6==T1*T2*T3*T4*T5*T6==T_end*inv(T9)*inv(T8)*inv(T7)
% T_A6=T_end*inv(T9)*inv(T8)*inv(T7);
T_A61=T_end*inverse_kine987(thv(1,1:9));   %T16
T_A62=T_end*inverse_kine987(thv(2,1:9)); %从virtual分别获得两组，分别对应actual4组，实际上有4*8组，这里做了条件筛选；
%     tha1=ikine_Actual6(T_A61);    theta(1:4,1:6)=tha1;
%     tha2=ikine_Actual6(T_A62);    theta(5:8,1:6)=tha2;
theta(1:4,1:6)=ikine_Actual_4(T_A61);
theta(5:8,1:6)=ikine_Actual_4(T_A62);
% theta

forward_kine19(theta(1,:));
theta_deg=theta*180/pi;%; 把弧度输出为角度，方便阅读
% theta_deg(1:8,1:6)=theta(:,1:6)*180/pi;
% theta_deg(1:8,7)=theta(:,7);
% theta_deg(1:8,8:9)=theta(:,8:9)*180/pi;
% theta_deg
  
%% 验证
for i=1:4
%     eval(['Q',num2str(i),'=','vpa(forward_kine49(theta(i,1:9))-T,6)']);
    T8=forward_kine16(theta(i,1:6))-T_RCM;
    T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4));
    equ(i)=T8p;
%     eval(['Q',num2str(i),'=','T8p']); 
end
%     equ
for i=1:8
%     eval(['Q',num2str(i),'=','vpa(forward_kine49(theta(i,1:9))-T,6)']);
    T8=forward_kine19(theta(i,1:9))-T_end;
    T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4));
%     eval(['Q',num2str(i),'=','T8p']); 
    equ(i)=T8p;
end
    equ;

% robot_dof9.display();% figure('NumberTitle', 'off', 'Name', 'DOF9');
% robot_dof9.teach(theta(8,1:9)); 
T=[0 1 0 -510;
    1 0 0 -110;
    0 0 -1 -110;
    0 0 0 1];
% tr2rpy(T) %3.1416       3.1416       1.5708
% R=rpy2tr(3.1416,3.1416,1.5708)
% R=rpy2tr(-1.5708,3.1416,3.1416,'xyz' )
% R=rpy2tr(-1.5708,3.1416,3.1416,'zyx' )
% R=rpy2tr(-1.5708,3.1416,3.1416,'yxz' )
% R=rpy2tr(-1.5708,3.1416,3.1416,'arm' )
% R=rpy2tr(-1.5708,3.1416,3.1416,'vehicle' )
% R=rpy2tr(-1.5708,3.1416,3.1416,'camera' )
%    'deg'      Compute angles in degrees (radians default)
%    'xyz'      Rotations about X, Y, Z axes (for a robot gripper)
%    'zyx'      Rotations about Z, Y, X axes (for a mobile robot, default)
%    'yxz'      Rotations about Y, X, Z axes (for a camera)
%    'arm'      Rotations about X, Y, Z axes (for a robot arm)
%    'vehicle'  Rotations about Z, Y, X axes (for a mobile robot)
%    'camera'   Rotations about Y, X, Z axes (for a camera)
% Teach面板的RPY与函数rpy2tr略有不同？

% home_pose==T=rpy_to_tr(pi,pi,pi/2)

function T=rpy_to_tr(gamma,beta,alpha,RCMx,RCMy,RCMz)
% RPY角
% 绕参考坐标系，绕定轴X（Roll）—Y（Pitch）—Z（Yaw）旋转，旋转矩阵左乘。顺序：绕X转γ，绕Y转β，绕Z转α
% 公式：R(γ,β,α) = Rot(z,α)Rot(y,β)Rot(x,γ)
% 
% 欧拉角
% 绕自身坐标系，绕动轴Z-Y-X旋转，旋转矩阵右乘。顺序：绕Z转α，绕Y转β，绕X转γ
% 公式：R(α,β,γ) = Rot(z,α)Rot(y,β)Rot(x,γ)
T=[ cos(alpha)*cos(beta),cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma),cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma),RCMx;
    sin(alpha)*cos(beta),sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma),sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma),RCMy;
    -sin(beta),cos(beta)*sin(gamma),cos(beta)*cos(gamma),RCMz;
    0,0,0,1;
];

end

%% 数据处理，保留3位，四舍五入；
function theta=the_rounding(the)
    a = the;
    n = 4; %保留位数
    b = a*10^n;% 先化成整数，
    a = round(b); %s四舍五入
    need_num = a/10^n; %再转成小数
    need_str = num2str(need_num);
end

%% ikine_Virtual_2(T)
function th=ikine_Virtual_2(T)
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
    
% 理论上虚拟六轴也有八组解，在实体789轴上，只有4组不同的解
% 这里又筛去了一般d7<0的解，只剩2组实际可用解；
    i=1;
    for j=1:4
        if d7(j)>0 %取d7大于0的解
            th(i,9)=In_pi(th9(j));
            th(i,8)=In_pi(th8(j));
            th(i,7)=d7(j);
            i=i+1;
        end
    end
end

%% ikine_Virtual_4(T)
function th=ikine_Virtual_4(T)
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
    
    d7(5:8)=d7(1:4);
    i=1; 
    for j=1:8
        if d7(j)>0 %取d7大于0的解
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

%% ikine_Actual_4(T)
function th=ikine_Actual_4(T)
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    
    % th1
%     m1=px-d6*ax;
%     n1=py-d6*ay;
%     if (d4^2<=m1^2+n1^2) %th1盲区判定
%         k1=d4/sqrt(m1^2+n1^2);
%         th1(1)=acos(-k1)-atan2(m1,n1);
%         th1(2)=-acos(-k1)-atan2(m1,n1);
    m1=d6*ax-px;
    n1=d6*ay-py;
    if (d4^2<=m1^2+n1^2) %th1盲区判定
        k1=d4/sqrt(m1^2+n1^2);
        th1(1)=acos(k1)-atan2(m1,n1);
        th1(2)=-acos(k1)-atan2(m1,n1);
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

    % th234
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

%% ikine_Actual_8(T)
function th=ikine_Actual_8(T)
%     d1=90;d4=100;d5=100;d6=100;a2=-420;a3=-400;%Solidworks-ROS模型略有不同；
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

    for i=1:8
        th(i,1)=In_pi(th1(i));
        th(i,2)=In_pi(th2(i));
        th(i,3)=th3(i);
        th(i,4)=In_pi(th4(i));
        th(i,5)=th5(i);
        th(i,6)=In_pi(th6(i));
    end
end

%% forward kinematic
% 9轴部分和虚拟六轴模型不一样，需要单独设定
function T = forward_kine19(theta)% 九轴
%     syms d1 d4 d5 d6 a2 a3 a8 a9 d7;
    d1=90;d4=90;d5=90;d6=90; d7=0;
    %     d1=90;d4=100;d5=100;d6=100; d7=0; %Solidworks-ROS模型略有不同；
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
    T7=DH_forward(d(7),theta(7),a(7),alpha(7)); 
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T1*T2*T3*T4*T5*T6*T7*T8*T9;
end
function T = forward_kine16(theta) %前实体六轴
    d1=90;d4=90;d5=90;d6=90;d7=0;    a2=-420;a3=-400; a8=50;a9=50;
    %     d1=90;d4=100;d5=100;d6=100; %Solidworks-ROS模型略有不同；
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
    alpha=[0 0 0 -pi/2 pi/2 0 pi/2 -pi/2 0];%不同的alpha[]
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
function T = forward_kine456(theta)% 虚拟六轴
    a8=50;a9=50; %d7=0;
    d=[0 0 0 0 0 0 0 0 0];      
    a=[0 0 0 0 0 0 0 a8 a9]; 
    alpha=[0 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T=T4*T5*T6;
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


%% Standard 参数标准化
function mn = N_zero(mn) %limit_zero
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<1E-6
                mn(i,j)=0;
            end
        end
    end
end

function theta = In_pi(theta) %limit_inpi
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
