%% initialize
clc; close all; clear all;
format shortg; format compact;

%% DH方法
% DH分为标准DH和改进DH，建模的平移旋转变换顺序有少许不同，一次关节变换包含4个最基础的变换矩阵，且为两个不同的关节的变换参数
% 本文使用变换矩阵 T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);为自定义矩阵，4个参数全为前关节的变换参数

%  'standard'表示标准DH，参数设在i和i+1之间，'modified'改进DH，参数设在i-1和i之间；
% L = Link([THETAi Di Ai-1 ALPHAi-1 SIGMA],'modified');
% % theta：绕Zi轴，从Xi-1旋转到Xi的角度
% % D：沿Zi轴，从Xi-1移动到Xi的距离
% % A：沿Xi轴，从Zi移动到Zi+1的距离
% % alpha：绕Xi轴，从Zi旋转到Zi+1的角度

% L = Link([THETA D A ALPHA SIGMA],'standard');
% % theta：绕Zi轴，从Xi旋转到Xi+1的角度
% % D：沿Zi轴，从Xi移动到Xi+1的距离
% % A：沿Xi轴，从Zi移动到Zi+1的距离
% % alpha：绕Xi+1轴，从Zi旋转到Zi+1的角度
% Li = Link([关节角度 连杆偏置 连杆长度 连杆扭转角 sigma旋转0移动1 关节偏移量],'standard');
% Li = Link([theta(1) d(1) a(1) alpha(1) 0 offset],'standard');

%% UR5
theta=[0 0 0 0 0 0]; %两公垂线之间的夹角；
d=[90 -100 100 90 90 90]; %两公垂线之间的距离；
a=[0 -420 -400 0 0 0]; %两关节轴线之间公垂线长度；
alpha=[pi/2 0 0 pi/2 -pi/2 0]; %两关节轴线之间夹角；
offset=[0 0 0 0 0 0]; %[0 -pi/2 0 -pi/2 0 0]; %初始角度theta的偏转量；
sigma=0; mdh=0;%旋转关节；向后建模，默认T1为T01；

% L1 = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
% L2 = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
% L3 = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
% L4 = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
% L5 = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
% L6 = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);
% robot_UR5=SerialLink([L1,L2,L3,L4,L5,L6],'name','UR5'); 
% robot_UR5.display();  
% robot_UR5.teach(); 

% theta=[0 0 0 0 0 0];
% theta=[0 -pi/2 0 -pi/2 0 0]; %初始偏移版本
% d=[90 0 0 90 90 90]; % d=[90 -100 100 90 90 90]; 
% % a=[0 420 400 0 0 0]; 
% a=[0 -420 -400 0 0 0]; 
% alpha=[pi/2 0 0 pi/2 -pi/2 0];
% offset=[0 0 0 0 0 0]; % offset=[0 -pi/2 0 -pi/2 0 0]; 
% sigma=0; mdh=0;

L1 = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
L2 = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
L3 = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
L4 = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
L5 = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
L6 = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);
robot_UR5=SerialLink([L1,L2,L3,L4,L5,L6],'name','UR5'); 
robot_UR5.display();
figure('NumberTitle', 'off', 'Name', 'UR5');
robot_UR5.teach(theta); 
% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|         90|          0|     1.5708|          0|
% |  2|         q2|          0|       -420|          0|          0|
% |  3|         q3|          0|       -400|          0|          0|
% |  4|         q4|         90|          0|     1.5708|          0|
% |  5|         q5|         90|          0|    -1.5708|          0|
% |  6|         q6|         90|          0|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+
    
    
    