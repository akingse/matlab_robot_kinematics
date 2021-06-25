%% initialize
clc; close all; clear all;
format shortg; format compact;

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
生成简单的蒙特卡洛图
% Monte Carlo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

N=100; %随机值数量；这里取1000，运算快

%% 虚拟六轴模型 Virtual6
% theta=[0 0 0 0 0 0]; %绕Z 
% d=[0 0 0 0 0 0]; %沿Z 
% a=[0 0 0 0 50 50]; %沿X  % a=[0 420 400 0 0 0]; 
% alpha=[-pi/2 pi/2 0 pi/2 -pi/2 0]; %绕X
% offset=[0 0 0 0 0 0]; 
% L(1) = Link([theta(1) d(1) a(1) alpha(1) 0 offset(1)],0); %此表达形式sixlink.fkine(B)不识别
% L(2) = Link([theta(2) d(2) a(2) alpha(2) 0 offset(2)],0);
% L(3) = Link([theta(3) d(3) a(3) alpha(3) 0 offset(3)],0);
% L(4) = Link([theta(4) d(4) a(4) alpha(4) 1 offset(4)],0);  L(4).qlim = [0 1000];
% L(5) = Link([theta(5) d(5) a(5) alpha(5) 0 offset(5)],0);
% L(6) = Link([theta(6) d(6) a(6) alpha(6) 0 offset(6)],0);
% sixlink=SerialLink(L,'name','V6') %

% R1=unifrnd(-pi,pi,[1,N]);  %6个连杆的各关节随机值，蒙特卡洛法要求
% R2=unifrnd(-pi,pi,[1,N]);
% R3=unifrnd(-pi,pi,[1,N]);
% R4=unifrnd(0,1000,[1,N]);
% R5=unifrnd(-pi,pi,[1,N]);
% R6=unifrnd(-pi,pi,[1,N]);

% ---------------------------------------------------------------------------
%% 实体六轴模型 Actual6
theta=[0 0 0 0 0 0];%theta=[0 -90 0 -90 0 0]/180*pi;
d=[90 0 0 90 90 90]; % d=[90 -100 100 90 90 90]; 
a=[0 -420 -400 0 0 0]; % a=[0 420 400 0 0 0]; 
alpha=[pi/2 0 0 pi/2 -pi/2 0];
% offset=[0 0 0 0 0 0]; % 
offset=[0 -pi/2 0 -pi/2 0 0]; 
sigma=0; mdh=0;
% L(1) = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
% L(2) = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
% L(3) = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
% L(4) = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
% L(5) = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
% L(6) = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);
% sixlink=SerialLink(L,'name','A6') %
% sixlink.teach(theta); 
% hold on
% R1=unifrnd(-pi,pi,[1,N]);  %6个连杆的各关节随机值，蒙特卡洛法要求
% R2=unifrnd(-pi,pi,[1,N]);
% R3=unifrnd(-pi,pi,[1,N]);
% R4=unifrnd(-pi,pi,[1,N]);
% R5=unifrnd(-pi,pi,[1,N]);
% R6=unifrnd(-pi,pi,[1,N]);


%% DOF9
theta=[0 0 0 0 0 0 0 0 0]; %绕Z 
d=[90 0 0 90 90 90 0 0 0]; %沿Z 
a=[0 -420 -400 0 0 0 0 50 50]; %沿X  % a=[0 420 400 0 0 0]; 
alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0]; %绕X
offset=[0 0 0 0 0 0 0 0 0]; %初始偏移角
% offset=[0 -pi/2 0 -pi/2 0 0 0 pi/2 0]; 
sigma=0; mdh=0; 
L(1) = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
L(2) = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
L(3) = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
L(4) = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
L(5) = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
L(6) = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);
L(7) = Link([theta(7) d(7) a(7) alpha(7) 1 offset(7)], mdh);  L(7).qlim = [0 1000];
L(8) = Link([theta(8) d(8) a(8) alpha(8) sigma offset(8)],mdh); 
L(9) = Link([theta(9) d(9) a(9) alpha(9) sigma offset(9)],mdh);
sixlink=SerialLink([L(1) L(2) L(3) L(4) L(5) L(6) L(7) L(8) L(9)],'name','DOF9');%   
theta=[0 0 0 0 0 0 1000 0 0];
sixlink.teach(theta); 
hold on;

R1=unifrnd(-pi,pi,[1,N]);  %Continuous uniform random numbers
R2=unifrnd(-pi,pi,[1,N]);
R3=unifrnd(-pi,pi,[1,N]);
R4=unifrnd(-pi,pi,[1,N]);
R5=unifrnd(-pi,pi,[1,N]);
R6=unifrnd(-pi,pi,[1,N]);
R7=unifrnd(0,1000,[1,N]);
R8=unifrnd(-pi,pi,[1,N]);
R9=unifrnd(-pi,pi,[1,N]);

A= cell(N, 6); %定义一个元胞组，用于储存以上6个连杆关节变量随机值
for i = 1:N
%     A{i} =[R1(i) R2(i) R3(i) R4(i) R5(i) R6(i)]; 
    A{i} = [R1(i) R2(i) R3(i) R4(i) R5(i) R6(i) R7(i) R8(i) R9(i)]; 
end                                         
B=cell2mat(A);    %转换为矩阵                  
RV=double(sixlink.fkine(B));       %末端位姿矩阵
fig=scatter3(squeeze(RV(1,4,:)),squeeze(RV(2,4,:)),squeeze(RV(3,4,:)),1); %xyz三维图
% view(0,90) %deg
% % figure,scatter3(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),squeeze(T07(3,4,:)),1); %xyz三维图



%% px py pz蒙特卡洛
theta=[0 0 0 pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) 100 pi*(2*rand(1)-1) pi*(2*rand(1)-1)];
% theta_deg=theta*180/pi

th4=-pi:0.01:pi;
th5=-pi:0.01:pi;
th6=-pi:0.01:pi;
d7=100;%0:1:20*pi;
th8=-pi:0.01:pi;
th9=-pi:0.01:pi;

a8=50;
a9=50;
px=-a8.*cos(th8).*(sin(th4).*sin(th6)-cos(th4).*cos(th5).*cos(th6))-a9.*sin(th9).*(cos(th6).*sin(th4)+cos(th4).*cos(th5).*sin(th6))-d7.*cos(th4).*sin(th5)-a9.*cos(th9).*(cos(th8).*(sin(th4).*sin(th6)-cos(th4).*cos(th5).*cos(th6))+cos(th4).*sin(th5).*sin(th8))-a8.*cos(th4).*sin(th5).*sin(th8);
py=a8.*cos(th8).*(cos(th4).*sin(th6)+cos(th5).*cos(th6).*sin(th4))+a9.*sin(th9).*(cos(th4).*cos(th6)-cos(th5).*sin(th4).*sin(th6))-d7.*sin(th4).*sin(th5)+a9.*cos(th9).*(cos(th8).*(cos(th4).*sin(th6)+cos(th5).*cos(th6).*sin(th4))-sin(th4).*sin(th5).*sin(th8))-a8.*sin(th4).*sin(th5).*sin(th8);
pz=d7.*cos(th5)+a9.*cos(th9).*(cos(th5).*sin(th8)+cos(th6).*cos(th8).*sin(th5))+a8.*cos(th5).*sin(th8)+a8.*cos(th6).*cos(th8).*sin(th5)-a9.*sin(th5).*sin(th6).*sin(th9);

% T=forward_kine49(theta)
% scatter3(px,py,pz,1)
% unifrnd(0,1) %连续均匀分布数组
% Statistics and Machine Learning Toolbox %重装  


% scatter3(sin(x),cos(y),z,1)%
% x=unifrnd(0,12,[1,10000000]);
% y=unifrnd(0,9,[1,10000000]);
% pinshu=sum(y<x.^2 & x<=3)+sum(y<12-x & x>=3);
% area_appr=12*9*pinshu/10^7



