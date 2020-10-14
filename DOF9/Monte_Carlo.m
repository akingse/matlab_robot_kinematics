%% initialize
clc; close all; clear all;
format shortg; format compact;


%% Monte Carlo
% 虚拟
% theta=[0 0 0 0 0 0]; %绕Z 
% d=[0 0 0 0 0 0]; %沿Z 
% a=[0 0 0 0 50 50]; %沿X  % a=[0 420 400 0 0 0]; 
% alpha=[pi/2 -pi/2 0 pi/2 -pi/2 0]; %绕X
% offset=[0 0 0 0 0 0]; 
% L(1) = Link([theta(1) d(1) a(1) alpha(1) 0 offset(1)],0); %此表达形式sixlink.fkine(B)不识别
% L(2) = Link([theta(2) d(2) a(2) alpha(2) 0 offset(2)],0);
% L(3) = Link([theta(3) d(3) a(3) alpha(3) 0 offset(3)],0);
% L(4) = Link([theta(4) d(4) a(4) alpha(4) 1 offset(4)],0);  L(4).qlim = [0 1000];
% L(5) = Link([theta(5) d(5) a(5) alpha(5) 0 offset(5)],0);
% L(6) = Link([theta(6) d(6) a(6) alpha(6) 0 offset(6)],0);
% ---------------------------------------------------------------------------
% 实体
theta=[0 0 0 0 0 0];%theta=[0 -90 0 -90 0 0]/180*pi;
d=[90 0 0 90 90 90]; % d=[90 -100 100 90 90 90]; 
a=[0 -420 -400 0 0 0]; % a=[0 420 400 0 0 0]; 
alpha=[pi/2 0 0 pi/2 -pi/2 0];
% offset=[0 0 0 0 0 0]; % 
offset=[0 -pi/2 0 -pi/2 0 0]; 
sigma=0; mdh=0;
L(1) = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
L(2) = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
L(3) = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
L(4) = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
L(5) = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
L(6) = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);

% ---------------------------------------------------------------------------
sixlink=SerialLink(L,'name','6R') %
% sevenlink.display();
% sevenlink.teach(theta); 
% 
N=10000; %随机值数量，一般取大，这里取1000，运算快
R1=unifrnd(-pi,pi,[1,N]);  %7个连杆的各关节随机值，蒙特卡洛法要求
R2=unifrnd(-pi,pi,[1,N]);
R3=unifrnd(-pi,pi,[1,N]);
R4=unifrnd(0,1000,[1,N]);
% R4=unifrnd(-pi,pi,[1,N]);
R5=unifrnd(-pi,pi,[1,N]);
R6=unifrnd(-pi,pi,[1,N]);
A= cell(N, 6); %定义一个元胞组，用于储存以上7个连杆关节变量随机值
for i = 1:N
    A{i} =[R1(i) R2(i) R3(i) R4(i) R5(i) R6(i)]; 
end                                         
B=cell2mat(A);    %转换为矩阵                  
T07=double(sixlink.fkine(B));       %末端位姿矩阵
% fig=scatter3(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),squeeze(T07(3,4,:)),1); %xyz三维图
% view(0,90) %deg
% % figure,scatter3(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),squeeze(T07(3,4,:)),1); %xyz三维图

% view(90,90) %deg
% T=get(gca,'view') %得到视角（方位角和仰视角）。
% ---------------------------------------------------------------------------
f = figure('Units','normalized','OuterPosition',[0 0.5 1, 0.5]); %从屏幕左侧中间开始，按比例画图
% figure('Position',[100, 100, 1000, 1000]); %固定窗口
ax1 = subplot(2,2,1);xlabel('x');ylabel('y');hold on;
ax2 = subplot(2,2,2);ylabel('y');zlabel('z');hold on;
ax3 = subplot(2,2,3);xlabel('x');zlabel('z');hold on;
ax4 = subplot(2,2,4);xlabel('x');ylabel('y');zlabel('z');hold on;
fig=scatter3(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),squeeze(T07(3,4,:)),1);
plotObjs =fig;
copyobj(plotObjs,ax1);
copyobj(plotObjs,ax2);
copyobj(plotObjs,ax3);
copyobj(plotObjs,ax4);
view(ax1,0,90); title(ax1,'top');
view(ax2,90,0); title(ax2,'left');
view(ax3,0,0);  title(ax3,'front');
view(ax4,-37.5,30);  title(ax4,'axon');
% ax1 = subplot(1,3,1);
% view(ax1,0,90); title(ax1,'top');
% ax2 = subplot(1,3,2);
% fig=scatter3(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),squeeze(T07(3,4,:)),1); 
% view(ax2,90,0); title(ax2,'left');
% ax3 = subplot(1,3,3);
% fig=scatter3(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),squeeze(T07(3,4,:)),1); 
% view(ax3,0,0);  title(ax3,'front');

% % 
% % 使用方法：subplot（m,n,p）或者subplot（m n p）。
% % subplot是将多个图画到一个平面上的工具。其中，m表示是图排成m行，n表示图排成n列，p表示图所在的位置
% %%Plot some surfaces on 1st subplot
% % [X,Y,Z] = peaks(100); %[X,Y,Z] = peaks;
% X=squeeze(T07(1,4,:));
% Y=squeeze(T07(2,4,:));
% Z=squeeze(T07(3,4,:));
% % z=peaks;默认返回一个49*49的矩阵
% % z=peaks(n);返回一个n*n的矩阵
% s = surf(ax1,X,Y,Z); hold(ax1,'on');
% p = surf(ax1,X,Y,zeros(size(X))); hold(ax1,'off');
% plotObjs = [s,p]
% %Copy plot objects to other 2 subplots
% copyobj(plotObjs,ax2);
% copyobj(plotObjs,ax3);
% %%Set different viewing angle for each subplot
% view(ax1,0,90); title(ax1,'top');
% view(ax2,90,0); title(ax2,'left');
% view(ax3,0,0);  title(ax3,'front');

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

%% 蒙特卡洛
% scatter3(X,Y,Z) 在向量 X、Y 和 Z 指定的位置显示圆圈。
% scatter3(X,Y,Z,S) 使用 S 指定的大小绘制每个圆圈。要绘制大小相等的圆圈，请将 S 指定为标量。要绘制具有特定大小的每个圆，请将 S 指定为向量。
% scatter3(X,Y,Z,S,C) 使用 C 指定的颜色绘制每个圆圈。
% https://www.cnblogs.com/yibeimingyue/p/10030280.html
% startup_rvc %启动工具箱
% Robotics, Vision & Control: (c) Peter Corke 1992-2017 http://www.petercorke.com
% - Robotics Toolbox for MATLAB (release 10.3.1)
%{
% 测试程序
L(1)= Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2);
L(2)= Link('revolute', 'd', 147, 'a', 0, 'alpha', pi/2);
L(3)= Link('revolute', 'd', 600, 'a', 0, 'alpha', pi/2);
L(4)= Link('revolute', 'd', 147, 'a', 0, 'alpha', -pi/2);
L(5)= Link('revolute', 'd', 600, 'a', 0, 'alpha', pi/2);
L(6)= Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2);
L(7)= Link('revolute', 'd', 160, 'a', 0, 'alpha', pi/2);
sevenlink=SerialLink(L,'name','7R') %建立7R的模型
N=10000; %随机值数量，一般取大，这里取1000，运算快
R1=unifrnd(-pi,pi,[1,N]);  %7个连杆的各关节随机值，蒙特卡洛法要求
R2=unifrnd(pi/3,5*pi/3,[1,N]);
R3=unifrnd(-pi,pi,[1,N]);
R4=unifrnd(-pi/2,pi/2,[1,N]);
R5=unifrnd(-pi,pi,[1,N]);
R6=unifrnd(-5*pi/6,5*pi/6,[1,N]);
R7=unifrnd(-pi,pi,[1,N]); 
A= cell(N, 7); %定义一个元胞组，用于储存以上7个连杆关节变量随机值
for i = 1:N
    A{i} =[R1(i) R2(i) R3(i) R4(i) R5(i) R6(i) R7(i)]; 
end                                         
B=cell2mat(A);    %转换为矩阵                  
T07=double(sevenlink.fkine(B));       %末端位姿矩阵
%以下分别输入，得两个图
figure,scatter3(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),squeeze(T07(3,4,:)),1); %xyz三维图
title('scatter3')
% hold on; %放在同一张图,且在同一个框
% figure,plot(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),'.');title('plot') %xy二维图
%}
% theta=[th1 th2 th3 th4 th5 th6 d7 th8 th9]; 
theta=[0 0 0 0 0 0]; %绕Z 
d=[0 0 0 0 0 0]; %沿Z 
a=[0 0 0 0 50 50]; %沿X  % a=[0 420 400 0 0 0]; 
alpha=[pi/2 -pi/2 0 pi/2 -pi/2 0]; %绕X
offset=[0 0 0 0 0 0]; 
% L(1) = Link([theta(1) d(1) a(1) alpha(1) 0 offset(1)],0); %此表达形式sixlink.fkine(B)不识别
% L(2) = Link([theta(2) d(2) a(2) alpha(2) 0 offset(2)],0);
% L(3) = Link([theta(3) d(3) a(3) alpha(3) 0 offset(3)],0);
% L(4) = Link([theta(4) d(4) a(4) alpha(4) 1 offset(4)],0);  L(4).qlim = [0 1000];
% L(5) = Link([theta(5) d(5) a(5) alpha(5) 0 offset(5)],0);
% L(6) = Link([theta(6) d(6) a(6) alpha(6) 0 offset(6)],0);
L(1)= Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2);
L(2)= Link('revolute', 'd', 147, 'a', 0, 'alpha', pi/2);
L(3)= Link('revolute', 'd', 600, 'a', 0, 'alpha', pi/2);
L(4)= Link('revolute', 'd', 147, 'a', 0, 'alpha', -pi/2);
L(5)= Link('revolute', 'd', 600, 'a', 0, 'alpha', pi/2);
L(6)= Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2);
sixlink=SerialLink(L,'name','6V');
% sixlink.display();
% sixlink.teach(theta); 
q1=[1 1 1 0 0 0];
% T=sixlink.fkine(the)

N=100; %随机值数量
R4=unifrnd(-pi,pi,[1,N]);%连杆的各关节随机值，蒙特卡洛法要求
R5=unifrnd(-pi,pi,[1,N]);
R6=unifrnd(-pi,pi,[1,N]);
R7=200;
R8=unifrnd(-pi,pi,[1,N]); 
R9=unifrnd(-pi,pi,[1,N]);  
A= cell(N, 6); %定义一个元胞组
for i = 1:N
    A{i} =[R4(i) R5(i) R6(i) R7 R8(i) R9(i)]; 
end           
B=cell2mat(A);    %转换为矩阵  
% T06=sixlink.fkine(B);
% T06= fkine(sixlink,B)%换一个函数表达形式
% scatter3(squeeze(T06(1,4,:)),squeeze(T06(2,4,:)),squeeze(T06(3,4,:)),1); %xyz三维图



