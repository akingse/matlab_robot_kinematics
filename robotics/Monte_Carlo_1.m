%% setting
clc;clear;close all;
format shortg;format compact;



%% plot3

%建立机器人模型
%        theta    d           a          alpha     offset
ML1=Link([0       0           0           0          0     ],'modified'); 
ML2=Link([0       0           0.180      -pi/2       0     ],'modified');
ML3=Link([0       0           0.600       0          0     ],'modified');
ML4=Link([0       0.630       0.130      -pi/2       0     ],'modified');
ML5=Link([0       0           0           pi/2       0     ],'modified');
ML6=Link([0       0           0          -pi/2       0     ],'modified');
modrobot=SerialLink([ML1 ML2 ML3 ML4 ML5 ML6],'name','modified');
modrobot.teach(); 
hold on

N=1000                                            %随机次数
theta1=-165/180*pi+(165/180*pi+165/180*pi)*rand(N,1); %关节1限制
theta2=-90/180*pi+(155/180*pi+90/180*pi)*rand(N,1) ;  %关节2限制
theta3=-200/180*pi+(70/180*pi+200/180*pi)*rand(N,1) ; %关节3限制
theta4=-170/180*pi+(170/180*pi+170/180*pi)*rand(N,1); %关节4限制
theta5=-135/180*pi+(135/180*pi+135/180*pi)*rand(N,1); %关节5限制
theta6=-360/180*pi+(360/180*pi+360/180*pi)*rand(N,1) ;%关节6限制

for n=1:N %error
modmyt06=modrobot.fkine([theta1(n),theta2(n),theta3(n),theta4(n),theta5(n),theta6(n)]);
plot3(modmyt06.t(1),modmyt06.t(2),modmyt06.t(3),'b.','MarkerSize',1); 
view(0,90);  %设定观察视角

end


%% scatter3
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
% sixlink.teach(theta); 
hold on

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
% T06=double(sixlink.fkine(B));       %末端位姿矩阵
% scatter3(squeeze(T06(1,4,:)),squeeze(T06(2,4,:)),squeeze(T06(3,4,:)),1); %xyz三维图
% 
% view(90,90) %deg
% T=get(gca,'view') %得到视角（方位角和仰视角）。


A= cell(N, 6); 
for i = 1:N
%     A{i} = [R1(i) R2(i) R3(i) R4(i) R5(i) R6(i) R7(i) R8(i) R9(i)]; 
    A{i} = [0 0 0 R4(i) R5(i) R6(i)]; 
end

B=cell2mat(A);
T07=double(sixlink.fkine(B)); 
% T07=double(robot9.fkine(B)); 

% fig=scatter3(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),squeeze(T07(3,4,:)),1); %xyz三维图
% f = figure('Units','normalized','OuterPosition',[0 0.5 1, 0.5]); 
% figure('Position',[100, 100, 1000, 1000]); 

% ax1 = subplot(2,2,1);xlabel('x');ylabel('y');hold on;
% ax2 = subplot(2,2,2);ylabel('y');zlabel('z');hold on;
% ax3 = subplot(2,2,3);xlabel('x');zlabel('z');hold on;
ax4 = subplot(2,2,4);xlabel('x');ylabel('y');zlabel('z');hold on;
fig=scatter3(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),squeeze(T07(3,4,:)),1);
plotObjs =fig;
% copyobj(plotObjs,ax1);
% copyobj(plotObjs,ax2);
% copyobj(plotObjs,ax3);
copyobj(plotObjs,ax4);
% view(ax1,0,90); title(ax1,'top');
% view(ax2,90,0); title(ax2,'left');
% view(ax3,0,0);  title(ax3,'front');
view(ax4,-37.5,30);  title(ax4,'axon');


%% 蒙特卡洛 scatter3
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

%% 三视图
% f = figure('Units','normalized','OuterPosition',[0 0.5 1, 0.5]); %从屏幕左侧中间开始，按比例画图
% % figure('Position',[100, 100, 1000, 1000]); %固定窗口
% ax1 = subplot(2,2,1);xlabel('x');ylabel('y');hold on;
% ax2 = subplot(2,2,2);ylabel('y');zlabel('z');hold on;
% ax3 = subplot(2,2,3);xlabel('x');zlabel('z');hold on;
% ax4 = subplot(2,2,4);xlabel('x');ylabel('y');zlabel('z');hold on;
% fig=scatter3(squeeze(T07(1,4,:)),squeeze(T07(2,4,:)),squeeze(T07(3,4,:)),1);
% plotObjs =fig;

% Copy graphics objects and their descendants；复制图形对象及其后代
% copyobj(plotObjs,ax1);
% copyobj(plotObjs,ax2);
% copyobj(plotObjs,ax3);
% copyobj(plotObjs,ax4);
% view(ax1,0,90); title(ax1,'top');
% view(ax2,90,0); title(ax2,'left');
% view(ax3,0,0);  title(ax3,'front');
% view(ax4,-37.5,30);  title(ax4,'axon');

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

%% function
function T = forward_kine49(theta)% 虚拟六轴
    syms a8 a9 d7; %a8=50;a9=50; %d7=0;
    d=[0 0 0 0 0 0 0 0 0];      
    a=[0 0 0 0 0 0 0 a8 a9]; 
    alpha=[0 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0]; %不同的alpha[]
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(0,theta(7),a(7),alpha(7));
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T4*T5*T6*T7*T8*T9;
end

function T = DH_forward(theta,d,a,alpha) % 正运动函数
% T=transZ(0,0,d)*trotZ(theta)*transX(a,0,0)*trotX(alpha);
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end


