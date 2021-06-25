%% initialize
clc; close all; clear all;
format long; format compact;

theta=[0 0 0 0 0 0]; %绕Z 

% L(i) = Link([theta(1) d(1) a(1) alpha(1) 0],0);
% L(1) = Link([theta(1) 0 0 pi/2 0],0); %virtual6的基坐标系变换 T_AV
% L(2) = Link([theta(2) 0 0 -pi/2 0],0);
% L(3) = Link([theta(3) 0 0 pi/2 0],0);
% L(4) = Link([theta(4) 0 0 0 0],0);
% L(5) = Link([theta(5) 0 0 pi/2 1],0); L(5).qlim = [0 2000];
% L(5) = Link([theta(5) d(5) a(5) alpha(5) 0],0); 
% L(6) = Link([theta(6) d(6) a(6) alpha(6) 0],0);
% robot_V6=SerialLink( L,'name','V6'); 
% robot_V6.display(); %DH表
% robot_V6.teach(); 


%% 二阶偏导
syms px py pz nx ny nz a9;
Px=px-a9*nx;
Py=py-a9*ny;
Pz=pz-a9*nz;
% P=Px^2+Py^2+Pz^2
% simplify(P)
% collect(P)
a=2;b=2;c=3;
% figure(1);
% ezsurf('x+y+10*x*y')
% ezsurf('sin(x)+sin(y)+sin(x)*sin(y)')
% view(90,0);
% ezsurf('2*sin(x)+cos(y)+2*sin(x)*cos(y)')
% view(0,0);
% hold on;
% figure(2);
% ezsurf('x^2-y^2');%标准马鞍面
th8=pi/2;th9=0; %z=5  A=-4 D=12
th8=pi/2;th9=pi; %z=-1  A=0 D=0
th8=-pi/2;th9=0; %z=-3  A=4 D=4
th8=-pi/2;th9=pi; %z=-1 A=0 D=0
% A*C-B^2>0且A<0极大，A>0极小；
% A*C-B^2<0没有极值
% A*C-B^2=0另作讨论

% syms th8 th9
% p(th8,th9)=2*sin(th8)+cos(th9)+2*sin(th8)*cos(th9)
% a=diff(p,th8);%
a=2*cos(th8) + 2*cos(th8)*cos(th9);
% A=diff(a,th8);%
A=- 2*sin(th8) - 2*cos(th9)*sin(th8);
% c=diff(p,th9);%
c=- sin(th9) - 2*sin(th8)*sin(th9);
% C=diff(c,th9);%
C=- cos(th9) - 2*cos(th9)*sin(th8);
% b=diff(p,th8);%
b=2*cos(th8) + 2*cos(th8)*cos(th9);
% B=diff(b,th9);%
B=-2*cos(th8)*sin(th9);


% A*C-B^2
% D=(cos(th9) + 2*cos(th9)*sin(th8))*(2*sin(th8) + 2*cos(th9)*sin(th8)) - 4*cos(th8)^2*sin(th9)^2
% a=cos(th8)*cos(th4)*sin(th5) + sin(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6));
% b=cos(th8)*sin(th4)*sin(th5) - sin(th8)*(cos(th4)*sin(th6)+sin(th4)*cos(th5)*cos(th6));
% c=cos(th8)*cos(th5) + sin(th8)*sin(th5)*cos(th6);
% d=a^2+b^2+c^2

% syms th4
% simplify(cos(th4-pi/2))
% simplify(sin(th4-pi/2))
