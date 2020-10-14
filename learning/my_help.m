%% initialize
clc; close all; clear all;
format shortg; format compact;
%% shang
l=8; q=10000; F=20000; M=40000; E=200e9; I=2e-6;
N2= (F*l/4+3*q*l*l/8-M) /l;
N1=q*l/2+F-N2;
x=linspace (0, l, 100) ;dx=l/100;
M1=N1*x (1:25) ;
M2=N1*x (26:50) -F* (x (26:50) -0.25*l) ;
M3=N2*x (51:100) -q*x (51:100) .^2/2;
M=[M1, M2, M3];
A0=cumtrapz (M) *dx/ (E*I) ;
Y0=cumtrapz (A0) *dx;
C=[0, 1;l, 1]\[-Y0 (1) ;-Y0 (101) ];
A=A0+C (1) ;Y=Y0+C (1) *x+C (2) ;
% subplot (3, 1, 1) ;plot (x, M) ;grid on
% subplot (3, 1, 2) ;plot (x, A) ;grid on
% subplot (3, 1, 3) ;plot (x, Y) ;grid on

% syms x y;
% [x,y]=solve(x+y==3,2*x+y==10,x,y)

%% yang
% syms v1 v2;
% m=0.27;M=3.6;g=9.8;
% F=9;n=8;c=1.03;theta=30*pi/180;h=0.4;
% 
% a=m*g*h+n*F*c*sin(theta)*sin(theta)-M*g*c*sin(theta);
% b=1/2*m*v1^2+1/2*M*v2^2;
% c=m*sqrt(2*g*h);
% d=m*v1+M*v2;
% [v1,v2]=solve(a==b,c==d,v1,v2)
% 
% vpa([v1,v2],6)
% h1=vpa(v1.^2/2/g,6)

%% junjie
a=pi/4:0.01:pi/2;
F=14700./(sin(a)-0.15*cos(a));
figure('NumberTitle','off','Name', 'F=14700/(sin(a)-0.15*cos(a))');
plot(a,F);
xlabel('rad');ylabel('F/kN');title('sin(x)');



%% À×ÎÂ·¨-che
r1=18;
r2=25;
r3=pi:0.01:3*pi; %v3=1;
v3=sin(r3/4+1/4);
r4=20;
r5=50;

a2=(r4^2-r1^2-r2^2-r3.^2)/sqrt((2*r1*r2)^2+(2*r2*r3).^2);
b2=r3/r2;
theta2=acos(a2)-atan(b2);

a4=(r1^2+r3.^2+r4^2-r2^2)/sqrt((2*r1*r4)^2+(2*r3*r4).^2);
b4=r3/r1;
theta4=acos(a4)-atan(b4);
theta5=theta2+0.1;

u=r2*cos(theta2)-r2*sin(theta2).*cos(theta4)./sin(theta4);
omega2=v3./u;

Vx=-r5*omega2.*sin(theta5);
Vy=r5*omega2.*cos(theta5);
V=sqrt(Vx.^2+Vy.^2);


figure(2);% title('V');
plot(r3,Vx,'-r');
hold on;
plot(r3,Vy,'--');
hold on;
plot(r3,V,':k');
xlabel('x')
ylabel('y')
legend('Vx','Vy','V')

