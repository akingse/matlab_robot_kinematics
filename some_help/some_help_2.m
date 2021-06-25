%% initialize
clc; close all; clear all;
format long; format compact;

theta=[0 0 0]; %绕Z 

%% 激光测量装置DH
% L(i) = Link([theta(1) d(1) a(1) alpha(1) 0],0);
L(1) = Link([theta(1) 100 0 pi/2 0],0);
L(2) = Link([theta(2) 0 0 -pi/2 0],0);
L(3) = Link([0 0 0 0 1],0);L(3).qlim = [0 2000];
robot_V6=SerialLink( L,'name','V6'); 
robot_V6.teach(theta); 


% T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
% syms th1 th2 d1 d3 x y z
T1=transl(0,0,d1)*trotz(th1)*trotx(pi/2);
T2=trotz(th2)*trotx(-pi/2);
T3=transl(0,0,d3);
T=T1*T2*T3
% [ cos(th1)*cos(th2), -sin(th1), -cos(th1)*sin(th2), -d3*cos(th1)*sin(th2)]
% [ cos(th2)*sin(th1),  cos(th1), -sin(th1)*sin(th2), -d3*sin(th1)*sin(th2)]
% [          sin(th2),         0,           cos(th2),      d1 + d3*cos(th2)]
% [                 0,         0,                  0,                     1]

d1=100;
x=500;
y=600;
z=700;

d3(1)=sqrt(x^2+y^2+(z-d1)^2);
d3(2)=-d3(1);
th2(1:2)=acos((z-d1)./d3);
th2(3:4)=-th2(1:2)
d3(3:4)=d3(1:2);
th1=atan2(-y./(d3.*sin(th2)),-x./(d3.*sin(th2)));
theta=[th1(1) th2(1) d3(1);
       th1(2) th2(2) d3(2);
       th1(3) th2(3) d3(3);
       th1(4) th2(4) d3(4)]


i=3;
T1=transl(0,0,d1)*trotz(theta(i,1))*trotx(pi/2);
T2=trotz(theta(i,2))*trotx(-pi/2);
T3=transl(0,0,theta(i,3));
T=T1*T2*T3

robot_V6.teach(theta(3,:)); 



% x==a2*cos(th1)*cos(th2)+d3*sin(th1)
% y==a2*sin(th1)*cos(th2)-d3*cos(th1)
% x-a2*cos(th1)*cos(th2)==d3*sin(th1) 
% y-a2*sin(th1)*cos(th2)==-d3*cos(th1)

% x^2+y^2+(a2*cos(th2))^2-2*x*a2*cos(th1)*cos(th2)-2*y*a2*sin(th1)*cos(th2)==d3^2
% (x^2+y^2+(a2*cos(th2))^2-d3^2)/2==x*a2*cos(th1)*cos(th2)+y*a2*sin(th1)*cos(th2)

% cos(th1)*(x-a2*cos(th1)*cos(th2))==d3*sin(th1)*cos(th1)
% sin(th1)*(y-a2*sin(th1)*cos(th2))==- d3*cos(th1)*sin(th1)
% 
% jud=cos(th1)*(x-a2*cos(th1)*cos(th2))+sin(th1)*(y-a2*sin(th1)*cos(th2))%==0
% % simplify(jud) %x*cos(th1) - a2*cos(th2) + y*sin(th1)==0
% x*cos(th1) + y*sin(th1) == a2*cos(th2)


% 如果实在不好解，诉诸一元二次方程吧；这样的话会产生更多的解，需要筛选；
% th1=atan2(x,y)+acos(a2*cos(th2)/sqrt(x^2+y^2))
