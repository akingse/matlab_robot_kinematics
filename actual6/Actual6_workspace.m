%% initialize
clc; close all; clear all;
format long; format compact;



%% 工作空间
% d5^2 + a2^2 + a3^2 + d4^2 + d6^2
syms d1 d4 d5 d6 a2 a3 %多元函数的二阶偏导
syms th3 th4 th5
th3=-3.14:0.01:3.14;
th4=-3.14:0.01:3.14;
th5=-3.14:0.01:3.14;
d4=90;d5=90;d6=90;a2=-420;a3=-400;
% p(th3,th4,th5)=
max=0;

for th3=-3.14:0.01:3.14;
%     for th4=-3.14:0.01:3.14;
%         for th5=-3.14:0.01:3.14;
%     p=2*a2*a3*cos(th3) + 2*d4*d6*cos(th5)+2*a3*d5*sin(th4) - 2*a3*d6*cos(th4).*sin(th5)+2*a2*d5*sin(th3+th4)-2*a2*d6*cos(th3+th4).*sin(th5);
% 
%     if p>max 
%         max=p;
%     end 
% end
% end
end

maxx=max+d5^2 + a2^2 + a3^2 + d4^2 + d6^2 %   5.4599e+05 %9.0669e+05=9.066905253510528e+05
mx=d5^2 + a2^2 + a3^2 +2*a2*a3 + d4^2 + d6^2 %+2*(-a2-a3)*sqrt(d5^2 + d6^2) %9.0544e+05
x=(sqrt(d5^2+d6^2)+sqrt((a2+a3)^2+d4^2))*d4/sqrt((a2+a3)^2+d4^2)
y=(a2+a3)*sqrt(d5^2+d6^2)/sqrt((a2+a3)^2+d4^2)
max2=(a2+a3+y)^2+x^2 %9.066914283964944e+05

% fx=diff(p,th3) %2*a2*d5*cos(th3 + th4) - 2*a2*a3*sin(th3) + 2*a2*d6*sin(th3 + th4)*sin(th5)
% fy=diff(p,th4) %2*a2*d5*cos(th3 + th4) + 2*a3*d5*cos(th4) + 2*a2*d6*sin(th3 + th4)*sin(th5) + 2*a3*d6*sin(th4)*sin(th5)
% fz=diff(p,th5) %- 2*d4*d6*sin(th5) - 2*a2*d6*cos(th3 + th4)*cos(th5) - 2*a3*d6*cos(th4)*cos(th5)
% m=max(p)

syms th1 th2 th3 th4 th5 th6;
syms d1 d4 d5 d6 a2 a3 a8 a9; 
    theta=[th1 th2 th3 th4 th5 th6];
    d=[d1 0 0 d4 d5 d6];
    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 0];
    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
% T=T1*T2*T3
px=a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3); %cos(th1)*(a3*cos(th2 + th3) + a2*cos(th2))
py=a2*cos(th2)*sin(th1) + a3*cos(th2)*cos(th3)*sin(th1) - a3*sin(th1)*sin(th2)*sin(th3); %sin(th1)*(a3*cos(th2 + th3) + a2*cos(th2))
pz=d1 + a2*sin(th2) + a3*cos(th2)*sin(th3) + a3*cos(th3)*sin(th2); %d1 + a3*sin(th2 + th3) + a2*sin(th2)
simplify(pz)
% simplify(px^2+py^2+(pz-d1)^2) %a2^2 + 2*cos(th3)*a2*a3 + a3^2
% (px^2+py^2+pz^2) %a2^2 + 2*cos(th3)*a2*a3 + 2*sin(th2)*a2*d1 + a3^2 + 2*sin(th2 + th3)*a3*d1 + d1^2


%     T=T4*T5*T6

% px= d5*sin(th4) - d6*cos(th4)*sin(th5)%
% py= - d5*cos(th4) - d6*sin(th4)*sin(th5)%
% pz= d4 + d6*cos(th5)%
% px= d5*sin(th4) - d6*cos(th4)*sin(th5);
% py= - d5*cos(th4) - d6*sin(th4)*sin(th5);
% pz= d6*cos(th5);
% simplify(px^2+py^2+(pz-d1)^2) %d5^2+d6^2 
%d4^2+d6^2+d5^2+2*d4*d6*cos(th5)

    
    
function T = DH_forward(theta,d,a,alpha)
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end
