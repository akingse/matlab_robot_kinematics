%% initialize
clc; close all; clear all;
format shortg; format compact;
%% old calculate version
% modeling1 旧版本的计算草稿
% inv(T1)*T1-eye(4) %cos(x)^2+sin(x)^2!=1;
% r = simple(f);
% T1i=[cos(th1) sin(th1) 0 0;-sin(th1) cos(th1) 0 0;0 0 1 0;0 0 0 1];%inv(T1)
% T2i=[cos(th2) 0 sin(th2) -100*sin(th2);-sin(th1) 0 cos(th1) -100*cos(th2);0 -1 0 0;0 0 0 1];%inv(T2)
% T3i=[-sin(th3) cos(th3) 0 -480*cos(th3);-cos(th3) -sin(th3) 0 480*sin(th3);0 0 1 -120;0 0 0 1];%inv(T3)

% syms nx ox ax px ny oy ay py nz oz az pz;
% % nx=1; ox=0; ax=0; px=0;ny=0; oy=1; ay=0; py=200;nz=0; oz=0; az=1; pz=1080;
% T0=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];%给定任意位姿；
% T1i*T0==T26
% T2i*T1i*T0==T36
% T3i*T2i*T1i*T0==T46
% T26i=T1i*T0;
% T36i=T2i*T1i*T0;
% T46i=T3i*T2i*T1i*T0;

% T26i(2,1),T26i(2,2),T26i(2,3),T26i(2,4)
% T26(2,1),T26(2,2),T26(2,3),T26(2,4)
% % syms phi r;%phi=φ
% r=sqrt(px^2+py^2);
% phi=atan2(py,px);

% e1=T1i(1,1)*px+T1i(1,2)*py+T1i(1,3)*pz;
% e2=T1i(2,1)*px+T1i(2,2)*py+T1i(2,3)*pz;
% e3=T1i(3,1)*px+T1i(3,2)*py+T1i(3,3)*pz;
% d1=T26(1,4);
% d2=T26(2,4);
% d3=T26(3,4);
% solve(e2==d2,th1) %th5=0;%=>th1=0;

% [th2,th3]=solve(e1==d1,e3==d3)
% [th2,th3, params, conds] = solve(e1==d1,e3==d3, 'return',1)
% syms x y;
% x=solve(x+y==3,x-y==-1,x)
% T36=T3*T4*T5*T6;
% T2i=inv(T1)*inv(T2);
% 
% % syms phi r;
% r=sqrt(px^2+py^2);
% phi=atan2(py,px);

% e1=T2i(1,1)*px+T2i(1,2)*py+T2i(1,3)*pz;
% e2=T2i(2,1)*px+T2i(2,2)*py+T2i(2,3)*pz;
% e3=T2i(3,1)*px+T2i(3,2)*py+T2i(3,3)*pz;
% d1=T36(1,4);
% d2=T36(2,4);
% d3=T36(3,4);

%% AK5 
% syms a2 a3 d2 d4
% 
% T10=trotz(th1);
% T11=transl(0,120,100);
% T12=trotx(pi/2);
% T20=trotz(th2);
% T21=transl(0,480,120);
% T30=trotz(th3);
% T31=transl(0,400,-100);
% T40=trotz(th4);
% T41=transl(0,100,0);
% T42=trotx(-pi/2);
% T50=trotz(th5);
% T51=transl(0,100,0);
% T52=trotx(-pi/2);
% T60=trotz(th6);
% % 
% T1=T10*T11*T12
% T1i=invtrot(T12)*invtrans(T11)*invtrot(T10);
% T2=T20*T21
% T2i=invtrans(T21)*invtrot(T20);
% T3=T30*T31
% T3i=invtrans(T31)*invtrot(T30);
% T4=T40*T41*T42
% T4i=invtrot(T42)*invtrans(T41)*invtrot(T40);
% T5=T50*T51*T52
% T5i=invtrot(T52)*invtrans(T51)*invtrot(T50);
% T6=T60
% T6i=invtrot(T60);

% T=T10*T11*T12*T20*T21*T30*T31*T40*T41*T42*T50*T51*T52*T60;
% T0=[1 0 0 0;0 0 1 200;0 -1 0 1080;0 0 0 1];
% T=T1*T2*T3*T4*T5*T6;
% T1i*T;
% T2*T3*T4*T5*T6;
%% new modeling2
% T26=simplify(T2*T3*T4*T5*T6);  
% T26i=invtrot(T1)*Tq;
% T36=simplify(T3*T4*T5*T6)  %T36i=inv(T2)*T26i, 
% T36i=invtrot(trotz(th2))*invtrot(trotx(-pi/2))*invtrans(transl(0,y2,z2))*T26i;
% T46=simplify(T4*T5*T6);
% T46i=invtrot(trotz(th3))*invtrans(transl(0,y3,z3))*T36i;
% T56=simplify(T5*T6);
% T56i=invtrot(trotz(th4))*invtrans(transl(0,y4,z4))*T46i;

%solve the function
% T26(2,3)==cos(th5);
% T26(2,4)==y2+z3+z4+y6*cos(th5);
% T26i(2,3)==ay*cos(th1)-ax*sin(th1);
% T26i(2,4)==py*cos(th1)-px*sin(th1);
% y*cos(th1)-px*sin(th1)-y6*(ay*cos(th1)-ax*sin(th1))==y2+z3+z4==k;
% cos(phi)=py-y6*ax;sin(phi)=px-y6*ay;
% r=sqrt((py-y6*ax)^2+(px-y6*ay)^2);
% phi=atan2(px-y6*ay,py-y6*ax);
% theta=atan2(k,sqrt(r^2-k^2));
% th1=phi+theta
% th1=phi-theta
%
% T36(3,3)==cos(th5);
% T36(3,4)==z3+z4+y6*cos(th5);
% T36i(3,3)==ay*cos(th1)-ax*sin(th1)
% T36i(3,4)==py*cos(th1)-y2-px*sin(th1)
% [th1,th5]=solve(T26(2,3)-T26i(2,3),T26(2,4)-T26i(2,4),th1,th5,'Real',true)%返回一个实解
% [th1,th5]=solve(T26(2,3)-T26i(2,3),T26(2,4)-T26i(2,4),th1,th5,'IgnoreAnalyticConstraints',true)%应用简化规则

% r1=sqrt(px^2+py^2);% phi1=atan(py/px);
% r2=sqrt(ax^2+ay^2);% phi2=atan(ay/ax);
% y=r2*sin(phi2-th1)-y6*r1*sin(phi1-th1)-k;
% [th1]=solve(y,th1,'Real',true)
% format short;
% % %test of matrix of puma560

% T1=trotz(pi/2)*trotz(th1);%
% % T1=trotz(th1);
% T2=transl(0,d2,0)*trotx(-pi/2)*trotz(th2);
% T3=transl(a2,0,0)*trotz(-pi/2)*trotz(th3);%
% % T3=transl(a2,0,0)*trotz(th3);
% T4=transl(a3,d4,0)*trotx(-pi/2)*trotz(th4);
% T5=transl(0,0,0)*trotx(pi/2)*trotz(th5);
% T6=transl(0,0,0)*trotx(-pi/2)*trotz(th6);
% T=T1*T2*T3*T4*T5*T6;
% T=T2*T3*T4*T5*T6
% simplify(T) ;

%% UR_noap
syms th1 th2 th3 th4 th5 th6 th8 th9 d1 d4 d5 d6 a1 a2 a3 a8 a9 d7;
syms nx ox ax px ny oy ay py nz oz az pz;
% T=(cos(th6)*(sin(th1)*sin(th5)+cos(th2+th3+th4)*cos(th1)*cos(th5))-sin(th2+th3+th4)*cos(th1)*sin(th6))*sin(th1)-...
% (-cos(th6)*(cos(th1)*sin(th5)-cos(th2+th3+th4)*cos(th5)*sin(th1))-sin(th2+th3+th4)*sin(th1)*sin(th6))*cos(th1);
% T=(-sin(th6)*(sin(th1)*sin(th5)+cos(th2+th3+th4)*cos(th1)*cos(th5))-sin(th2+th3+th4)*cos(th1)*cos(th6))*sin(th1)-...
% (sin(th6)*(cos(th1)*sin(th5)-cos(th2+th3+th4)*cos(th5)*sin(th1))-sin(th2+th3+th4)*cos(th6)*sin(th1))*cos(th1);
% T=d6*(-cos(th1)*cos(th5)-cos(th2+th3+th4)*sin(th1)*sin(th5))-a2*cos(th2)*sin(th1)-d4*cos(th1)-d6*(cos(th1)*cos(th5)+cos(th2+th3+th4)*sin(th1)*sin(th5))+d5*sin(th2+th3+th4)*sin(th1)+a3* sin(th1)*cos(th2+th3)
% T=d6*(cos(th5)*sin(th1)-cos(th2+th3+th4)*cos(th1)*sin(th5))-d6*(cos(th5)*sin(th1)-cos(th2+th3+th4)*cos(th1)*sin(th5))+d4*sin(th1)+a2*cos(th1)*cos(th2)+d5*sin(th2+th3+th4)*cos(th1)+a3*cos(th1)*cos(th2+th3)
% T=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))-d5*sin(th2+th3+th4)
% T=(cos(th2+th3+th4)*sin(th6)+sin(th2+th3+th4)*cos(th5)*cos(th6))*sin(th6)-(cos(th2+th3+th4)*cos(th6)-sin(th2+th3+th4)*cos(th5)*sin(th6))*cos(th6)
% th1=0; th2=0; th3=0; th4=0; th5=0; th6=0;
% nx=cos(th6)*(sin(th1)*sin(th5)+cos(th2+th3+th4)*cos(th1)*cos(th5))-sin(th2+th3+th4)*cos(th1)*sin(th6);
% ny=-cos(th6)*(cos(th1)*sin(th5)-cos(th2+th3+th4)*cos(th5)*sin(th1))-sin(th2+th3+th4)*sin(th1)*sin(th6);
% nz=cos(th2+th3+th4)*sin(th6)+sin(th2+th3+th4)*cos(th5)*cos(th6);
% ox=-sin(th6)*(sin(th1)*sin(th5)+cos(th2+th3+th4)*cos(th1)*cos(th5))-sin(th2+th3+th4)*cos(th1)*cos(th6);
% oy=sin(th6)*(cos(th1)*sin(th5)-cos(th2+th3+th4)*cos(th5)*sin(th1))-sin(th2+th3+th4)*cos(th6)*sin(th1);
% oz=cos(th2+th3+th4)*cos(th6)-sin(th2+th3+th4)*cos(th5)*sin(th6);
% ax=cos(th5)*sin(th1)-cos(th2+th3+th4)*cos(th1)*sin(th5);
% ay=-cos(th1)*cos(th5)-cos(th2+th3+th4)*sin(th1)*sin(th5);
% az=-sin(th2+th3+th4)*sin(th5);
% 
% px=a2*cos(th1)*cos(th2)+a3*cos(th1)*cos(th2+th3)+d4*sin(th1)+d6*(cos(th5)*sin(th1)-cos(th2+th3+th4)*cos(th1)*sin(th5))+d5*sin(th2+th3+th4)*cos(th1);
% py=a2*cos(th2)*sin(th1)+a3*sin(th1)*cos(th2+th3)-d4*cos(th1)-d6*(cos(th1)*cos(th5)+cos(th2+th3+th4)*sin(th1)*sin(th5))+d5*sin(th2+th3+th4)*sin(th1);
% pz=d1+d5*(sin(th2+th3)*sin(th4)-cos(th2+th3)*cos(th4))+a3*sin(th2+th3)+a2*sin(th2)-d6*sin(th5)*(cos(th2+th3)*sin(th4)+sin(th2+th3)*cos(th4));

% px*ny-py*nx;
% pz-d1=a2*sin(th2)+a3*sin(th2+th3)-d5*cos(th2+th3+th4)-d6*sin(th2+th3+th4)*sin(th5);
% R=[nx ox ax; ny oy ay; nz oz az]
% [theta,v]=tr2angvec(R)
% nz=ox*ay-oy*ax;
% oz=-(nx*ay-ny*ax);
% az=nx*oy-ny*ox;
% T=-sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ox*cos(th1)+oy*sin(th1))
% T=nz*sin(th6)+oz*cos(th6); 
% m3=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6)*(nx*cos(th1)+ny*sin(th1))+cos(th6)*(ox*cos(th1)+oy*sin(th1)));
% n6=nx*sin(th1)-ny*cos(th1)
% n234=oz*cos(th6)+nz*sin(th6)
% q=d6*(ax*cos(th1)+ay*sin(th1))
% T=simplify(q)
% a3*cos(th2 + th3)*sin(th1) - d4*cos(th1) - 2*d6*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) - a2*cos(th2)*sin(th1) + d5*sin(th2 + th3 + th4)
% d4*sin(th1) + a3*cos(th2 + th3)*cos(th1) + a2*cos(th1)*cos(th2) + d5*sin(th2 + th3 + th4)*cos(th1)
% n6=ny*cos(th1)-nx*sin(th1); 
% m6=oy*cos(th1)-ox*sin(th1);

m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)));
n23=pz-d1-d6*az+d5*(nz*sin(th6)+oz*cos(th6)); 

mn=m23^2+n23^2; %a2^2 + 2*cos(th3)*a2*a3 + a3^2

% T=simplify(mn)
% U=simplify(n23)
% m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)));
% n23=-d1+pz-d6*az+d5*(nz*sin(th6)+oz*cos(th6));
% T=simplify(m23^2+n23^2) %a2^2+2*cos(th3)*a2*a3+a3^2
%% modeling3
syms th1 th2 th3 th4 th5 th6 nx ox ax px ny oy ay py nz oz az pz;
Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];Q=[];
syms y1 y2 y3 y4 z1 z2 z3 z5;
% y1=120;y2=-480;y3=-400;y4=-100;z1=100;z2=-120;z3=100;z5=100;
% th1=0;th2=0;th3=0;th4=0;th5=0;th6=0;
% nx=1; ox=0; ax=0; px=0;ny=0; oy=0; ay=1; py=200;nz=0; oz=-1; az=0; pz=1080;
% th1=0;th2=pi/6;th3=pi/3;th4=-pi/2;th5=-pi/2;th6=0;
% th1=pi/3;th2=pi/2;th3=-pi/2;th4=-pi/2;th5=-pi/3;th6=pi/4;
% th1=-pi/6;th2=pi/4;th3=pi/3;th4=pi/3;th5=pi/3;th6=pi/4;
% th1=-pi/2;th2=-pi/6;th3=-pi/3;th4=pi/3;th5=2*pi/3;th6=pi/4;

% T1=trotz(th1)*transl(0,y1,z1)*trotx(-pi/2);
% T2=trotz(th2)*transl(0,y2,z2);
% T3=trotz(th3)*transl(0,y3,z3);
% T4=trotz(th4)*transl(0,y4,0);
% T5=troty(th5)*transl(0,0,z5);
% T6=trotz(th6);
T=T1*T2*T3*T4*T5*T6;
% T13=simplify(T1*T2*T3);
% T46=simplify(T4*T5*T6);
% simplify(T13*T46);
T16=simplify(T1*T2*T3*T4*T5*T6);
T26=simplify(T2*T3*T4*T5*T6);
T25=simplify(T2*T3*T4*T5);
T25i=invtrot(trotx(-pi/2))*invtrans(transl(0,y1,z1))*invtrot(trotz(th1))*Tq*invtrot(trotz(th6));

% T26i=invtrot(trotx(-pi/2))*invtrans(transl(0,y1,z1))*invtrot(trotz(th1))*Tq;
% T36=simplify(T3*T4*T5*T6);
% T36i=invtrans(transl(0,y2,z2))*invtrot(trotz(th2))*T26i;
% T46=simplify(T4*T5*T6)
% T46i=invtrans(transl(0,y3,z3))*invtrot(trotz(th3))*T36i
% % [th1,th2]=solve(y3*sin(th1)+y2*sin(th2)-k1,y3*cos(th1)+y2*cos(th2)-k2,th1,th2,'Real',true);

% -------------------------------------------------------------------------
% cos(th5) = ay*cos(th1)-ax*sin(th1);
% z2+z3+z5*cos(th5) = py*cos(th1)-y1-px*sin(th1);
% y1+z2+z3 = (py-ay*z5)*cos(th1)-(px-ax*z5)*sin(th1);
% sin(phi)=(py-ay*z5)/r;cos(phi)=(px-ax*z5)/r;
phi=atan2(py-ay*z5,px-ax*z5);r=sqrt((py-ay*z5)^2+(px-ax*z5)^2);k=y1+z2+z3;
% y1+z2+z3 = r*sin(phi-th1);theta=phi-th1;
% sin(phi-th1)=k/r;cos(phi-th1)=±sqrt(1-k^2/r^2);
theta=atan2(k,sqrt(r^2-k^2)); %theta=±atan2(k,sqrt(r^2-k^2));
th11=phi+theta; %error; th11=atan2(py-ay*z5,px-ax*z5)-atan2(k,sqrt(r^2-k^2));
th12=phi-theta; %th11=atan2(py-ay*z5,px-ax*z5)-atan2(k,-sqrt(r^2-k^2));
Q(1,1)=th11*180/pi;Q(5,1)=th12*180/pi;
Q(2,1)=Q(1,1);Q(3,1)=Q(1,1);Q(4,1)=Q(1,1);Q(6,1)=Q(5,1);Q(7,1)=Q(5,1);Q(8,1)=Q(5,1);

% cos(th5)=ay*cos(th1)-ax*sin(th1);sin(th5)=±sqrt(1-(ay*cos(th1)-ax*sin(th1))^2);
a=ay*cos(th11)-ax*sin(th11);b=sqrt(1-(ay*cos(th11)-ax*sin(th11))^2);
c=ay*cos(th12)-ax*sin(th12);d=sqrt(1-(ay*cos(th12)-ax*sin(th12))^2);
th52=-atan2(b,a);%th51=atan2(b,a);
th54=-atan2(d,c);%th53=atan2(d,c);
Q(1,5)=th52*180/pi;Q(5,5)=th54*180/pi;
Q(2,5)=Q(1,5);Q(3,5)=Q(1,5);Q(4,5)=Q(1,5);Q(6,5)=Q(5,5);Q(7,5)=Q(5,5);Q(8,5)=Q(5,5);

% -cos(th6)*sin(th5) = ny*cos(th1)-nx*sin(th1);⑨ %th5≠0;
% sin(th5)*sin(th6) = oy*cos(th1)-ox*sin(th1);⑩
% -tan(th6)=(oy*cos(th1)-ox*sin(th1))/(ny*cos(th1)-nx*sin(th1));
th61=-atan2(oy*cos(th11)-ox*sin(th11),ny*cos(th11)-nx*sin(th11));
th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12));
Q(1,6)=th61*180/pi;Q(5,6)=th62*180/pi;
Q(2,6)=Q(1,6);Q(3,6)=Q(1,6);Q(4,6)=Q(1,6);Q(6,6)=Q(5,6);Q(7,6)=Q(5,6);Q(8,6)=Q(5,6);
% cos(th2+th3+th4)*sin(th5) = ax*cos(th1)+ay*sin(th1)  ③
% sin(th2+th3+th4)*sin(th5) = -az  ⑦
% tan(th2+th3+th4)=-az/(ax*cos(th1)+ay*sin(th1));
thsum1=atan2(-az,ax*cos(th11)+ay*sin(th11));%thsum=th2+th3+th4;
thsum2=atan2(-az,ax*cos(th12)+ay*sin(th12));

% -y3*sin(th2+th3)-y2*sin(th2)+z5*sin(th5)*cos(th2+th3+th4)-y4*sin(th2+th3+th4)=px*cos(th1)+py*sin(th1)  ④
% y3*cos(th2+th3)+y2*cos(th2)+z5*sin(th5)* sin(th2+th3+th4)+y4*cos(th2+th3+th4)=z1-pz  ⑧
% y3*sin(th2+th3)+y2*sin(th2) = z5*(ax*cos(th1)+ay*sin(th1))+y4*az/sin(th5)-(px*cos(th1)+py*sin(th1));   
% y3*cos(th2+th3)+y2*cos(th2) = z5*az+y4*(ax*cos(th1)+ay*sin(th1))/sin(th5)+z1-pz; 
k1=z5*(ax*cos(th11)+ay*sin(th11))+y4*az/sin(th52)-(px*cos(th11)+py*sin(th11));
k2=z5*az-y4*(ax*cos(th11)+ay*sin(th11))/sin(th52)+z1-pz;
% k3=z5*(ax*cos(th12)+ay*sin(th12))+y4*az/sin(th54)-(px*cos(th12)+py*sin(th12));
% k4=z1-pz+z5*az-y4*(ax*cos(th12)+ay*sin(th12))/sin(th54);
% y3*sin(th2+th3)+y2*sin(th2) = k1;
% y3*cos(th2+th3)+y2*cos(th2) = k2;
k5=(k1^2+k2^2-y2^2-y3^2)/(2*y2*y3); %k6=(k3^2+k4^2-y2^2-y3^2)/(2*y2*y3)
% cos(th3)=k;sin(th3)=±sqrt(1-k^2);
th31=atan2(sqrt(1-k5^2),k5);
th32=-atan2(sqrt(1-k5^2),k5);
Q(1,3)=th31*180/pi;Q(3,3)=th32*180/pi;
Q(2,3)=Q(1,3);Q(5,3)=Q(1,3);Q(6,3)=Q(1,3);Q(4,3)=Q(3,3);Q(7,3)=Q(3,3);Q(8,3)=Q(3,3);

% sin(phi)=2*k2*y2/r;cos(phi)=2*k1*y2/r;
phi=atan2(2*k2*y2,2*k1*y2);%phi=atan2(k2,k1);
r=sqrt((2*k2*y2)^2+(2*k1*y2)^2);
k=(k1^2+k2^2+y2^2-y3^2)/r;
theta=atan2(k,sqrt(1-k^2));
th21=theta+phi;
th22=-theta+phi;
Q(1,2)=th21*180/pi;Q(2,2)=th22*180/pi;
Q(3,2)=Q(1,2);Q(5,2)=Q(1,2);Q(7,2)=Q(1,2);Q(4,2)=Q(2,2);Q(6,2)=Q(2,2);Q(8,2)=Q(2,2);

Q(1,4)=(thsum1-th31-th21+pi)*180/pi; %th41
Q(2,4)=(thsum1-th31-th22+pi)*180/pi;
Q(3,4)=(thsum1-th32-th21+pi)*180/pi;
Q(4,4)=(thsum1-th32-th22+pi)*180/pi;
Q(5,4)=(thsum2-th31-th21+pi)*180/pi;
Q(6,4)=(thsum2-th31-th22+pi)*180/pi;
Q(7,4)=(thsum2-th32-th21+pi)*180/pi;
Q(8,4)=(thsum2-th32-th22+pi)*180/pi;
Q
%% new modeling4
syms th1 th2 th3 th4 th5 th6 nx ox ax px ny oy ay py nz oz az pz;
Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];global Q;
syms y1 y2 y3 y4 z1 z2 z3 z5;
y1=120;y2=-480;y3=-400;y4=-100;z1=100;z2=-120;z3=100;z5=100;
% Q0=[0 0 0 0 0 0]
% Q0=[60 90 -90 -90 -60 -45]
% Q0=[0 30 60 90 120 45]
Q0=[30 45 60 70 80 90]
th1=Q0(1)/180*pi;th2=Q0(2)/180*pi;th3=Q0(3)/180*pi;
th4=Q0(4)/180*pi;th5=Q0(5)/180*pi;th6=Q0(6)/180*pi;
T1=trotz(th1)*transl(0,y1,z1)*trotx(-pi/2);
T2=trotz(th2)*transl(0,y2,z2);
T3=trotz(th3)*transl(0,y3,z3);
T4=trotz(th4)*transl(0,y4,0);
T5=troty(th5)*transl(0,0,z5);
T6=trotz(th6);
T=T1*T2*T3*T4*T5*T6
nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
% T26=simplify(T2*T3*T4*T5*T6);
% T26i=invtrot(trotx(-pi/2))*invtrans(transl(0,y1,z1))*invtrot(trotz(th1))*Tq;
% T25=simplify(T2*T3*T4*T5);
% T25i=invtrot(trotx(-pi/2))*invtrans(transl(0,y1,z1))*invtrot(trotz(th1))*Tq*invtrot(trotz(th6));
%% modeling3.1;
% phi=atan2(py-ay*z5,px-ax*z5);rth1=sqrt((py-ay*z5)^2+(px-ax*z5)^2);
% kth1=y1+z2+z3;
% theta=atan2(kth1,sqrt(rth1^2-kth1^2));%judge:(r^2-k^2)>0;
% th11=(phi+theta)*180/pi
% th12=phi-theta; 
% for i=1:8
%     Q(i,1)=th12*180/pi;
% end
% 
% c12=ay*cos(th12)-ax*sin(th12);s12=sqrt(1-(ay*cos(th12)-ax*sin(th12))^2); 
% th53=atan2(s12,c12); 
% th54=-th53; 
% for i=1:4
%     Q(i,5)=th53*180/pi;Q(i+4,5)=th54*180/pi;
% end
% y6=oy*cos(th12)-ox*sin(th12);
% x6=ny*cos(th12)-nx*sin(th12);
% 
% if (oy*cos(th12)-ox*sin(th12)>=0)&(ny*cos(th12)-nx*sin(th12)>=0) %①
%     th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12))+pi;
% elseif (oy*cos(th12)-ox*sin(th12)>0)&(ny*cos(th12)-nx*sin(th12)<0) %②
%     th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12))+pi;
% elseif (oy*cos(th12)-ox*sin(th12)<0)&(ny*cos(th12)-nx*sin(th12)<0) %③
%     th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12));
% elseif (oy*cos(th12)-ox*sin(th12)<0)&(ny*cos(th12)-nx*sin(th12)>0) %④
%     th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12));
% end
% for i=1:8
%     Q(i,6)=th62*180/pi; 
% end
% %-----------------------------------------------------------------------------
% ysum=-az;
% xsum=ax*cos(th12)+ay*sin(th12);
% 
% if (-az>=0)&(ax*cos(th12)+ay*sin(th12)>=0) %①
%     thsum2=atan2(-az,ax*cos(th12)+ay*sin(th12))-pi;
% elseif (-az>0)&(ax*cos(th12)+ay*sin(th12)<0) %②
%     thsum2=atan2(-az,ax*cos(th12)+ay*sin(th12));
% elseif (-az<0)&(ax*cos(th12)+ay*sin(th12)<0) %③
%     thsum2=atan2(-az,ax*cos(th12)+ay*sin(th12))+pi;
% elseif (-az<0)&(ax*cos(th12)+ay*sin(th12)>0) %④
%     thsum2=atan2(-az,ax*cos(th12)+ay*sin(th12))+pi;
% end
% 
% k5=z5*(ax*cos(th12)+ay*sin(th12))+y4*az/sin(th53)-(px*cos(th12)+py*sin(th12)); 
% k6=z5*az-y4*(ax*cos(th12)+ay*sin(th12))/sin(th53)+z1-pz;
% k7=z5*(ax*cos(th12)+ay*sin(th12))+y4*az/sin(th54)-(px*cos(th12)+py*sin(th12));
% k8=z5*az-y4*(ax*cos(th12)+ay*sin(th12))/sin(th54)+z1-pz;
% 
% kth33=(k5^2+k6^2-y2^2-y3^2)/(2*y2*y3);
% kth34=(k7^2+k8^2-y2^2-y3^2)/(2*y2*y3);
% r3=sqrt((2*k5*y2)^2+(2*k6*y2)^2);phi3=atan2(2*k6*y2/r3,2*k5*y2/r3);
% r4=sqrt((2*k7*y2)^2+(2*k8*y2)^2);phi4=atan2(2*k8*y2/r4,2*k7*y2/r4);
% kth23=(k5^2+k6^2+y2^2-y3^2)/r3; 
% kth24=(k7^2+k8^2+y2^2-y3^2)/r4;
% 
% if (kth33<=1)&(kth23<=1)
%     th35=atan2(sqrt(1-kth33^2),kth33);
%     th36=-th35;
%     theta=atan2(kth23,sqrt(1-kth23^2));
%     th25=theta-phi3;
%     th26=-theta-phi3;
% else
%     th35=NaN;th36=NaN;th25=NaN;th26=NaN;
% end
% if (kth34<=1)&(kth24<=1)
%     th37=atan2(sqrt(1-kth34^2),kth34);
%     th38=-th37;
%     theta=atan2(kth24,sqrt(1-kth24^2));
%     th27=theta-phi4;
%     th28=-theta-phi4+pi;
% else
%     th37=NaN;th38=NaN;th27=NaN;th28=NaN;
% end
% 
% Q(1,3)=th35*180/pi;Q(2,3)=Q(1,3);
% Q(3,3)=th36*180/pi;Q(4,3)=Q(3,3);
% Q(5,3)=th37*180/pi;Q(6,3)=Q(5,3);
% Q(7,3)=th38*180/pi;Q(8,3)=Q(7,3);
% Q(1,2)=th25*180/pi;Q(3,2)=Q(1,2);
% Q(2,2)=th26*180/pi;Q(4,2)=Q(2,2);
% Q(5,2)=th27*180/pi;Q(7,2)=Q(5,2);
% Q(6,2)=th28*180/pi;Q(8,2)=Q(6,2);
% 
% Q(1,4)=(thsum2-th35-th25)*180/pi; 
% Q(2,4)=(thsum2-th35-th26)*180/pi;
% Q(3,4)=(thsum2-th36-th25)*180/pi;
% Q(4,4)=(thsum2-th36-th26)*180/pi;
% Q(5,4)=(thsum2-th37-th27)*180/pi;
% Q(6,4)=(thsum2-th37-th28)*180/pi;
% Q(7,4)=(thsum2-th38-th27)*180/pi;
% Q(8,4)=(thsum2-th38-th28)*180/pi;
% Q
% for i=1:8;
%     Qt=verify(i);
%     if (abs(Qt(1,4)-T(1,4))<1)&(abs(Qt(2,4)-T(2,4))<1)&(abs(Qt(3,4)-T(3,4))<1)
%         for j=1:6
%             S(j)=Q(i,j);
%         end
%         eval(['S',num2str(i),'=','S']);
%         eval(['Q',num2str(i),'=','verify(i)']);
%     end
% end    
% % Q1=verify(1)
% % Q8=verify(8)
%% modeling2
syms th1 th2 th3 th4 th5 th6 nx ox ax px ny oy ay py nz oz az pz;
Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];global Q;
syms y2 y3 y4 y5 y6 z2 z3 z4;
Q1=[30 45 60 70 80 90]
Q0=Q1/180*pi;
% Q0=[0 30 -60 0 0 0]
th1=Q0(1);th2=Q0(2);th3=Q0(3);
th4=Q0(4);th5=Q0(5);th6=Q0(6);
y2=120;y3=-480;y4=-400;y5=-100;y6=100;z2=100;z3=-120;z4=100;

T1=trotz(th1);
T2=transl(0,y2,z2)*trotx(-pi/2)*trotz(th2);
T3=transl(0,y3,z3)*trotz(th3);
T4=transl(0,y4,z4)*trotz(th4);
T5=transl(0,y5,0)*trotx(pi/2)*trotz(th5);
T6=transl(0,y6,0)*trotx(-pi/2)*trotz(th6);
T=T1*T2*T3*T4*T5*T6
nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
% T16=T1*T2*T3*T4*T5*T6;
% T16=simplify(T1*T2*T3*T4*T5*T6);
% T25=simplify(T2*T3*T4*T5);
% T25i=invtrot(trotz(th1))*Tq*invtrot(trotz(th6))*invtrot(trotx(-pi/2))*invtrans(transl(0,y6,0));

% -------------------------------------------------------------------------
% py*cos(th1)-px*sin(th1)-y6*(ay*cos(th1)-ax*sin(th1)) = y2+z3+z4 ⑧ => th1
% (py-y6*ay)*cos(th1)-(px-y6*ax)*sin(th1)=y2+z3+z4;
% r1*sin(phi1)*cos(th1)-r1*cos(phi1)*sin(th1)=y2+z3+z4;
phi1=atan2(py-y6*ay,px-y6*ax);r1=sqrt((py-y6*ay)^2+(px-y6*ax)^2);
% sin(phi1-th1)=(y2+z3+z4)/r1;cos(phi1-th1)=±sqrt(1-((y2+z3+z4)/r1)^2);
theta1=atan2((y2+z3+z4)/r1,sqrt(1-((y2+z3+z4)/r1)^2));
th11=phi1-theta1;
th12=phi1+theta1;
for i=1:4
    Q(i,1)=th11*180/pi;Q(i+4,1)=th12*180/pi;
end

% ay*cos(th1)-ax*sin(th1) = cos(th5) ⑥ => th5
% ±sqrt(1-(ay*cos(th1)-ax*sin(th1))^2)=sin(th5); 
th51=atan2(sqrt(1-(ay*cos(th11)-ax*sin(th11))^2),ay*cos(th11)-ax*sin(th11));
th52=-atan2(sqrt(1-(ay*cos(th11)-ax*sin(th11))^2),ay*cos(th11)-ax*sin(th11));
th53=atan2(sqrt(1-(ay*cos(th12)-ax*sin(th12))^2),ay*cos(th12)-ax*sin(th12));
th54=-atan2(sqrt(1-(ay*cos(th12)-ax*sin(th12))^2),ay*cos(th12)-ax*sin(th12));
for i=1:4
    Q(i,5)=th53*180/pi;Q(i+4,5)=th54*180/pi;
end

% sin(th6)*(ny*cos(th1)-nx*sin(th1))+cos(th6)*(oy*cos(th1)-ox*sin(th1))=0 ⑦ => th6
% -tan(th6)=(oy*cos(th1)-ox*sin(th1))/(ny*cos(th1)-nx*sin(th1));
if (oy*cos(th1)-ox*sin(th1)>0) %y>0,①②
    th61=-atan2(oy*cos(th11)-ox*sin(th11),ny*cos(th11)-nx*sin(th11))+pi;
    th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12))+pi;
else %y<0,③④
    th61=-atan2(oy*cos(th11)-ox*sin(th11),ny*cos(th11)-nx*sin(th11))-pi;
    th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12))-pi;
end
for i=1:8
    Q(i,6)=th62*180/pi;
end

% az=sin(th2+th3+th4)*sin(th5) ⑩ => th2+th3+th4
% -oz*cos(th6)-nz*sin(th6) = cos(th2+th3+th4) ?　sin(th5)=0 出现奇异点；
% ±sqrt(1-(-oz*cos(th6)-nz*sin(th6))^2)=sin(th2+th3+th4);
% thsum1=atan2(sqrt(1-(-oz*cos(th61)-nz*sin(th61))^2),-oz*cos(th61)-nz*sin(th61));
% thsum2=-atan2(sqrt(1-(-oz*cos(th61)-nz*sin(th61))^2),-oz*cos(th61)-nz*sin(th61));
thsum3=atan2(sqrt(1-(-oz*cos(th62)-nz*sin(th62))^2),-oz*cos(th62)-nz*sin(th62));
thsum4=-atan2(sqrt(1-(-oz*cos(th62)-nz*sin(th62))^2),-oz*cos(th62)-nz*sin(th62));

% px*cos(th1)+py*sin(th1)-y6*(ax*cos(th1)+ay*sin(th1))=-y5*sin(th2+th3+th4)-y4*sin(th2+th3)-y3*sin(th2) ④
% pz-az*y6=z2-y4*cos(th2+th3)-y3*cos(th2)-y5*cos(th2+th3+th4) ?
% cos(th2+th3+th4)=-(oz*cos(th6)+nz*sin(th6));
% sin(th2+th3+th4)=-(sin(th6)*(nx*cos(th1)+ny*sin(th1))+cos(th6)*(ox*cos(th1)+oy*sin(th1)));

% y4*sin(th2+th3)+y3*sin(th2)=-(px*cos(th1)+py*sin(th1))+y6*(ax*cos(th1)+ay*sin(th1))+y5*(sin(th6)*(nx*cos(th1)+ny*sin(th1))+cos(th6)*(ox*cos(th1)+oy*sin(th1)));
% y4*cos(th2+th3)+y3*cos(th2)=z2-pz+az*y6+y5*(oz*cos(th6)+nz*sin(th6));
km1=-(px*cos(th11)+py*sin(th11))+y6*(ax*cos(th11)+ay*sin(th11))+y5*(sin(th61)*(nx*cos(th11)+ny*sin(th11))+cos(th61)*(ox*cos(th11)+oy*sin(th11)));
km2=z2-pz+az*y6+y5*(oz*cos(th61)+nz*sin(th61));
km3=-(px*cos(th12)+py*sin(th12))+y6*(ax*cos(th12)+ay*sin(th12))+y5*(sin(th62)*(nx*cos(th12)+ny*sin(th12))+cos(th62)*(ox*cos(th12)+oy*sin(th12)));
km4=z2-pz+az*y6+y5*(oz*cos(th62)+nz*sin(th62));

k31=(km1^2+km2^2-y4^2-y3^2)/(2*y4*y3);
k32=(km3^2+km4^2-y4^2-y3^2)/(2*y4*y3);
r1=sqrt((2*km1*y3)^2+(2*km2*y3)^2);phi1=atan2(2*km2*y3/r1,2*km1*y3/r1);
r2=sqrt((2*km3*y3)^2+(2*km4*y3)^2);phi2=atan2(2*km4*y3/r2,2*km3*y3/r2);
k21=(km1^2+km2^2+y3^2-y4^2)/r1; 
k22=(km3^2+km4^2+y3^2-y4^2)/r2;

if (k31<=1)&(k21<=1)
    th31=atan2(sqrt(1-k31^2),k31);
    th32=-th31;
    theta1=atan2(k21,sqrt(1-k21^2));
    th21=theta1-phi1;
    th22=-theta1-phi1+pi;
else
    th31=NaN;th32=NaN;th21=NaN;th22=NaN;
end
if (k32<=1)&(k22<=1)
    th33=atan2(sqrt(1-k32^2),k32);
    th34=-th33;
    theta2=atan2(k22,sqrt(1-k22^2));
    th23=theta2-phi2;
    th24=-theta2-phi2+pi;
else
    th33=NaN;th34=NaN;th23=NaN;th24=NaN;
end

for i=1:2
    Q(i,3)=th31*180/pi;Q(i+2,3)=th32*180/pi;
    Q(i+4,3)=th33*180/pi;Q(i+6,3)=th34*180/pi;
end
for i=1:2:3
    Q(i,2)=th21*180/pi;Q(i+1,2)=th22*180/pi;
    Q(i+4,2)=th23*180/pi;Q(i+5,2)=th24*180/pi;
end
Q(1,4)=(thsum3-th31-th21)*180/pi; 
Q(2,4)=(thsum3-th31-th22)*180/pi;
Q(3,4)=(thsum3-th32-th21)*180/pi;
Q(4,4)=(thsum3-th32-th22)*180/pi;
Q(5,4)=(thsum4-th33-th23)*180/pi;
Q(6,4)=(thsum4-th33-th24)*180/pi;
Q(7,4)=(thsum4-th34-th23)*180/pi;
Q(8,4)=(thsum4-th34-th24)*180/pi;
Q
%% function1
for i=1:8;
    Qt=verify(i);
    if (abs(Qt(1,4)-T(1,4))<1)&(abs(Qt(2,4)-T(2,4))<1)&(abs(Qt(3,4)-T(3,4))<1)
        for j=1:6
            S(j)=Q(i,j);
        end
        eval(['S',num2str(i),'=','S']);
        eval(['Q',num2str(i),'=','verify(i)']);
    end
end  

function [Ti]=verify(i)
    global Q;
    syms y1 y2 y3 y4 z1 z2 z3 z5;
    y1=120;y2=-480;y3=-400;y4=-100;z1=100;z2=-120;z3=100;z5=100;
    T1=trotz(Q(i,1)*pi/180)*transl(0,y1,z1)*trotx(-pi/2);
    T2=trotz(Q(i,2)*pi/180)*transl(0,y2,z2);
    T3=trotz(Q(i,3)*pi/180)*transl(0,y3,z3);
    T4=trotz(Q(i,4)*pi/180)*transl(0,y4,0);
    T5=troty(Q(i,5)*pi/180)*transl(0,0,z5);
    T6=trotz(Q(i,6)*pi/180);
    Ti=T1*T2*T3*T4*T5*T6;
end

function [Ti]=invtrot(T)  %inverse of rotation
    for i=1:2
        for j=i+1:3
            temp=T(i,j);
            T(i,j)=T(j,i);
            T(j,i)=temp;
        end
    end
    Ti=T;
end
function [Ti]=invtrans(T)  %inverse of translation
    for i=1:3
        T(i,4)=-T(i,4);
    end
    Ti=T;
end

% function [Ti]=transl(x,y,z)
% Ti=[1 0 0 x;0 1 0 y;0 0 1 z;0 0 0 1];
% end
% function [Ti]=trotx(theta)
% Ti=[1 0 0 0;0 cos(theta) -sin(theta) 0;0 sin(theta) cos(theta) 0;0 0 0 1];
% end
% function [Ti]=troty(theta)
% Ti=[cos(theta) 0 sin(theta) 0;0 1 0 0;-sin(theta) 0 cos(theta) 0;0 0 0 1];
% end
% function [Ti]=trotz(theta)
% Ti=[cos(theta) -sin(theta) 0 0;sin(theta) cos(theta) 0 0;0 0 1 0;0 0 0 1];
% end
%% 
