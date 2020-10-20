clc; %version1;
syms th1 th2 th3 th4 th5 th6 nx ox ax px ny oy ay py nz oz az pz;
Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];Q=[];
syms y1 y2 y3 y4 z1 z2 z3 z5;
y1=120;y2=-480;y3=-400;y4=-100;z1=100;z2=-120;z3=100;z5=100;
nx=0; ox=0; ax=-1; px=540;ny=1; oy=0; ay=0; py=100;nz=0; oz=-1; az=0; pz=615.69;

T1=trotz(th1)*transl(0,y1,z1)*trotx(-pi/2);
T2=trotz(th2)*transl(0,y2,z2);
T3=trotz(th3)*transl(0,y3,z3);
T4=trotz(th4)*transl(0,y4,0);
T5=troty(th5)*transl(0,0,z5);
T6=trotz(th6);
T=T1*T2*T3*T4*T5*T6;
% T26=simplify(T2*T3*T4*T5*T6);
% T26i=invtrot(trotx(-pi/2))*invtrans(transl(0,y1,z1))*invtrot(trotz(th1))*Tq;

%-----------------------------------------------------------------------------
phi=atan2(py-ay*z5,px-ax*z5);r=sqrt((py-ay*z5)^2+(px-ax*z5)^2);
k=y1+z2+z3;
theta=atan2(k,sqrt(r^2-k^2));
th11=phi+theta;
th12=phi-theta;
Q(1,1)=th11*180/pi;Q(5,1)=th12*180/pi;
Q(2,1)=Q(1,1);Q(3,1)=Q(1,1);Q(4,1)=Q(1,1);Q(6,1)=Q(5,1);Q(7,1)=Q(5,1);Q(8,1)=Q(5,1);

a=ay*cos(th11)-ax*sin(th11);b=sqrt(1-(ay*cos(th11)-ax*sin(th11))^2);
c=ay*cos(th12)-ax*sin(th12);d=sqrt(1-(ay*cos(th12)-ax*sin(th12))^2);
th52=-atan2(b,a);
th54=-atan2(d,c);
Q(1,5)=th52*180/pi;Q(5,5)=th54*180/pi;
Q(2,5)=Q(1,5);Q(3,5)=Q(1,5);Q(4,5)=Q(1,5);Q(6,5)=Q(5,5);Q(7,5)=Q(5,5);Q(8,5)=Q(5,5);

th61=-atan2(oy*cos(th11)-ox*sin(th11),ny*cos(th11)-nx*sin(th11));
th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12));
Q(1,6)=th61*180/pi;Q(5,6)=th62*180/pi;
Q(2,6)=Q(1,6);Q(3,6)=Q(1,6);Q(4,6)=Q(1,6);Q(6,6)=Q(5,6);Q(7,6)=Q(5,6);Q(8,6)=Q(5,6);

%-----------------------------------------------------------------------------
thsum1=atan2(-az,ax*cos(th11)+ay*sin(th11));
thsum2=atan2(-az,ax*cos(th12)+ay*sin(th12));

k1=z5*(ax*cos(th11)+ay*sin(th11))+y4*az/sin(th52)-(px*cos(th11)+py*sin(th11));
k2=z1-pz+z5*az-y4*(ax*cos(th11)+ay*sin(th11))/sin(th52);
k5=(k1^2+k2^2-y2^2-y3^2)/(2*y2*y3);
th31=atan2(sqrt(1-k5^2),k5);
th32=-atan2(sqrt(1-k5^2),k5);
Q(1,3)=th31*180/pi;Q(3,3)=th32*180/pi;
Q(2,3)=Q(1,3);Q(5,3)=Q(1,3);Q(6,3)=Q(1,3);Q(4,3)=Q(3,3);Q(7,3)=Q(3,3);Q(8,3)=Q(3,3);

phi=atan2(2*k2*y2,2*k1*y2);r=sqrt((2*k2*y2)^2+(2*k1*y2)^2);
k=(k1^2+k2^2+y2^2-y3^2)/r;
theta=atan2(k,sqrt(1-k^2));
th21=theta-phi;
th22=-theta-phi;
Q(1,2)=th21*180/pi;Q(2,2)=th22*180/pi;
Q(3,2)=Q(1,2);Q(5,2)=Q(1,2);Q(7,2)=Q(1,2);Q(4,2)=Q(2,2);Q(6,2)=Q(2,2);Q(8,2)=Q(2,2);

Q(1,4)=(thsum1-th31-th21+pi)*180/pi; 
Q(2,4)=(thsum1-th31-th22+pi)*180/pi;
Q(3,4)=(thsum1-th32-th21+pi)*180/pi;
Q(4,4)=(thsum1-th32-th22+pi)*180/pi;
Q(5,4)=(thsum2-th31-th21+pi)*180/pi;
Q(6,4)=(thsum2-th31-th22+pi)*180/pi;
Q(7,4)=(thsum2-th32-th21+pi)*180/pi;
Q(8,4)=(thsum2-th32-th22+pi)*180/pi;
Q
%----------------------------------------------------------------

% version2;
clc;
format long g;
syms th1 th2 th3 th4 th5 th6 nx ox ax px ny oy ay py nz oz az pz;
Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];global Q;
syms y1 y2 y3 y4 z1 z2 z3 z5;
y1=120;y2=-480;y3=-400;y4=-100;z1=100;z2=-120;z3=100;z5=100;
th1=pi/3;th2=pi/6;th3=pi/3;th4=pi/2;th5=-pi/2;th6=0;
% th1=pi/6;th2=pi/4;th3=pi/3;th4=-pi/2;th5=pi/2;th6=pi/4;
Q0=180/pi*[th1 th2 th3 th4 th5 th6]
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
%-----------------------------------------------------------------------------
phi=atan2(py-ay*z5,px-ax*z5);r1=sqrt((py-ay*z5)^2+(px-ax*z5)^2);
kth1=y1+z2+z3;
theta=atan2(kth1,sqrt(r1^2-kth1^2));%judge:(r^2-k^2)>0;
th11=phi+theta;
th12=phi-theta; %√
Q(1,1)=th11*180/pi;Q(5,1)=th12*180/pi;
Q(2,1)=Q(1,1);Q(3,1)=Q(1,1);Q(4,1)=Q(1,1);Q(6,1)=Q(5,1);Q(7,1)=Q(5,1);Q(8,1)=Q(5,1);

cos1=ay*cos(th11)-ax*sin(th11);sin1=sqrt(1-(ay*cos(th11)-ax*sin(th11))^2);
cos2=ay*cos(th12)-ax*sin(th12);sin2=sqrt(1-(ay*cos(th12)-ax*sin(th12))^2); %√
th51=atan2(sin1,cos1);
th52=-atan2(sin1,cos1);
th53=atan2(sin2,cos2); %√
th54=-atan2(sin2,cos2);
Q(1,5)=th51*180/pi;Q(5,5)=th53*180/pi;
Q(2,5)=Q(1,5);Q(3,5)=Q(1,5);Q(4,5)=Q(1,5);Q(6,5)=Q(5,5);Q(7,5)=Q(5,5);Q(8,5)=Q(5,5);

th61=-atan2(oy*cos(th11)-ox*sin(th11),ny*cos(th11)-nx*sin(th11));
th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12)); %√
Q(1,6)=(pi+th61)*180/pi;Q(5,6)=(pi+th62)*180/pi;
Q(2,6)=Q(1,6);Q(3,6)=Q(1,6);Q(4,6)=Q(1,6);Q(6,6)=Q(5,6);Q(7,6)=Q(5,6);Q(8,6)=Q(5,6);

%-----------------------------------------------------------------------------
thsum1=atan2(-az,ax*cos(th11)+ay*sin(th11));
thsum2=atan2(-az,ax*cos(th12)+ay*sin(th12));%√

% k1=z5*(ax*cos(th11)+ay*sin(th11))+y4*az/sin(th51)-(px*cos(th11)+py*sin(th11));
% k2=z5*az-y4*(ax*cos(th11)+ay*sin(th11))/sin(th51)+z1-pz;
% k3=z5*(ax*cos(th11)+ay*sin(th11))+y4*az/sin(th52)-(px*cos(th11)+py*sin(th11));
% k4=z5*az-y4*(ax*cos(th11)+ay*sin(th11))/sin(th52)+z1-pz;
k5=z5*(ax*cos(th12)+ay*sin(th12))+y4*az/sin(th53)-(px*cos(th12)+py*sin(th12)); %√
k6=z5*az-y4*(ax*cos(th12)+ay*sin(th12))/sin(th53)+z1-pz; %√
k7=z5*(ax*cos(th12)+ay*sin(th12))+y4*az/sin(th54)-(px*cos(th12)+py*sin(th12));
k8=z5*az-y4*(ax*cos(th12)+ay*sin(th12))/sin(th54)+z1-pz;

kth31=(k1^2+k2^2-y2^2-y3^2)/(2*y2*y3); 
kth32=(k3^2+k4^2-y2^2-y3^2)/(2*y2*y3);
kth33=(k5^2+k6^2-y2^2-y3^2)/(2*y2*y3); %√
kth34=(k7^2+k8^2-y2^2-y3^2)/(2*y2*y3);
r=sqrt((2*k6*y2)^2+(2*k5*y2)^2);
phi=atan2(2*k6*y2/r,2*k5*y2/r);
kth21=(k1^2+k2^2+y2^2-y3^2)/r; 
kth22=(k3^2+k4^2+y2^2-y3^2)/r;
kth23=(k5^2+k6^2+y2^2-y3^2)/r; %√
kth24=(k7^2+k8^2+y2^2-y3^2)/r;
if (kth33<1)&(kth23<1)
th31=atan2(sqrt(1-kth33^2),kth33);
th32=-atan2(sqrt(1-kth33^2),kth33);
theta=atan2(kth23,sqrt(1-kth23^2));
th21=theta-phi;
th22=-theta-phi;
end
Q(1,3)=th31*180/pi;Q(3,3)=th32*180/pi;
Q(2,3)=Q(1,3);Q(5,3)=Q(1,3);Q(6,3)=Q(1,3);Q(4,3)=Q(3,3);Q(7,3)=Q(3,3);Q(8,3)=Q(3,3);

Q(1,2)=th21*180/pi;Q(2,2)=th22*180/pi;
Q(3,2)=Q(1,2);Q(5,2)=Q(1,2);Q(7,2)=Q(1,2);Q(4,2)=Q(2,2);Q(6,2)=Q(2,2);Q(8,2)=Q(2,2);
Q(1,4)=(thsum1-th31-th21)*180/pi; 
Q(2,4)=(thsum1-th31-th22)*180/pi;
Q(3,4)=(thsum1-th32-th21)*180/pi;
Q(4,4)=(thsum1-th32-th22)*180/pi;
Q(5,4)=(thsum2-th31-th21)*180/pi;
Q(6,4)=(thsum2-th31-th22)*180/pi;
Q(7,4)=(thsum2-th32-th21)*180/pi;
Q(8,4)=(thsum2-th32-th22)*180/pi;
Q

for i=1:8;
    Qt=verify(i);
    if (abs(Qt(1,4)-T(1,4))<1)||(abs(Qt(2,4)-T(2,4))<1)||(abs(Qt(3,4)-T(3,4))<1)
        eval(['Q',num2str(i),'=','verify(i)']);
    end
end    

% -------------------------------------------------------------------------
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


%--------------------------------------
%version3
clc;clear all;
format long g;
syms th1 th2 th3 th4 th5 th6 nx ox ax px ny oy ay py nz oz az pz;
Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];global Q;
syms y1 y2 y3 y4 z1 z2 z3 z5;
y1=120;y2=-480;y3=-400;y4=-100;z1=100;z2=-120;z3=100;z5=100;
% th1=0;th2=0;th3=0;th4=pi/2;th5=0;th6=0;
% th1=pi/3;th2=pi/6;th3=pi/3;th4=pi/2;th5=-pi/2;th6=pi/4;
th1=pi/6;th2=pi/4;th3=-pi/3;th4=-pi/2;th5=-pi/2;th6=pi/4;
Q0=180/pi*[th1 th2 th3 th4 th5 th6]
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
%-----------------------------------------------------------------------------
phi=atan2(py-ay*z5,px-ax*z5);rth1=sqrt((py-ay*z5)^2+(px-ax*z5)^2);
kth1=y1+z2+z3;
theta=atan2(kth1,sqrt(rth1^2-kth1^2));%judge:(r^2-k^2)>0;
th11=phi+theta;
th12=phi-theta; %√
for i=1:8
    Q(i,1)=th11*180/pi;Q(i+8,1)=th12*180/pi;
end
c11=ay*cos(th11)-ax*sin(th11);s11=sqrt(1-(ay*cos(th11)-ax*sin(th11))^2);
c12=ay*cos(th12)-ax*sin(th12);s12=sqrt(1-(ay*cos(th12)-ax*sin(th12))^2); %√
th51=atan2(s11,c11);
th52=-th51;
th53=atan2(s12,c12); %√
th54=-th53; %√
for i=1:4
    Q(i,5)=th51*180/pi;Q(i+4,5)=th52*180/pi;
    Q(i+8,5)=th53*180/pi;Q(i+12,5)=th54*180/pi;
end

th61=-atan2(oy*cos(th11)-ox*sin(th11),ny*cos(th11)-nx*sin(th11))+pi;
if (oy*cos(th11)-ox*sin(th11)>0)&(ny*cos(th11)-nx*sin(th11)<0)
    th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12))+pi;
% elseif (oy*cos(th11)-ox*sin(th11)<0)&(ny*cos(th11)-nx*sin(th11)>0)
%     th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12))-pi;
else
    th62=-atan2(oy*cos(th12)-ox*sin(th12),ny*cos(th12)-nx*sin(th12));
end
for i=1:8
    Q(i,6)=th61*180/pi;Q(i+8,6)=th62*180/pi;
end
%-----------------------------------------------------------------------------
thsum1=atan2(-az,ax*cos(th11)+ay*sin(th11));
if (-az<0) %&(ax*cos(th11)+ay*sin(th11)>0)
    thsum2=atan2(-az,ax*cos(th12)+ay*sin(th12))+pi;
% elseif (-az<0)&(ax*cos(th11)+ay*sin(th11)<0)
%     thsum2=atan2(-az,ax*cos(th12)+ay*sin(th12))+pi;
else
    thsum2=atan2(-az,ax*cos(th12)+ay*sin(th12));
end
k1=z5*(ax*cos(th11)+ay*sin(th11))+y4*az/sin(th51)-(px*cos(th11)+py*sin(th11));
k2=z5*az-y4*(ax*cos(th11)+ay*sin(th11))/sin(th51)+z1-pz;
k3=z5*(ax*cos(th11)+ay*sin(th11))+y4*az/sin(th52)-(px*cos(th11)+py*sin(th11));
k4=z5*az-y4*(ax*cos(th11)+ay*sin(th11))/sin(th52)+z1-pz;
k5=z5*(ax*cos(th12)+ay*sin(th12))+y4*az/sin(th53)-(px*cos(th12)+py*sin(th12)); %√
k6=z5*az-y4*(ax*cos(th12)+ay*sin(th12))/sin(th53)+z1-pz; %√
k7=z5*(ax*cos(th12)+ay*sin(th12))+y4*az/sin(th54)-(px*cos(th12)+py*sin(th12));
k8=z5*az-y4*(ax*cos(th12)+ay*sin(th12))/sin(th54)+z1-pz;

kth31=(k1^2+k2^2-y2^2-y3^2)/(2*y2*y3); 
kth32=(k3^2+k4^2-y2^2-y3^2)/(2*y2*y3);
kth33=(k5^2+k6^2-y2^2-y3^2)/(2*y2*y3);
kth34=(k7^2+k8^2-y2^2-y3^2)/(2*y2*y3);
r1=sqrt((2*k1*y2)^2+(2*k2*y2)^2);phi1=atan2(2*k2*y2/r1,2*k1*y2/r1);
r2=sqrt((2*k3*y2)^2+(2*k4*y2)^2);phi2=atan2(2*k4*y2/r2,2*k3*y2/r2);
r3=sqrt((2*k5*y2)^2+(2*k6*y2)^2);phi3=atan2(2*k6*y2/r3,2*k5*y2/r3);
r4=sqrt((2*k7*y2)^2+(2*k8*y2)^2);phi4=atan2(2*k8*y2/r4,2*k7*y2/r4);
kth21=(k1^2+k2^2+y2^2-y3^2)/r1; 
kth22=(k3^2+k4^2+y2^2-y3^2)/r2;
kth23=(k5^2+k6^2+y2^2-y3^2)/r3; 
kth24=(k7^2+k8^2+y2^2-y3^2)/r4;

if (kth31<=1)&(kth21<=1)
    th31=atan2(sqrt(1-kth31^2),kth31);
    th32=-th31;
    theta=atan2(kth21,sqrt(1-kth21^2));
    th21=theta-phi1;
    th22=-theta-phi1;
else
    th31=NaN;th32=NaN;th21=NaN;th22=NaN;
end
if (kth32<=1)&(kth22<=1)
    th33=atan2(sqrt(1-kth32^2),kth32);
    th34=-th31;
    theta=atan2(kth22,sqrt(1-kth22^2));
    th23=theta-phi2;
    th24=-theta-phi2;
else
    th33=NaN;th34=NaN;th23=NaN;th24=NaN;
end
if (kth33<=1)&(kth23<=1)
    th35=atan2(sqrt(1-kth33^2),kth33);
    th36=-th31;
    theta=atan2(kth23,sqrt(1-kth23^2));
    th25=theta-phi3;
    th26=-theta-phi3;
else
    th35=NaN;th36=NaN;th25=NaN;th26=NaN;
end
if (kth34<=1)&(kth24<=1)
    th37=atan2(sqrt(1-kth34^2),kth34);
    th38=-th37;
    theta=atan2(kth24,sqrt(1-kth24^2));
    th27=theta-phi4;
    th28=-theta-phi4;
else
    th37=NaN;th38=NaN;th27=NaN;th28=NaN;
end

Q(1,3)=th31*180/pi;Q(2,3)=Q(1,3);
Q(3,3)=th32*180/pi;Q(4,3)=Q(3,3);
Q(5,3)=th33*180/pi;Q(6,3)=Q(5,3);
Q(7,3)=th34*180/pi;Q(8,3)=Q(7,3);
Q(9,3)=th35*180/pi;Q(10,3)=Q(9,3);
Q(11,3)=th36*180/pi;Q(12,3)=Q(11,3);
Q(13,3)=th37*180/pi;Q(14,3)=Q(13,3);
Q(15,3)=th38*180/pi;Q(16,3)=Q(15,3);

Q(1,2)=th21*180/pi;Q(3,2)=Q(1,2);
Q(2,2)=th22*180/pi;Q(4,2)=Q(2,2);
Q(5,2)=th23*180/pi;Q(7,2)=Q(5,2);
Q(6,2)=th24*180/pi;Q(8,2)=Q(6,2);
Q(9,2)=th25*180/pi;Q(11,2)=Q(9,2);
Q(10,2)=th26*180/pi;Q(12,2)=Q(10,2);
Q(13,2)=th27*180/pi;Q(15,2)=Q(13,2);
Q(14,2)=th28*180/pi;Q(16,2)=Q(14,2);

Q(1,4)=(thsum1-th31-th21)*180/pi; 
Q(2,4)=(thsum1-th31-th22)*180/pi;
Q(3,4)=(thsum1-th32-th21)*180/pi;
Q(4,4)=(thsum1-th32-th22)*180/pi;
Q(5,4)=(thsum1-th33-th23)*180/pi;
Q(6,4)=(thsum1-th33-th24)*180/pi;
Q(7,4)=(thsum1-th34-th23)*180/pi;
Q(8,4)=(thsum1-th34-th24)*180/pi;

Q(9,4)=(thsum2-th35-th25)*180/pi; 
Q(10,4)=(thsum2-th35-th26)*180/pi;
Q(11,4)=(thsum2-th36-th25)*180/pi;
Q(12,4)=(thsum2-th36-th26)*180/pi;
Q(13,4)=(thsum2-th37-th27)*180/pi;
Q(14,4)=(thsum2-th37-th28)*180/pi;
Q(15,4)=(thsum2-th38-th27)*180/pi;
Q(16,4)=(thsum2-th38-th28)*180/pi;
Q
for i=1:16;
    Qt=verify(i);
    if (abs(Qt(1,4)-T(1,4))<1)&(abs(Qt(2,4)-T(2,4))<1)&(abs(Qt(3,4)-T(3,4))<1)
        for j=1:6
            S(j)=Q(i,j);
        end
        eval(['S',num2str(i),'=','S']);
        eval(['Q',num2str(i),'=','verify(i)']);
    end
end    

% -------------------------------------------------------------------------
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

%--------------------------------------------------------
------
