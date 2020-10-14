%% setting
clc;clear;close all;
format short g;format compact;

%% ��������
%{
syms a8 a9 d7 th8 th9 nx ny nz ox oy oz ax ay az px py pz
syms th1 th2 th3 th4 th5 th6 d7 th8 th9;
syms nx ox ax px ny oy ay py nz oz az pz;
theta=[th1 th2 th3 th4 th5 th6 d7 th8 th9]; %�ؽ��������
% a8=50;a9=50;
% theta=[0 0 0 0 0 0 100 0 pi/4];

%}
% syms th1 th2 th3 th4 th5 th6 d7 th8 th9;
% theta=[th1 th2 th3 th4 th5 th6 d7 th8 th9];
% forward_kine456(theta)

% syms th1 th2 th3 th4 th5 th6 th8 th9 d1 d4 d5 d6 a1 a2 a3 a8 a9 d7;
% syms nx ox ax px ny oy ay py nz oz az pz;
% 
% nx=-cos(th9)*(cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))+cos(th4)*sin(th5)*sin(th8))-sin(th9)*(cos(th6)*sin(th4)+cos(th4)*cos(th5)*sin(th6));
% ox=sin(th9)*(cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))+cos(th4)*sin(th5)*sin(th8))-cos(th9)*(cos(th6)*sin(th4)+cos(th4)*cos(th5)*sin(th6));
% ax=sin(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))-cos(th4)*cos(th8)*sin(th5);
% px=-a8*cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))-a9*sin(th9)*(cos(th6)*sin(th4)+cos(th4)*cos(th5)*sin(th6))-d7*cos(th4)*sin(th5)-a9*cos(th9)*(cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))+cos(th4)*sin(th5)*sin(th8))-a8*cos(th4)*sin(th5)*sin(th8);
% 
% ny=cos(th9)*(cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))-sin(th4)*sin(th5)*sin(th8))+sin(th9)*(cos(th4)*cos(th6)-cos(th5)*sin(th4)*sin(th6));
% oy=cos(th9)*(cos(th4)*cos(th6)-cos(th5)*sin(th4)*sin(th6))-sin(th9)*(cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))-sin(th4)*sin(th5)*sin(th8));
% ay=-sin(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))-cos(th8)*sin(th4)*sin(th5);
% py=a8*cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))+a9*sin(th9)*(cos(th4)*cos(th6)-cos(th5)*sin(th4)*sin(th6))-d7*sin(th4)*sin(th5)+a9*cos(th9)*(cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))-sin(th4)*sin(th5)*sin(th8))-a8*sin(th4)*sin(th5)*sin(th8);
% 
% nz=cos(th9)*(cos(th5)*sin(th8)+cos(th6)*cos(th8)*sin(th5))-sin(th5)*sin(th6)*sin(th9);
% oz=-sin(th9)*(cos(th5)*sin(th8)+cos(th6)*cos(th8)*sin(th5))-cos(th9)*sin(th5)*sin(th6);
% az=cos(th5)*cos(th8)-cos(th6)*sin(th5)*sin(th8);
% pz=d7*cos(th5)+a9*cos(th9)*(cos(th5)*sin(th8)+cos(th6)*cos(th8)*sin(th5))+a8*cos(th5)*sin(th8)+a8*cos(th6)*cos(th8)*sin(th5)-a9*sin(th5)*sin(th6)*sin(th9);

% -sin(th4)*sin(th6) + cos(th4)*cos(th5)*cos(th6) == -ax*sin(th8) + cos(th8)*(nx*cos(th9)-ox*sin(th9))  �� ��
%  cos(th4)*sin(th6) + sin(th4)*cos(th5)*cos(th6) == -ay*sin(th8) + cos(th8)*(ny*cos(th9)-oy*sin(th9))  �� ��
%                               sin(th5)*cos(th6) == -az*sin(th8) + cos(th8)*(nz*cos(th9)-oz*sin(th9))  ��
% 
% -cos(th4)*sin(th5) == ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))  ��
% -sin(th4)*sin(th5) == ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))  ��
%           cos(th5) == az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))  ��
% 
%  sin(th4)*cos(th6) + cos(th4)*cos(th5)*sin(th6) == -ox*cos(th9)-nx*sin(th9)  ��
% -cos(th4)*cos(th6) + sin(th4)*cos(th5)*sin(th6) == -oy*cos(th9)-ny*sin(th9)  ��
%                               sin(th5)*sin(th6) == -oz*cos(th9)-nz*sin(th9)  ��
% 
% -d7*cos(th4)*sin(th5) == px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9))  ��
% -d7*sin(th4)*sin(th5) == py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9))  �٢�
%           d7*cos(th5) == pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9))  �٢�
% -------------------------------------------------------------------------
% 
% -d7*cos(th4)*sin(th5) == px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9))  % 6��ʽ����
%    -cos(th4)*sin(th5) == ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))
% -d7*sin(th4)*sin(th5) == py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9))  
%    -sin(th4)*sin(th5) == ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))
%           d7*cos(th5) == pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9))  
%              cos(th5) == az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))

% cos(th5) == az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))  
%  sin(th4)*cos(th6) + cos(th4)*sin(th6) == -ox*cos(th9)-nx*sin(th9) % cos(th5) == 1
% -cos(th4)*cos(th6) + sin(th4)*sin(th6) == -oy*cos(th9)-ny*sin(th9)

% -cos(th4)*sin(th5) == ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))  
% -sin(th4)*sin(th5) == ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9)) 

% sin(th5)*cos(th6) == -az*sin(th8) + cos(th8)*(nz*cos(th9)-oz*sin(th9))  
% sin(th5)*sin(th6) == -oz*cos(th9)-nz*sin(th9)  



%% ��th9 �м仯��
syms nx ny nz ox oy oz ax ay az px py pz
syms th1 th2 th3 th4 th5 th6 d7 th8 th9  a8 a9 d7;
% һ��һ����������ǿ
% d7*(ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))) == px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9))  ��
% d7*(ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))) == py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9))  ��
% d7*(az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))) == pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9))  ��
% d7 == (Pz-a8*(nz*cos(th9)-oz*sin(th9))) / (az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9)))  �ܴ��� �� ��
Px=px-a9*nx;
Py=py-a9*ny;
Pz=pz-a9*nz;
% (Pz-a8*(nz*cos(th9)-oz*sin(th9)))*(ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)))==(Px-a8*(nx*cos(th9)-ox*sin(th9)))*(az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9)))
% (Pz-a8*(nz*cos(th9)-oz*sin(th9)))*(ay*cos(th8)+sin(th8)*(ny*cos(th9)-oy*sin(th9)))==(Py-a8*(ny*cos(th9)-oy*sin(th9)))*(az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9)))
% Rx=nx*cos(th9)-ox*sin(th9); %syms Rx Ry Rz;
% Ry=ny*cos(th9)-oy*sin(th9);
% Rz=nz*cos(th9)-oz*sin(th9);
syms Rx Ry Rz Px Py Pz a8 cos sin; %ʹ��matlab����Ĺؼ������� Rx Ry Rz
 %С�ķ��ű����ظ�����.����ʹ�� sym/subsindex 

% (Pz-a8*Rz)*(ax*cos(th8) + sin(th8)*Rx) == (Px-a8*Rx)*(az*cos(th8) + sin(th8)*Rz)
% (Pz-a8*Rz)*(ay*cos(th8) + sin(th8)*Ry) == (Py-a8*Ry)*(az*cos(th8) + sin(th8)*Rz)
% �ϲ�ͬ���� cos(th8) sin(th8) ����
% ((Pz-a8*Rz)*ax-(Px-a8*Rx)*az)*cos(th8) == ((Px-a8*Rx)*Rz-(Pz-a8*Rz)*Rx)*sin(th8)
% ((Pz-a8*Rz)*ay-(Py-a8*Ry)*az)*cos(th8) == ((Py-a8*Ry)*Rz-(Pz-a8*Rz)*Ry)*sin(th8)
% �������
T8=((Pz-a8*Rz)*ax-(Px-a8*Rx)*az)*((Py-a8*Ry)*Rz-(Pz-a8*Rz)*Ry) - ((Pz-a8*Rz)*ay-(Py-a8*Ry)*az)*((Px-a8*Rx)*Rz-(Pz-a8*Rz)*Rx);
simplify(T8); %(Pz - Rz*a8)*(Px*Ry*az - Px*Rz*ay - Py*Rx*az + Py*Rz*ax + Pz*Rx*ay - Pz*Ry*ax)
% �ؼ����裬��Ҫ�˿����˿��͵õ�4�η���
% 0==Pz - a8*(nz*cos(th9)-oz*sin(th9))
% 0==(Pz*ay-Py*az)*Rx + (Px*az-Pz*ax)*Ry + (Py*ax-Px*ay)*Rz
Rx=nx*cos-ox*sin; %cos(th9) sin(th9)
Ry=ny*cos-oy*sin;
Rz=nz*cos-oz*sin;
T9=Px*Ry*az - Px*Rz*ay - Py*Rx*az + Py*Rz*ax + Pz*Rx*ay - Pz*Ry*ax;

collect(T9)% (Px*ay*oz-Px*az*oy-Py*ax*oz+Py*az*ox+Pz*ax*oy-Pz*ay*ox)*sin+(Px*az*ny-Px*ay*nz+Py*ax*nz-Py*az*nx-Pz*ax*ny+Pz*ay*nx)*cos
%(Px*(ay*oz-az*oy)+Py*(az*ox-ax*oz)+Pz*(ax*oy-ay*ox))*sin+(Px*(az*ny-ay*nz)+Py*(ax*nz-az*nx)+Pz*(ay*nx-ax*ny))*cos
Pm=Px*(az*ny-ay*nz)+Py*(ax*nz-az*nx)+Pz*(ay*nx-ax*ny);%�����Գƣ�nice
Pn=Px*(az*oy-ay*oz)+Py*(ax*oz-az*ox)+Pz*(ay*ox-ax*oy);
% 0==Pm*cos(th9) - Pn*sin(th9)
%���ˣ��ɵ�th9�ı��ʽ���������ţ�

%% th9
% d7*(ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))) == px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9))  ��
% d7*(ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))) == py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9))  ��
% d7*(az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))) == pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9))  ��

d11=ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9));% simplify(d11) %-cos(th4)*sin(th5)
d12=px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9));           % simplify(d12) %-d7*cos(th4)*sin(th5)
d21=ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9));% simplify(d21) %-sin(th4)*sin(th5)
d22=py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9));           % simplify(d22) %-d7*sin(th4)*sin(th5)
d31=az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9));% simplify(d31) %cos(th5)
d32=pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9));           % simplify(d32) %d7*cos(th5)
% up=px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9)); %-d7*cos(th4)*sin(th5)
% down=ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)); %-cos(th4)*sin(th5)%��ĸ
% up=py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9)); %-d7*sin(th4)*sin(th5)
% down=ay*cos(th8)+sin(th8)*(ny*cos(th9)-oy*sin(th9)); %-sin(th4)*sin(th5)%��ĸ
% up=pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9));  %d7*cos(th5)
% down=az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9));  %cos(th5)%��ĸ
m46 = -ox*cos(th9)-nx*sin(th9);  simplify(m46);
n46 = oy*cos(th9)+ny*sin(th9);  simplify(n46);
Px=px-a9*nx;
Py=py-a9*ny;
Pz=pz-a9*nz; 
Rx= nx*cos(th9)-ox*sin(th9);
Ry= ny*cos(th9)-oy*sin(th9);
Rz= nz*cos(th9)-oz*sin(th9);
% syms Rx Ry Rz Px Py Pz;
% % MATLAB�е�strfind��strrep���������Ƿֱ��������ַ����в����Ӵ����滻�Ӵ���
% ��1��simplify �����Ա��ʽ���л���
% ��2��radsimp�����Ժ���ʽ�ı��ʽ���л���
% ��3��combine ���������ʽ������͡��˻������������ʽ���ֵ�����кϲ���
% ��4��collet�ϲ�ͬ����
% ��5��factor����ʵ����ʽ�ֽ�
% ��6)convert������ɱ��ʽ��ʽ��ת��

% T=((Pz-a8*Rz)*ax-(Px-a8*Rx)*az)*((Py-a8*Ry)*Rz-(Pz-a8*Rz)*Ry) - ((Px-a8*Rx)*Rz-(Pz-a8*Rz)*Rx)*((Pz-a8*Rz)*ay-(Py-a8*Ry)*az); %ȫ����syms�ɻ���
% simplify(T) %(Pz - Rz*a8)*(Px*Ry*az - Px*Rz*ay - Py*Rx*az + Py*Rz*ax + Pz*Rx*ay - Pz*Ry*ax)
%(Pz - Rz*a8)*(Pz*Rx*ay - Py*Rx*az + Px*Ry*az - Pz*Ry*ax  + Py*Rz*ax - Px*Rz*ay)
% 0==Rx*(Pz*ay-Py*az) + Ry*(Px*az-Pz*ax) + Rz*(Py*ax-Px*ay);
% 0==Pz - Rz*a8
% simplify(Pz) % Pz = d7*cos(th5)+a8*(nz*cos(th9)-oz*sin(th9))
%          0== d7*cos(th5) == pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9))

% th9=acos(Pz/(a8*sqrt(nz^2+oz^2)))-atan2(oz,nz);
% th9=-acos(Pz/(a8*sqrt(nz^2+oz^2)))-atan2(oz,nz);
% Ax=Pz*ay-Py*az;
% Ay=Px*az-Pz*ax;
% Az=Py*ax-Px*ay;
% 0==(Ax*nx+Ay*ny+Az*nz)*cos(th9)-(Ax*ox+Ay*oy+Az*oz)*sin(th9);
Pm=(Pz*ay-Py*az)*nx+(Px*az-Pz*ax)*ny+(Py*ax-Px*ay)*nz;
Pn=(Pz*ay-Py*az)*ox+(Px*az-Pz*ax)*oy+(Py*ax-Px*ay)*oz;
% simplify(Pm) %sin(th9)*(a8 + d7*sin(th8))
% simplify(Pn) %cos(th9)*(a8 + d7*sin(th8))
th9=atan(Pm/Pn);
th9=atan(Pm/Pn)+pi;


Pn=(Px*az-Pz*ax)*ny+(Pz*ay-Py*az)*nx+(Py*ax-Px*ay)*nz;
Po=(Px*az-Pz*ax)*oy+(Pz*ay-Py*az)*ox+(Py*ax-Px*ay)*oz;
% a=a8*nz*Pn; % a8*sin(th9)*(a8 + d7*sin(th8))*(cos(th5)*cos(th9)*sin(th8) - sin(th5)*sin(th6)*sin(th9) + cos(th6)*cos(th8)*cos(th9)*sin(th5))
% b=a8*oz*Po; %-a8*cos(th9)*(a8 + d7*sin(th8))*(cos(th9)*sin(th5)*sin(th6) + cos(th5)*sin(th8)*sin(th9) + cos(th6)*cos(th8)*sin(th5)*sin(th9))
% c=-a8*(oz*Pn+nz*Po); %a8*(a8 + d7*sin(th8))*(cos(th5)*sin(th8) - 2*cos(th5)*cos(th9)^2*sin(th8) + cos(th6)*cos(th8)*sin(th5) - 2*cos(th6)*cos(th8)*cos(th9)^2*sin(th5) + 2*cos(th9)*sin(th5)*sin(th6)*sin(th9))
% d=-Pz*Pn; %-sin(th9)*(a8 + d7*sin(th8))*(d7*cos(th5) + a8*cos(th5)*sin(th8) + a8*cos(th6)*cos(th8)*sin(th5))
% e=Pz*Po; %  cos(th9)*(a8 + d7*sin(th8))*(d7*cos(th5) + a8*cos(th5)*sin(th8) + a8*cos(th6)*cos(th8)*sin(th5))
% s=d*cos(th9)+e*sin(th9);%0
% delta=simplify(c*c-4*a*b) %a8^2*(a8 + d7*sin(th8))^2*(cos(th5)*sin(th8) + cos(th6)*cos(th8)*sin(th5))^2
% T=simplify(t)
% U=simplify(d)
%% th8
% m81=ax*(Pz-a8*Rz)-az*(Px-a8*Rx);%-d7*sin(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6))
% m82=Rz*(Px-a8*Rx)-Rx*(Pz-a8*Rz);%-d7*cos(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6))
% m83=ay*(Pz-a8*Rz)-az*(Py-a8*Ry);%-d7*sin(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6))
% m84=Rz*(Py-a8*Ry)-Ry*(Pz-a8*Rz);%-d7*cos(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6))
% T=simplify(m84)
% m8=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx)); 
% n8=(Rz*(Px-a8*Rx)-Rx*(Pz-a8*Rz)); 
% X1=(-3*B+sqrt(9*B^2-24*A*C))/(12*A)
% m4=ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9));
% n4=ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9));
% 
% m6=-oz*cos(th9)-nz*sin(th9);
% n6=-az*sin(th8) + cos(th8)*(nz*cos(th9)-oz*sin(th9));





