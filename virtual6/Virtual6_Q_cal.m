%% setting
clc;clear;close all;
format short g;format compact;

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
Quartic equation, z(-y)z version;
����һ������汾���Ƶ����̣����������м��Ƶ����̣���ȻҲ���˺ܶ���·��
�������м�û�м�ʱ������������²�����һԪ�Ĵκ��������´�������Ҫ���жϣ�
���m�ű����ÿ��ˣ������ǽ����z(-y)z�汾�ˣ����Ѽ򻯵�sim�溯������������󣬺ܶ��������ǲ��Ͻ��ģ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% �﷨˵��
% simplify(T) %�����������ʽ
% pretty(T) %��ʾ������ϰ����д��ʽ
% collect(T,x) %�����ϲ�ͬ����
% factor(T) %��ʽ�ֽ�
%}


%% initial
% syms a8 a9 d7 th8 th9 nx ny nz ox oy oz ax ay az px py pz %��Ҫʱע�͵����ű����Ի�ȡС��
% syms th1 th2 th3 th4 th5 th6 d7 th8 th9;
% theta=[th1 th2 th3 th4 th5 th6 d7 th8 th9]; %�ؽ��������
a8=50;a9=50;
% theta=[0 0 0 0 0 0 100 0 pi/4];
% theta=[0 0 0 pi/2*(2*rand(1)-1) pi/2*(2*rand(1)-1) pi*(2*rand(1)-1) 100 pi*(2*rand(1)-1) pi*(2*rand(1)-1)];
% theta=[0  0  0  149.09       -146.9       177.49       5729.6      -145.28      -67.267]/180*pi; % x2jie
% theta=[0  0  0  65.322       51.364      -23.943       5729.6      -54.784      -160.06]/180*pi; % x2jie
% theta=[0  0  0  65.118       101.43       110.65       5729.6      -84.581       142.52]/180*pi;  % x4jie
theta=[0  0  0  -30.937      -89.429       -24.31       5729.6      -87.366       71.545]/180*pi;  % x4jie
theta=[0 0 0 pi/2*(2*rand(1)-1) pi/2*(2*rand(1)-1) pi*(2*rand(1)-1) 10 pi*(2*rand(1)-1) pi*(2*rand(1)-1)];
% theta=[0  0  0  -30.937      -89.429       -24.31       5729.6      -87.366       71.545]/180*pi;  % x4jie
% theta_deg=[0 0 0 10 20 30 40*180/pi 50 60]


theta_deg=theta*180/pi
T=forward_kine(theta);
% T=rpy2tr(1,1,0);T(1,4)=-100;T(2,4)=-100;T(3,4)=500;
% theta=ikine_Virtual(T);
% for i=1:8
%     eval(['Q',num2str(i),'=','vpa(forward_kine49(theta(i,1:9))-T,6)']);
%     T8=forward_kine49(theta(i,1:9))-T;
%     eval(['Q',num2str(i),'=','vpa(sum(sum(T8)),6)']);
% end


%% �����Ƶ� z(-y)z
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

nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
%% th9
% ��ȥd7
% ���ܢݢ޷ֱ����� �٢� �٢ڵó� 
% d7*(ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))) == px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9))  ��
% d7*(ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))) == py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9))  ��
% d7*(az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))) == pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9))  ��
% d7*M1=N1 
% d7*M2=N2
% d7*M3=N3
% ��е�ṹ�趨 d7>0; ��ֻ�趨 d7~=0;d7<0�����壬���ڹ����ռ��ڣ�d7=0�������⡣
% d7=N3/M3;  (N3/M3)*M1==N1;  N3*M1==N1*M3;  (N3/M3)*M2==N2;  N3*M2==N2*M3;
% ���� N3 M3Ϊ���������� d7���
% d7 == (Pz-a8*(nz*cos(th9)-oz*sin(th9))) / (az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9)))  �ܴ��� �� ��
Px=px-a9*nx;
Py=py-a9*ny;
Pz=pz-a9*nz;
% (Pz-a8*(nz*cos(th9)-oz*sin(th9)))*(ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)))==(Px-a8*(nx*cos(th9)-ox*sin(th9)))*(az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9)))
% (Pz-a8*(nz*cos(th9)-oz*sin(th9)))*(ay*cos(th8)+sin(th8)*(ny*cos(th9)-oy*sin(th9)))==(Py-a8*(ny*cos(th9)-oy*sin(th9)))*(az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9)))

% Rx=nx*cos(th9)-ox*sin(th9);
% Ry=ny*cos(th9)-oy*sin(th9);
% Rz=nz*cos(th9)-oz*sin(th9);
% (Pz-a8*Rz)*(ax*cos(th8) + sin(th8)*Rx) == (Px-a8*Rx)*(az*cos(th8) + sin(th8)*Rz)
% (Pz-a8*Rz)*(ay*cos(th8) + sin(th8)*Ry) == (Py-a8*Ry)*(az*cos(th8) + sin(th8)*Rz)
% ((Pz-a8*Rz)*ax-(Px-a8*Rx)*az)*cos(th8) == ((Px-a8*Rx)*Rz-(Pz-a8*Rz)*Rx)*sin(th8)
% ((Pz-a8*Rz)*ay-(Py-a8*Ry)*az)*cos(th8) == ((Py-a8*Ry)*Rz-(Pz-a8*Rz)*Ry)*sin(th8)
% ������ˣ���ȥth8
%{
% ��ĸ���Ż��� % ��ȡ sin(th8) cos(th8)
% syms Px Py Pz Rx Ry Rz cos sin
% PRx=(Px-a8*Rx);  
% PRy=(Py-a8*Ry);  
% PRz=(Pz-a8*Rz);
% % (PRz*ax-PRx*az)*cos(th8)==(PRx*Rz-PRz*Rx)*sin(th8) 
% % (PRz*ay-PRy*az)*cos(th8)==(PRy*Rz-PRz*Ry)*sin(th8) 
% (PRz*ax-PRx*az)*(PRy*Rz-PRz*Ry)==(PRz*ay-PRy*az)*(PRx*Rz-PRz*Rx)
% ����ʱ���Ŵ������һ��(ax*(Pz-a8*Rz)-az*(Px-a8*Rx))*(Rz*(Py-a8*Ry)-Ry*(Pz-a8*Rz))==(ay*(Pz-a8*Rz)-az*(Py-a8*Ry))*(Rz*(Px-a8*Rx)-Rx*(Pz-a8*Rz))
% % Pzy=(Pz*ay-Py*az);
% % Pxz=(Px*az-Pz*ax);
% % Pyx=(Py*ax-Px*ay);
% % Pn=Pxz*ny+Pzy*nx+Pyx*nz;
% % Po=Pxz*oy+Pzy*ox+Pyx*oz;
%}
% ����Rx Ry Rz������sin(th9) cos(th9)
% T=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx))*(Rz*(Py-a8*Ry)-Ry*(Pz-a8*Rz)) - (ay*(Pz-a8*Rz)-az*(Py-a8*Ry))*(Rz*(Px-a8*Rx)-Rx*(Pz-a8*Rz));simplify(T)
% T=((Pz-a8*Rz)*ax-(Px-a8*Rx)*az)*((Py-a8*Ry)*Rz-(Pz-a8*Rz)*Ry) - ((Px-a8*Rx)*Rz-(Pz-a8*Rz)*Rx)*((Pz-a8*Rz)*ay-(Py-a8*Ry)*az);simplify(T)
% T=(a8*Rz-Pz)*((Pz*ay-Py*az)*Rx+(Px*az-Pz*ax)*Ry+(Py*ax-Px*ay)*Rz); % T=(Pz-Rz*a8)*(Pzy*Rx+Pxz*Ry+Pyx*Rz);

Pn=(Px*az-Pz*ax)*ny+(Pz*ay-Py*az)*nx+(Py*ax-Px*ay)*nz;
Po=(Px*az-Pz*ax)*oy+(Pz*ay-Py*az)*ox+(Py*ax-Px*ay)*oz;
% T=a8*nz*Pn*cos^2+a8*oz*Po*sin^2-a8*(oz*Pn+nz*Po)*cos*sin-Pz*Pn*cos+Pz*Po*sin;  % cos(th9);sin(th9)

% T=A*cos^2+B*sin^2+C*cos*sin+D*cos+E*sin;
A=a8*nz*Pn;
B=a8*oz*Po;
C=-a8*(oz*Pn+nz*Po);
D=-Pz*Pn;
E=Pz*Po;
% x=[-pi:0.01:pi]; %ͼ����ʾ����2���4��
% y=A*cos(x).^2+B*sin(x).^2+C*cos(x).*sin(x)+D*cos(x)+E*sin(x);
% plot(x,y);    hold on;    plot(x,0*x);    figure;
% T=A*cos(th9)^2+B*sin(th9)^2+C*cos(th9)*sin(th9)+D*cos(th9)+E*sin(th9); %trigonomEric
% T=a*x^2+b*y^2+c*x*y+d*x+e*y; % x=cos; y=sin;  % ���Ǻ���ʽ
% syms A B C D E x; % T=((A-B)*x^2+D*x+B)^2-(C*x+E)^2*(1-x^2);  colleC(T,x) % T=a*x^4+b*x^3+c*x^2+d*x+e;
% Tx=(C^2+(A-B)^2)*x^4+(2*C*E+2*D*(A-B))*x^3+(-C^2+D^2+E^2+2*B*(A-B))*x^2+(2*B*D-2*C*E)*x+B^2-E^2;
% T=((B-A)*y^2+E*y+A)^2-(C*y+D)^2*(1-y^2);  colleC(T,y) 
% Ty=(C^2+(A-B)^2)*y^4+(2*C*D-2*E*(A-B))*y^3+(-C^2+D^2+E^2-2*A*(A-B))*y^2+(2*A*E-2*C*D)*y+A^2-D^2;

a=(C^2+(A-B)^2);
b=(2*C*E+2*D*(A-B));
c=(-C^2+D^2+E^2+2*B*(A-B));
d=(2*B*D-2*C*E);
e=B^2-E^2;

% x=[-1:0.01:1]; % �������������� -1<=x<=1
% fun_image4(x,a,b,c,d,e);
x=Ferrari41(a,b,c,d,e) %2���4��
%{
x=[-1:0.01:1]; % �������������� -1<=x<=1
fun_image4(x,a,b,c,d,e);
y=Ferrari41(a,b,c,d,e)  % x y�������˳��ͬ��
% ��ͬ���ķ������y=sin(th9),ͨ��x^2+y^2==1����xy��Ӧƥ��
% x.^2+y.^2;
%}

lenx=length(x);
% if lenx==2 % ����ɸѡ���򵥴ֱ�
%     the9(1:2)=atan2(sqrt(1-x.^2),x);
%     the9(3:4)=-the9(1:2);
% else %lenx==4
%     the9(1:4)=atan2(sqrt(1-x.^2),x);
%     the9(5:8)=atan2(-sqrt(1-x.^2),x);
% end
if lenx==2
    thx9(1:2)=acos(x); % acos()������������[-1,1]��ֵ��[0,pi]
    thx9(3:4)=-thx9(1:2);
else %lenx==4
    thx9(1:4)=acos(x);
    thx9(5:8)=-thx9(1:4);
end
%
Tri=A*cos(thx9).^2+B*sin(thx9).^2+C*cos(thx9).*sin(thx9)+D*cos(thx9)+E*sin(thx9);
j=1; %th9=[]; 

%x�ж�����ʱ����Ӧth9�ĸ�������2����Ч
%x���ĸ���ʱ����Ӧth9�˸�������4����Ч
for i=1:2*lenx % cos()sin()ɸѡ��ɸȥ��ƽ�������Ķ����
    if N_zero(Tri(i))==0
        the9(j)=thx9(i);
        j=j+1;
    end
end
% len9=length(the9);
% Pz/a8 <=1,��x����4�⣬ԭ���ҵ��ˡ�
Tri=Pz - a8*(nz*cos(the9)-oz*sin(the9));
% Tri=(Px*(ay*oz-az*oy)+Py*(az*ox-ax*oz)+Pz*(ax*oy-ay*ox))*sin(the9)+(Px*(az*ny-ay*nz)+Py*(ax*nz-az*nx)+Pz*(ay*nx-ax*ny))*cos(the9)
% ����ɸѡ��lenx==4�����2���⣬���Ǵ˼��㷨����������Զ�ˡ�
j=1;
if lenx==4 % ɸȥ(Pz - Rz*a8)==0�Ĳ���
    for i=1:4 
        if N_zero(Tri(i))~=0
            th9(j)=the9(i);
            j=j+1;
        end
    end
else
    th9=the9;
end
th90=th9*180/pi;  th90=vpa(th90,6)



%% th8
Rx= nx*cos(th9)-ox*sin(th9);
Ry= ny*cos(th9)-oy*sin(th9);
Rz= nz*cos(th9)-oz*sin(th9);
% (ax*(Pz-a8*Rz)-az*(Px-a8*Rx))*cos(th8) == (Rz*(Px-a8*Rx)-Rx*(Pz-a8*Rz))*sin(th8)
% (ay*(Pz-a8*Rz)-az*(Py-a8*Ry))*cos(th8) == (Rz*(Py-a8*Ry)-Ry*(Pz-a8*Rz))*sin(th8)
% tan1=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx))./(Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz)); tan1=vpa(tan1,6)
% tan2=(ay*(Pz-a8*Rz)-az*(Py-a8*Ry))./(Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz)); tan2=vpa(tan2,6) % tan(theta+pi)==tan(theta)
% th8=atan((ay*(Pz-a8*Rz)-az*(Py-a8*Ry))./(Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz)));
    m8=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx)); 
    n8=(Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz)); 
    th8(1:2)=atan2(m8,n8);
%     m82=(ay*(Pz-a8*Rz)-az*(Py-a8*Ry)); vpa(m82,6)
%     n82=(Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz)); vpa(n82,6)
%     th8(3:4)=atan2(m82,n82);    vpa(tan(th8),6)
%     th8(1:2)=atan2(ax*(Pz-a8*Rz)-az*(Px-a8*Rx),Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz)); %������� pi
%     th8(3:4)=atan2(ay*(Pz-a8*Rz)-az*(Py-a8*Ry),Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz)); %th8(3:4)=th8(1:2)+pi;
%     th8(1:2)=atan(k8); %vpa(th8,6) % atan(1/0)==pi/2
    th8(3:4)=th8(1:2)+pi;
    th9(3:4)=th9(1:2);

% %     th8(1:4)=atan((ax*(Pz-a8*Rz)-az*(Px-a8*Rx))./(Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz)));
%     m8=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx)); 
%     n8=(Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz)); 
%     the8(1:4)=atan2(m8,n8);
%     the8(5:8)=the8(1:4)+pi;
%     the9(5:8)=the9(1:4);
% %     vpa(tan(th8),6)

th80=th8*180/pi % th80=vpa(th80,6) 
% Rx= nx*cos(th9)-ox*sin(th9);
% Ry= ny*cos(th9)-oy*sin(th9);
% Rz= nz*cos(th9)-oz*sin(th9);
% T=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx)).*cos(th8)-(Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz)).*sin(th8);  vpa(T,6)  %��֤��Ϊ��
% T=(ay*(Pz-a8*Rz)-az*(Py-a8*Ry)).*cos(th8)-(Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz)).*sin(th8);  vpa(T,6)


%% d7 ����ɸѡ
% d7*M1==N1; ��Ϊd7�������㣻 M3==N3==0ʱ;��ʽ N3*M1==N1*M3 & N3*M2==N2*M3 �Գ�����
% ת�ۣ���Ȼ0*M1==0*N1�����Ǵ�ʱ d7~=N3/M3��M(i)N(i)==0���ǲ��ֽⲻ�ȣ����d7ʱ��Ҫ������ж���
% �ɢܢݢ޿�֪d7��ĸ����ȫ��Ϊ�㣬(cos(th4)*sin(th5))^2+(sin(th4)*sin(th5))^2+(cos(th5))^2~=0����Ҫif�жϰ���0�Ķ�����
% M1=(ax*cos(the8)+sin(the8).*(nx*cos(the9)-ox*sin(the9)));  MN(1,1:8)=vpa(M1,6);
% N1=(Px-a8*(nx*cos(the9)-ox*sin(the9)));  MN(2,1:8)=vpa(N1,6);
% M2=(ay*cos(the8)+sin(the8).*(ny*cos(the9)-oy*sin(the9)));  MN(3,1:8)=vpa(M2,6);
% N2=(Py-a8*(ny*cos(the9)-oy*sin(the9)));  MN(4,1:8)=vpa(N2,6);
Mz=(az*cos(th8)+sin(th8).*(nz*cos(th9)-oz*sin(th9)));  % MN(5,1:8)=vpa(M3,6);
Nz=(Pz-a8*(nz*cos(th9)-oz*sin(th9)));  % MN(6,1:8)=vpa(N3,6);

% j=1;
% if lenx==4
%     for i=1:8 % ɸѡ����th8th9���
%     % 	if N_zero(M3(i))==0 && N_zero(N3(i))==0
%         if N_zero(M3(i)) && N_zero(N3(i))
%             th8(j)=th8(i);
%             th9(j)=the9(i);
%             j=j+1;
%         end
%     end
% else
%     th8=th8;
%     th9=the9;    
% end


d7(1:4)=Nz./Mz;
d7(5:8)=d7(1:4);
% vpa(d7,6)
%{
% chu1=vpa(M1./M2,6)
% chu2=vpa(N1./N2,6)
% Tx= M1.*N3-M3.*N1;
% Ty= M2.*N3-M3.*N2;
% vpa(Tx,6)
% vpa(Ty,6)
% d7(1,1:8)=N1./M1;
% d7(2,1:8)=N2./M2;
% d7(3,1:8)=N3./M3;
% d7(1,1:8)=(px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9)))./(ax*cos(th8)+sin(th8).*(nx*cos(th9)-ox*sin(th9))); 
% d7(2,1:8)=(py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9)))./(ay*cos(th8)+sin(th8).*(ny*cos(th9)-oy*sin(th9)));  
% d7(3,1:8)=(pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9)))./(az*cos(th8)+sin(th8).*(nz*cos(th9)-oz*sin(th9))); 
%  vpa(d7,6) % ������% d7=d73;  d7(5:8)=d7(1:4);
% d7=d7(1,1:8);
% th9=th9(1:4);
% th8=th8(1:4);
% d7=d7(1,2:2:8);  d7(5:8)=d7(1:4);
% th9=th9(2:2:8);
% th8=th8(2:2:8);
%}


%% th5
% cos(th5) == az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))  ��
n5=az*cos(th8) + sin(th8).*(nz*cos(th9)-oz*sin(th9));
% m5=sqrt(1-n5.^2); %m5=-sqrt(1-n5.^2);
% th5(1:4)=atan2(m5,n5);
% th5(5:8)=atan2(-m5,n5);
th5(1:4)=acos(n5);
th5(5:8)=-th5(1:4);
th50=vpa(th5*180/pi,6)
th8(5:8)=th8(1:4);
th9(5:8)=th9(1:4);

%% th4 th6
if n5==1 % ���cos(th5)==1,sin(th5)==0�������
%{
% % -sin(th4)*sin(th6) + cos(th4)*cos(th6) == -ax*sin(th8) + cos(th8)*(nx*cos(th9)-ox*sin(th9))  
% %  cos(th4)*sin(th6) + sin(th4)*cos(th6) == -ay*sin(th8) + cos(th8)*(ny*cos(th9)-oy*sin(th9))  
% % cos(th4+th6)==n46;
% % sin(th4+th6)==m46;
% %     n46=-ax*sin(th8) + cos(th8).*(nx*cos(th9)-ox*sin(th9));
% %     m46=-ay*sin(th8) + cos(th8).*(ny*cos(th9)-oy*sin(th9));
% %     th46=atan2(m46,n46);
% %     th4(1:8)=0;
% %     th6=th46;
%}
%  sin(th4)*cos(th6) + cos(th4)*cos(th5)*sin(th6) == -ox*cos(th9)-nx*sin(th9)  ��
% -cos(th4)*cos(th6) + sin(th4)*cos(th5)*sin(th6) == -oy*cos(th9)-ny*sin(th9)  ��
%  sin(th4)*cos(th6) + cos(th4)*sin(th6) == -ox*cos(th9)-nx*sin(th9)
%  cos(th4)*cos(th6) - sin(th4)*sin(th6) ==  oy*cos(th9)+ny*sin(th9)
    m46 = -ox*cos(th9)-nx*sin(th9);
    n46 = oy*cos(th9)+ny*sin(th9);
    % sin(th4+th6)=m46 %
    % cos(th4+th6)=n46 %
    th46=atan2(m46,n46);
    th4=0;
    th6=th46;
    
else % ���sin(th5)~=0��������
    % th4 
    % -cos(th4)*sin(th5) == ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))  ��
    % -sin(th4)*sin(th5) == ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))  ��
    % cos(th4)==-(ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)))/sin(th5)
    % sin(th4)==-(ay*cos(th8)+sin(th8)*(ny*cos(th9)-oy*sin(th9)))/sin(th5)
    n4=-(ax*cos(th8)+sin(th8).*(nx*cos(th9)-ox*sin(th9)))./sin(th5);
    m4=-(ay*cos(th8)+sin(th8).*(ny*cos(th9)-oy*sin(th9)))./sin(th5);
    th4=atan2(m4,n4);
    th40=vpa(th4*180/pi,6)
    % th6
    % sin(th5)*cos(th6) == -az*sin(th8) + cos(th8)*(nz*cos(th9)-oz*sin(th9))  ��
    % sin(th5)*sin(th6) == -oz*cos(th9)-nz*sin(th9)  ��
    % cos(th6) == (-az*sin(th8)+cos(th8)*(nz*cos(th9)-oz*sin(th9)))/sin(th5)
    % sin(th6) == (-oz*cos(th9)-nz*sin(th9))/sin(th5)
    n6=(-az*sin(th8)+cos(th8).*(nz*cos(th9)-oz*sin(th9)))./sin(th5); %��Ϊatan2��������������أ����ܻ����ڲ�ͬ���ޣ�Ӱ�����ս��
    m6=(-oz*cos(th9)-nz*sin(th9))./sin(th5);
    th6=atan2(m6,n6);
    th60=vpa(th6*180/pi,6)
end

%

%% sort out;
    for i=1:8
        th(i,9)=th9(i);
        th(i,8)=In_pi(th8(i));
        th(i,7)=d7(i);
        th(i,5)=th5(i);
        th(i,4)=th4(i);
        th(i,6)=th6(i);
    end
theta_deg=vpa(th*180/pi,6)
% T1=forward_kine(th(1,:));  vpa(T1-T,6)

for i=1:8
%     eval(['Q',num2str(i),'=','vpa(forward_kine(th(i,:))-T,6)']);
    T8=forward_kine(th(i,1:9))-T;
    eval(['Q',num2str(i),'=','vpa(sum(sum(T8)),6)']);
end

%{ 
%һ��������ɵ�һԪ�Ĵη��̣������ĸ���
a=(2*rand(1)-1);
b=(2*rand(1)-1);
c=(2*rand(1)-1);
d=(2*rand(1)-1);
e=(2*rand(1)-1);
x=Ferrari41(a,b,c,d,e) 
if ~isempty(x) %x=[]
    f = @(x) a*x^4+b*x^3+c*x^2+d*x+e;  
    y(1)=f(x(1));y(2)=f(x(2));y   %�����2��
end
%}

%% function
function th=ikine_Virtual(T)
% function th=Virtual6(T)
% ԭsimplify�򻯰汾��ɾ�����м��Ƶ����̣�ֻ���������õĴ��룻
    a8=50;a9=50;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    % th9
    Px=px-a9*nx;
    Py=py-a9*ny;
    Pz=pz-a9*nz;    
    Pn=Px*(az*ny-ay*nz)+Py*(ax*nz-az*nx)+Pz*(ay*nx-ax*ny);
    Po=Px*(az*oy-ay*oz)+Py*(ax*oz-az*ox)+Pz*(ay*ox-ax*oy);
    
    A=a8*nz*Pn;
    B=a8*oz*Po;
    C=-a8*(oz*Pn+nz*Po);
    D=-Pz*Pn;
    E=Pz*Po;
    a=(C^2+(A-B)^2);
    b=(2*C*E+2*D*(A-B));
    c=(-C^2+D^2+E^2+2*B*(A-B));
    d=(2*B*D-2*C*E);
    e=B^2-E^2;
    x=Ferrari41(a,b,c,d,e); 
%     x=real(x)
    lenx=length(x);
    if lenx==2;
        thx9(1:2)=acos(x); 
        thx9(3:4)=-thx9(1:2);
    else
        thx9(1:4)=acos(x);
        thx9(5:8)=-thx9(1:4);
    end
    Tri=A*cos(thx9).^2+B*sin(thx9).^2+C*cos(thx9).*sin(thx9)+D*cos(thx9)+E*sin(thx9);
    j=1;
    for i=1:2*lenx % cos()sin()ɸѡ
        if N_zero(Tri(i))==0
            the9(j)=thx9(i);
            j=j+1;
        end
    end
    Tri=Pz - a8*(nz*cos(the9)-oz*sin(the9));
    j=1;
    if lenx==4 % ɸȥ(Pz - Rz*a8)==0�Ĳ���
        for i=1:4 
            if N_zero(Tri(i))~=0
                th9(j)=the9(i);
                j=j+1;
            end
        end
    else
        th9=the9;
    end
    % th8
    Rx= nx*cos(th9)-ox*sin(th9);
    Ry= ny*cos(th9)-oy*sin(th9);
    Rz= nz*cos(th9)-oz*sin(th9);
    m8=(ax*(Pz-a8*Rz)-az*(Px-a8*Rx)); 
    n8=(Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz)); 
    th8(1:2)=atan2(m8,n8);
    th8(3:4)=th8(1:2)+pi;
    th9(3:4)=th9(1:2);
    % d7
    Mz=(az*cos(th8)+sin(th8).*(nz*cos(th9)-oz*sin(th9))); 
    Nz=(Pz-a8*(nz*cos(th9)-oz*sin(th9))); 
    d7(1:4)=Nz./Mz;
    d7(5:8)=d7(1:4);
    
    % th5 th6 th4
    n5=az*cos(th8) + sin(th8).*(nz*cos(th9)-oz*sin(th9));
    th5(1:4)=acos(n5);
    th5(5:8)=-th5(1:4);
    
    th8(5:8)=th8(1:4);
    th9(5:8)=th9(1:4);
    if n5==1
        n46=-ax*sin(th8) + cos(th8).*(nx*cos(th9)-ox*sin(th9));
        m46=-ay*sin(th8) + cos(th8).*(ny*cos(th9)-oy*sin(th9));
        th46=atan2(m46,n46);
        th4(1:8)=0;
        th6=th46;
    else
        n4=-(ax*cos(th8)+sin(th8).*(nx*cos(th9)-ox*sin(th9)))./sin(th5);
        m4=-(ay*cos(th8)+sin(th8).*(ny*cos(th9)-oy*sin(th9)))./sin(th5);
        th4=atan2(m4,n4);
        n6=(-az*sin(th8)+cos(th8).*(nz*cos(th9)-oz*sin(th9)))./sin(th5);
        m6=(-oz*cos(th9)-nz*sin(th9))./sin(th5);
        th6=atan2(m6,n6);
    end
    
    for i=1:8
        th(i,9)=th9(i);
        th(i,8)=In_pi(th8(i));
        th(i,7)=d7(i); %*pi/180;  %����ת�Ƕ�
        th(i,5)=th5(i);
        th(i,4)=th4(i);
        th(i,6)=th6(i);
    end
end

%% equation ��������
% RCMpoint-O9point ������������⣻Զ�ĵ�RCM (Remote Center Motion)
function T = equation(theta) %��ʽ����  Tq*T9i*T8i==T4i*T5*T6*T7
    syms a8 a9; syms d1 d4 d5 d6 a2 a3;    syms d7;
    d=[d1 0 0 d4 d5 d6 d7 0 0];    a=[0 a2 a3 0 0 0 0 a8 a9];
    alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
    % T0=DH_forward(0,0,0,pi/2); %O0 to O6����̬ת�� 
    %��O6�㣬����λ������ͬ�ģ�ZYZ��ϣ�
    T4=DH_forward(theta(4),0,0,alpha(4)); % T4=DH_forward(theta(4),0,0,pi/2);
    T5=DH_forward(theta(5),0,0,alpha(5)); % T5=DH_forward(theta(5),0,0,-pi/2);
    T6=DH_forward(theta(6),0,0,alpha(6)); % T6=DH_forward(theta(6),0,0,0);
    T7=DH_forward(0,theta(7),a(7),alpha(7)); % T7=DH_forward(0,theta(7),0,pi/2);
    T8=DH_forward(theta(8),d(8),a(8),alpha(8)); % T8=DH_forward(theta(8),0,a8,-pi/2);
    T9=DH_forward(theta(9),d(9),a(9),alpha(9)); % T9=DH_forward(theta(9),0,a9,0);
    T=T4*T5*T6*T7*T8*T9; %simplify(T)
    global py;
    syms nx ox ax px ny oy py ay nz oz az pz;  %th1 th2 th3 th4 th5 th6 d1 d4 d5 d6 a2 a3 
    Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1]; %����py���ܷ��ں����ڲ�����global��
    % T4i=DH_inverse(theta(4),0,0,alpha(4));
%     T7i=DH_inverse(0,theta(7),0,pi/2);
    T8i=DH_inverse(theta(8),d(8),a(8),alpha(8));%     T8i=DH_inverse(theta(8),0,a8,-pi/2);
    T9i=DH_inverse(theta(9),d(9),a(9),alpha(9));%     T9i=DH_inverse(theta(9),0,a9,0);
%     Tq98=Tq*T9i*T8i;   % simplify(Tq98)
end


%% linear_function
function x=Ferrari41(a,b,c,d,e)    % �����﷨�� һԪ�Ĵη���
    b1=b/a;  c1=c/a;  d1=d/a;  e1=e/a; %a1=1
    a=-1;  b=c1;  c=(4*e1-b1*d1);  d=d1^2-e1*(4*c1-b1^2);
    y=Cardano31(a,b,c,d); % y=y(1);
    a2=(1/4*b1^2-c1+y);
    b2=(1/2*b1*y-d1); 
    c2=(1/4*y^2-e1);
    b=b1/2+sqrt(a2);
    c=y/2+sign(b2)*sqrt(c2);
    xc(1)=(-b+sqrt(b^2-4*c))/2; % a=1;
    xc(2)=(-b-sqrt(b^2-4*c))/2;
    b=b1/2-sqrt(a2);
    c=y/2-sign(b2)*sqrt(c2);
    xc(3)=(-b+sqrt(b^2-4*c))/2;
    xc(4)=(-b-sqrt(b^2-4*c))/2;
    j=1; 
    for i=1:length(xc) % ɸѡʵ����
        if isreal(xc(i))
            x(j)=xc(i);
            j=j+1;
        end
    end
end
function x=Cardano31(a,b,c,d)        % ��������ʽ
    b=b/a;  c=c/a;  d=d/a; % a=1;
    p=-b^2/3+c; 
    q=(2*b^3)/27-(c*b)/3+d;
    delta=(q/2)^2+(p/3)^3;  %delta �ж�
    if delta>=0
        m=nthroot(-q/2+sqrt(delta),3); 
        n=nthroot(-q/2-sqrt(delta),3);
    else
        m=(-q/2+sqrt(delta))^(1/3);
        n=(-q/2-sqrt(delta))^(1/3);
    end
    x=m+n-b/3;
end
%% standard
function mn = N_zero(mn)
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<1E-6
                mn(i,j)=0;
            end
        end
    end
end   
function theta = In_pi(theta)
    while (abs(theta)>pi)
        if (theta>pi)
            theta=theta-2*pi;
        elseif (theta<-pi)
            theta=theta+2*pi;
        end
    end
end
%% kinetic_function
function T = forward_kine49(theta) % virtual����ר�ú��������˶�λ�˻�ȡ
    d=[0 0 0 0 0 0 0 0 0]; %syms  a8 a9 d7; %     a8=50;a9=50; %d7=0;
    a=[0 0 0 0 0 0 0 50 50]; 
    alpha=[0 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0]; %ר��alpha����
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(0,theta(7),a(7),alpha(7)); 
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T4*T5*T6*T7*T8*T9;
end

function T = forward_kine(theta)% virtual6 ר�ú���
    syms  a8 a9 d7;
    a8=50;a9=50; %d7=0;
    d=[0 0 0 0 0 0 0 0 0];
    a=[0 0 0 0 0 0 0 a8 a9];
    alpha=[0 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(0,theta(7),a(7),alpha(7)); 
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T4*T5*T6*T7*T8*T9;  %     T=T4*T5*T6*T7; %
    %{
    T8i=DH_inverse(theta(8),d(8),a(8),alpha(8));%     T8i=DH_inverse(theta(8),0,a8,-pi/2);
    T9i=DH_inverse(theta(9),d(9),a(9),alpha(9));%     T9i=DH_inverse(theta(9),0,a9,0);
    global py;syms nx ox ax px ny oy py ay nz oz az pz; 
    Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];
    T=Tq*T9i*T8i;  %Tq987 %ȷ��ʹ��ͬһ����
    %}
end
function T = DH_forward(theta,d,a,alpha) % ���˶�����
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end
function y = fun_image4(x,a,b,c,d,e)
    y=a*x.^4+b*x.^3+c*x.^2+d.*x+e;
    plot(x,y);
    hold on;
    plot(x,0*x);
    syms x
    a=vpa(a,2);b=vpa(b,2);c=vpa(c,2);d=vpa(d,2);e=vpa(e,2);
    Y=a*x.^4+b*x.^3+c*x.^2+d.*x+e
end


