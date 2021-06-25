%% initialize
clc; close all; clear all;
format shortg; format compact;

%{
% function dx=jiqiren(t,x) 
    lc1=0.5;    lc2=0.5;l1=1;r=1/25;
    bm=0.015;bl=0;
    m1=50;m2=50;
    I1=10;I2=10;
    a11=2300;a22=1800;
    b11=1300;b22=1500;
    c11=40;c12=0;c21=0;c22=40;
     d11=m1*lc1^2+m2*(l1^2+lc2^2+2*l1*lc2^2+2*l1*lc2*cos(x(3)))+I1+I2;
     d12=m2(lc2^2+l1*lc2*cos(x(3)))+I2;
     d21=m2(lc2^2+l1*lc2*cos(x(3)))+I2;
     d22=m2*lc2^2+I2;
     qd1=1;
qd2=1;
H1=x(1)-qd1;
H2=x(3)-qd2;
H11=diff(H1,1);
H22=diff(H2,1);
u1=-(a11*H1+c11*int(H1,0,t)+b11*H11);
u2=-(a22*H2+c22*int(H2,0,t)+b22*H22);
T1=u1-(r*bm+bl)*x(2);
T2=u2-(r*bm+bl)*x(4);
 dx=[x(2);
 ((d22*c11-d11*c21)/(d21*d12-d22*d11))*x(2)+((d22*c12-d11*c22)/(d21*d12-d22*d11))*x(4)+(d11/d21*d12-d22*d11)*T2-(d22/(d21*d12-d22*d11))*T1+(d22/(d21*d12-d22*d11))*((m1*lc1+m2*l1)*g*cos(x(1))+m2*lc2*g*cos(x(1)+x(3)))-(d22/(d21*d12-d22*d11))*(m2*lc2*g*cos(x(1)+x(3)));
 x(4);
 -(((d21*c11-d11*c21)/(d21*d12-d22*d11))*x(2)+((d21*c12-d11*c22)/(d21*d12-d22*d11))*x(4)+(d11/(d21*d12-d22*d11))*T2-(d21/(d21*d12-d22*d11))*T1)];
 x0=[0;0;0;0];
[t,y1]=ode45(@jiqiren,[0,1],x0);
 plot(t,y(:,1));
%}

%% ��ζ���ʽ��ֵ����·����
clear;
clc;
q_array=[0,50,150,100,40];%ָ����ֹλ��
t_array=[0,3,6,12,14];%ָ����ֹʱ��
v_array=[0,10,20,-15,0];%ָ����ֹ�ٶ�
a_array=[0,20,30,-20,0];%ָ����ֹ���ٶ�
t=[t_array(1)];q=[q_array(1)];v=[v_array(1)];a=[a_array(1)];%��ʼ״̬
for i=1:1:length(q_array)-1;%ÿһ�ι滮��ʱ��
     T=t_array(i+1)-t_array(i)
     a0=q_array(i);
     a1=v_array(i);
     a2=a_array(i)/2;
     a3=(20*q_array(i+1)-20*q_array(i)-(8*v_array(i+1)+12*v_array(i))*T-(3*a_array(i)-a_array(i+1))*T^2)/(2*T^3);
     a4=(30*q_array(i)-30*q_array(i+1)+(14*v_array(i+1)+16*v_array(i))*T+(3*a_array(i)-2*a_array(i+1))*T^2)/(2*T^4);
     a5=(12*q_array(i+1)-12*q_array(i)-(6*v_array(i+1)+6*v_array(i))*T-(a_array(i)-a_array(i+1))*T^2)/(2*T^5);%������ζ���ʽϵ�� 
     ti=t_array(i):0.001:t_array(i+1);
     qi=a0+a1*(ti-t_array(i))+a2*(ti-t_array(i)).^2+a3*(ti-t_array(i)).^3+a4*(ti-t_array(i)).^4+a5*(ti-t_array(i)).^5;
     vi=a1+2*a2*(ti-t_array(i))+3*a3*(ti-t_array(i)).^2+4*a4*(ti-t_array(i)).^3+5*a5*(ti-t_array(i)).^4;
     ai=2*a2+6*a3*(ti-t_array(i))+12*a4*(ti-t_array(i)).^2+20*a5*(ti-t_array(i)).^3;
     t=[t,ti(2:end)];q=[q,qi(2:end)];v=[v,vi(2:end)];a=[a,ai(2:end)];
end
% subplot(3,1,1),plot(t,q,'r'),xlabel('t'),ylabel('position');hold on;plot(t_array,q_array,'o','color','g'),grid on;
% subplot(3,1,2),plot(t,v,'b'),xlabel('t'),ylabel('velocity');hold on;plot(t_array,v_array,'*','color','y'),grid on;
% subplot(3,1,3),plot(t,a,'g'),xlabel('t'),ylabel('accelerate');hold on;plot(t_array,a_array,'^','color','r'),grid on;
% 

%% ֱ�߲岹
clear;
clc;
p0=[1,2,3];
pf=[2,4,5];         %ָ����ֹλ��
v=0.1;              %ָ���ٶ�
x=[p0(1)];y=[p0(2)];z=[p0(3)];
L=((pf(1)-p0(1))^2+(pf(2)-p0(2))^2+(pf(3)-p0(3))^2)^0.5;%ֱ�߳���
N=L/v;              %�岹����
dx=(pf(1)-p0(1))/N; %ÿ�����ڸ�������
dy=(pf(2)-p0(2))/N;
dz=(pf(3)-p0(3))/N;
for t=1:1:N         %�岹
x(t+1)=x(t)+dx;
y(t+1)=y(t)+dy;
z(t+1)=z(t)+dz;
end
% plot3(x,y,z,'r'),xlabel('x'),ylabel('y'),zlabel('z'),hold on,plot3(x,y,z,'o','color','g'),grid on;

%% ���λ��s���߲岹
% ��������˹켣�滮֮���λ��s���߲岹
clc;
clear;
%��ʼ����
x_arry=[0,10,20,30];
v_arry=[2,2,2];
A_arry=[3,3,3];
weiyi=[x_arry(1)];sudu=[0];shijian=[0];timeall=0;jiasudu=[0]
for i=1:1:length(x_arry)-1;
%���
    a=[];v=[];s=[];
%����Ӽ��ٶε�ʱ���λ��
    L=x_arry(i+1)-x_arry(i);
    A=A_arry(i);
    vs=v_arry(i);
    Ta=sqrt(vs/A);
    L1=A*(Ta^3)/6;
    L2=A*(Ta^3)*(5/6); 
%�������ι켣����λ��
    T=4*Ta+(L-2*L1-2*L2)/vs;
    for t=0:0.001:T
        if t<=Ta;%�Ӽ��ٶȽ׶�
            ad=A*t;
            vd=0.5*A*t^2;
            sd=(1/6)*A*t^3;
            a=[a,ad];v=[v,vd];s=[s,sd];
        elseif t>Ta && t<=2*Ta;%�Ӽ��ٽ׶�
            ad=-A*(t-2*Ta);
            vd=-0.5*A*(t-2*Ta)^2+A*Ta^2;
            sd=-(1/6)*A*(t-2*Ta)^3+A*Ta^2*t-A*Ta^3;
            a=[a,ad];v=[v,vd];s=[s,sd];
         elseif t>2*Ta && t<=T-2*Ta;%���ٽ׶�
            ad=0;
            vd=vs;
            sd=A*Ta^2*t-A*Ta^3;  
            a=[a,ad];v=[v,vd];s=[s,sd];
        elseif t>T-2*Ta && t<=T-Ta;%���ӶȽ׶�
            ad=-A*(t-(T-2*Ta));
            vd=-0.5*A*(t-T+2*Ta)^2+A*Ta^2;
            sd=-(1/6)*A*(t-T+2*Ta)^3+A*Ta^2*t-A*Ta^3;
            a=[a,ad];v=[v,vd];s=[s,sd];
         elseif t>T-Ta && t<=T;%�����׶�
            ad=A*(t-T);
            vd=0.5*A*(t-T)^2;
            sd=(1/6)*A*(t-T)^3-2*A*Ta^3+A*Ta^2*T;
            a=[a,ad];v=[v,vd];s=[s,sd];
        end
    end
%ʱ��
    time=[timeall:0.001:timeall+T];
    timeall=timeall+T;
%����ÿһ�ι켣
    weiyi=[weiyi,s(2:end)+x_arry(i)];
    sudu=[sudu,v(2:end)];
    jiasudu=[jiasudu,a(2:end)];
    shijian=[shijian,time(2:end)];
end
% subplot(3,1,1),plot(shijian,weiyi,'r');xlabel('t'),ylabel('position');grid on;
% subplot(3,1,2),plot(shijian,sudu,'b');xlabel('t'),ylabel('velocity');grid on;
% subplot(3,1,3),plot(shijian,jiasudu,'g');xlabel('t'),ylabel('accelerate');grid on;

%% ţ��ŷ����
% ����ѧ��ģ
syms l1 l2 m1 m2 g;
syms q1 q2 dq1 dq2 ddq1 ddq2;

% ������ʼ��
R{1}=[cos(q1) -sin(q1) 0;sin(q1) cos(q1) 0;0 0 1];
R{2}=[cos(q2) -sin(q2) 0;sin(q2) cos(q2) 0;0 0 1];
R{3}=[1 0 0;0 1 0;0 0 1];
% ����ϵԭ��λ�ƣ���P{1}��ʾ����ϵ1������ϵ0ԭ��λ�ù�ϵ����P{2}��ʾ����ϵ2������ϵ1ԭ��λ�ù�ϵ��
P = cell(1,3);
P{1}=[0;0;0];P{2}=[l1;0;0];P{3}=[l2;0;0];
% ÿ���������ĵ�λ��ʸ��
Pc = cell(1,3);
Pc{1}=[0;0;0];Pc{2}=[l1;0;0];Pc{3}=[l2;0;0];
% ��������
m = cell(1,3);
m{2}=m1;
m{3}=m2;
% ��������
I = cell(1,3);
I{2}=[0;0;0];
I{3}=[0;0;0];

% ���˼���ٶȺͽǼ��ٶ�
w = cell(1,3);dw = cell(1,3);
w{1}=[0;0;0];dw{1}=[0;0;0];% �����˵�������ת
% ��������ԭ������ļ��ٶ�
dv = cell(1,3);dvc = cell(1,3);
dv{1}=[0;g;0];% ��������

% �ؽ��ٶȺͼ��ٶ�
dq = cell(1,3); ddq = cell(1,3); 
dq{2}=[0;0;dq1];dq{3}=[0;0;dq2];
ddq{2}=[0;0;ddq1];ddq{3}=[0;0;ddq2];

% ĩ��ִ����û����
f = cell(1,4);n = cell(1,4);
f{4}=[0;0;0];
n{4}=[0;0;0];

%% �����˶�ѧ����
% ����
for i=1:2 %matlab�±��1��ʼ
w{i+1}=R{i}.'*w{i}+dq{i+1};
dw{i+1}=R{i}.'*dw{i}+cross(R{i}.'*w{i},dq{i+1})+ddq{i+1};

dv{i+1}=R{i}.'*(cross(dw{i},P{i})+cross(w{i},cross(w{i},P{i}))+dv{i});
dvc{i+1}=cross(dw{i+1},Pc{i+1})+cross(w{i+1},cross(w{i+1},Pc{i+1}))+dv{i+1};

F{i+1}=m{i+1}*dvc{i+1};
N{i+1}=[0;0;0];%�����������У�ÿ�����˹�������Ϊ0
end

% ����
for i=3:-1:2
    f{i}=R{i}*f{i+1}+F{i};
    n{i}=N{i}+R{i}*n{i+1}+cross(Pc{i},F{i})+cross(P{i},R{i}*f{i+1});
end

% ����
tau = cell(1,2);
tau{1} = n{2}(3);
tau{2} = n{3}(3);
celldisp(tau)



%% �������շ�
% �������У��޹��Ծ������˳���������λ���غ�
syms m1 m2 l1 l2 g 
syms q1 q2 q1d q2d q1dd q2dd 
syms x1(t) x1d x1dd x2(t) x2d x2dd

x1d=diff(x1,t); x2d=diff(x2,t);
x1dd=diff(x1,t,t); x2dd=diff(x2,t,t);

%��������
M1=diag([m1,m1]);
M2=diag([m2,m2]);

% �ٶ�
V1=[[l1*cos(x1(t)) 0];[l1*sin(x1(t)) 0]]*[x1d;x2d];
V2=[[-l1*sin(x1(t))-l2*sin(x1(t)+x2(t)) -l2*sin(x1(t)+x2(t))];
    [l1*cos(x1(t))+l2*cos(x1(t)+x2(t)) l2*cos(x1(t)+x2(t))]]*[x1d;x2d];

% ����
K1=simplify((1/2)*V1.'*M1*V1);
K2=simplify((1/2)*V2.'*M2*V2);
K =K1+K2;

% ����
u1=m1*g*l1*sin(q1);
u2=m2*g*(l1*sin(q1)+l2*sin(q1+q2));
u =u1+u2;

% �������շ���
L=K-u;
L=subs(L,{x1,x2,x1d,x2d,x1dd,x2dd},{q1,q2,q1d,q2d,q1dd,q2dd});

% �����˶����̼������ط���
dLdqd=[diff(L,q1d); diff(L,q2d)];
dLdqd =subs(dLdqd, {q1,q2,q1d,q2d,q1dd,q2dd}, {x1,x2,x1d,x2d,x1dd,x2dd});
ddLdqddt=diff(dLdqd,t);
ddLdqddt= subs(ddLdqddt,{x1,x2,x1d,x2d,x1dd,x2dd},{q1,q2,q1d,q2d,q1dd,q2dd});
dLdq=[diff(L,q1); diff(L,q2)];

% ������
syms f
f=simplify(ddLdqddt-dLdq)

