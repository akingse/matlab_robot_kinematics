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

%% 五次多项式插值，走路径点
clear;
clc;
q_array=[0,50,150,100,40];%指定起止位置
t_array=[0,3,6,12,14];%指定起止时间
v_array=[0,10,20,-15,0];%指定起止速度
a_array=[0,20,30,-20,0];%指定起止加速度
t=[t_array(1)];q=[q_array(1)];v=[v_array(1)];a=[a_array(1)];%初始状态
for i=1:1:length(q_array)-1;%每一段规划的时间
     T=t_array(i+1)-t_array(i)
     a0=q_array(i);
     a1=v_array(i);
     a2=a_array(i)/2;
     a3=(20*q_array(i+1)-20*q_array(i)-(8*v_array(i+1)+12*v_array(i))*T-(3*a_array(i)-a_array(i+1))*T^2)/(2*T^3);
     a4=(30*q_array(i)-30*q_array(i+1)+(14*v_array(i+1)+16*v_array(i))*T+(3*a_array(i)-2*a_array(i+1))*T^2)/(2*T^4);
     a5=(12*q_array(i+1)-12*q_array(i)-(6*v_array(i+1)+6*v_array(i))*T-(a_array(i)-a_array(i+1))*T^2)/(2*T^5);%计算五次多项式系数 
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

%% 直线插补
clear;
clc;
p0=[1,2,3];
pf=[2,4,5];         %指定起止位置
v=0.1;              %指定速度
x=[p0(1)];y=[p0(2)];z=[p0(3)];
L=((pf(1)-p0(1))^2+(pf(2)-p0(2))^2+(pf(3)-p0(3))^2)^0.5;%直线长度
N=L/v;              %插补次数
dx=(pf(1)-p0(1))/N; %每个周期各轴增量
dy=(pf(2)-p0(2))/N;
dz=(pf(3)-p0(3))/N;
for t=1:1:N         %插补
x(t+1)=x(t)+dx;
y(t+1)=y(t)+dy;
z(t+1)=z(t)+dz;
end
% plot3(x,y,z,'r'),xlabel('x'),ylabel('y'),zlabel('z'),hold on,plot3(x,y,z,'o','color','g'),grid on;

%% 五段位置s曲线插补
% 六轴机器人轨迹规划之五段位置s曲线插补
clc;
clear;
%初始条件
x_arry=[0,10,20,30];
v_arry=[2,2,2];
A_arry=[3,3,3];
weiyi=[x_arry(1)];sudu=[0];shijian=[0];timeall=0;jiasudu=[0]
for i=1:1:length(x_arry)-1;
%清空
    a=[];v=[];s=[];
%计算加减速段的时间和位移
    L=x_arry(i+1)-x_arry(i);
    A=A_arry(i);
    vs=v_arry(i);
    Ta=sqrt(vs/A);
    L1=A*(Ta^3)/6;
    L2=A*(Ta^3)*(5/6); 
%计算整段轨迹的总位移
    T=4*Ta+(L-2*L1-2*L2)/vs;
    for t=0:0.001:T
        if t<=Ta;%加加速度阶段
            ad=A*t;
            vd=0.5*A*t^2;
            sd=(1/6)*A*t^3;
            a=[a,ad];v=[v,vd];s=[s,sd];
        elseif t>Ta && t<=2*Ta;%加减速阶段
            ad=-A*(t-2*Ta);
            vd=-0.5*A*(t-2*Ta)^2+A*Ta^2;
            sd=-(1/6)*A*(t-2*Ta)^3+A*Ta^2*t-A*Ta^3;
            a=[a,ad];v=[v,vd];s=[s,sd];
         elseif t>2*Ta && t<=T-2*Ta;%匀速阶段
            ad=0;
            vd=vs;
            sd=A*Ta^2*t-A*Ta^3;  
            a=[a,ad];v=[v,vd];s=[s,sd];
        elseif t>T-2*Ta && t<=T-Ta;%减加度阶段
            ad=-A*(t-(T-2*Ta));
            vd=-0.5*A*(t-T+2*Ta)^2+A*Ta^2;
            sd=-(1/6)*A*(t-T+2*Ta)^3+A*Ta^2*t-A*Ta^3;
            a=[a,ad];v=[v,vd];s=[s,sd];
         elseif t>T-Ta && t<=T;%减减阶段
            ad=A*(t-T);
            vd=0.5*A*(t-T)^2;
            sd=(1/6)*A*(t-T)^3-2*A*Ta^3+A*Ta^2*T;
            a=[a,ad];v=[v,vd];s=[s,sd];
        end
    end
%时间
    time=[timeall:0.001:timeall+T];
    timeall=timeall+T;
%连接每一段轨迹
    weiyi=[weiyi,s(2:end)+x_arry(i)];
    sudu=[sudu,v(2:end)];
    jiasudu=[jiasudu,a(2:end)];
    shijian=[shijian,time(2:end)];
end
% subplot(3,1,1),plot(shijian,weiyi,'r');xlabel('t'),ylabel('position');grid on;
% subplot(3,1,2),plot(shijian,sudu,'b');xlabel('t'),ylabel('velocity');grid on;
% subplot(3,1,3),plot(shijian,jiasudu,'g');xlabel('t'),ylabel('accelerate');grid on;

%% 牛顿欧拉法
% 动力学建模
syms l1 l2 m1 m2 g;
syms q1 q2 dq1 dq2 ddq1 ddq2;

% 参数初始化
R{1}=[cos(q1) -sin(q1) 0;sin(q1) cos(q1) 0;0 0 1];
R{2}=[cos(q2) -sin(q2) 0;sin(q2) cos(q2) 0;0 0 1];
R{3}=[1 0 0;0 1 0;0 0 1];
% 坐标系原点位移，用P{1}表示坐标系1与坐标系0原点位置关系，用P{2}表示坐标系2与坐标系1原点位置关系。
P = cell(1,3);
P{1}=[0;0;0];P{2}=[l1;0;0];P{3}=[l2;0;0];
% 每个连杆质心的位置矢量
Pc = cell(1,3);
Pc{1}=[0;0;0];Pc{2}=[l1;0;0];Pc{3}=[l2;0;0];
% 连杆质量
m = cell(1,3);
m{2}=m1;
m{3}=m2;
% 惯性张量
I = cell(1,3);
I{2}=[0;0;0];
I{3}=[0;0;0];

% 连杆间角速度和角加速度
w = cell(1,3);dw = cell(1,3);
w{1}=[0;0;0];dw{1}=[0;0;0];% 机器人底座不旋转
% 连杆坐标原点和质心加速度
dv = cell(1,3);dvc = cell(1,3);
dv{1}=[0;g;0];% 重力因素

% 关节速度和加速度
dq = cell(1,3); ddq = cell(1,3); 
dq{2}=[0;0;dq1];dq{3}=[0;0;dq2];
ddq{2}=[0;0;ddq1];ddq{3}=[0;0;ddq2];

% 末端执行器没有力
f = cell(1,4);n = cell(1,4);
f{4}=[0;0;0];
n{4}=[0;0;0];

%% 建立运动学方程
% 外推
for i=1:2 %matlab下标从1开始
w{i+1}=R{i}.'*w{i}+dq{i+1};
dw{i+1}=R{i}.'*dw{i}+cross(R{i}.'*w{i},dq{i+1})+ddq{i+1};

dv{i+1}=R{i}.'*(cross(dw{i},P{i})+cross(w{i},cross(w{i},P{i}))+dv{i});
dvc{i+1}=cross(dw{i+1},Pc{i+1})+cross(w{i+1},cross(w{i+1},Pc{i+1}))+dv{i+1};

F{i+1}=m{i+1}*dvc{i+1};
N{i+1}=[0;0;0];%假设质量集中，每个连杆惯性张量为0
end

% 内推
for i=3:-1:2
    f{i}=R{i}*f{i+1}+F{i};
    n{i}=N{i}+R{i}*n{i+1}+cross(Pc{i},F{i})+cross(P{i},R{i}*f{i+1});
end

% 力矩
tau = cell(1,2);
tau{1} = n{2}(3);
tau{2} = n{3}(3);
celldisp(tau)



%% 拉格朗日法
% 质量集中，无惯性矩阵，连杆长度与质心位置重合
syms m1 m2 l1 l2 g 
syms q1 q2 q1d q2d q1dd q2dd 
syms x1(t) x1d x1dd x2(t) x2d x2dd

x1d=diff(x1,t); x2d=diff(x2,t);
x1dd=diff(x1,t,t); x2dd=diff(x2,t,t);

%质量矩阵
M1=diag([m1,m1]);
M2=diag([m2,m2]);

% 速度
V1=[[l1*cos(x1(t)) 0];[l1*sin(x1(t)) 0]]*[x1d;x2d];
V2=[[-l1*sin(x1(t))-l2*sin(x1(t)+x2(t)) -l2*sin(x1(t)+x2(t))];
    [l1*cos(x1(t))+l2*cos(x1(t)+x2(t)) l2*cos(x1(t)+x2(t))]]*[x1d;x2d];

% 动能
K1=simplify((1/2)*V1.'*M1*V1);
K2=simplify((1/2)*V2.'*M2*V2);
K =K1+K2;

% 势能
u1=m1*g*l1*sin(q1);
u2=m2*g*(l1*sin(q1)+l2*sin(q1+q2));
u =u1+u2;

% 拉格朗日方程
L=K-u;
L=subs(L,{x1,x2,x1d,x2d,x1dd,x2dd},{q1,q2,q1d,q2d,q1dd,q2dd});

% 利用运动方程计算力矩方程
dLdqd=[diff(L,q1d); diff(L,q2d)];
dLdqd =subs(dLdqd, {q1,q2,q1d,q2d,q1dd,q2dd}, {x1,x2,x1d,x2d,x1dd,x2dd});
ddLdqddt=diff(dLdqd,t);
ddLdqddt= subs(ddLdqddt,{x1,x2,x1d,x2d,x1dd,x2dd},{q1,q2,q1d,q2d,q1dd,q2dd});
dLdq=[diff(L,q1); diff(L,q2)];

% 输出结果
syms f
f=simplify(ddLdqddt-dLdq)

