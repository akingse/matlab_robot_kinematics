clear all;
clc;

%% mdl_motomanHP6;
%            theta      d      a      alpha
L(1) = Link([ 0       0     0.15   pi/2   0]); L(1).qlim=[-170/180*pi,170/180*pi];
L(2) = Link([ 0       -0.170     0.8     0    0]); L(2).qlim=[-90/180*pi,155/180*pi];
L(3) = Link([ 0       0.210     0.155  -pi/2   0]); L(3).qlim=[-175/180*pi,255/180*pi];
L(4) = Link([ 0      -0.640   0       pi/2   0]); L(4).qlim=[-180/180*pi,180/180*pi];
L(5) = Link([ 0       0      0      -pi/2   0]); L(5).qlim=[-45/180*pi,225/180*pi];
L(6) = Link([ 0      -0.095  0      0      0]); L(6).qlim=[-360/180*pi,360/180*pi];
q0 =[0   0   0   0   0   0];
HP6 = SerialLink(L, 'name', 'Motoman HP6');
HP6.display();
HP6.teach(q0);

%% 拿板
T0=[1 0 0 1.116;
    0 1 0 -0.002;
    0 0 1 -0.715;
    0 0 0 1];
T11=[0 0 -1 -0.47;
    1 0 0 -0.3;
    0 1 0 -0.25;
    0 0 0 1];
T1=[0 0 -1 -0.48;
    1 0 0 -0.3;
    0 1 0 -0.25;
    0 0 0 1];
T2=[0 0 -1 -0.35;
    0 1 0 0.25;
    -1 0 0 0.055;
    0 0 0 1];
T21=[0 0 -1 -0.34;
    0 1 0 0.25;
    -1 0 0 0.055;
    0 0 0 1];
q1=HP6.ikine6s(T0);%根据起始点位姿，得到起始点关节角
q2=HP6.ikine6s(T11);%根据终止点位姿，得到终止点关节角
[q0 ,qd1, qdd1]=jtraj(q1,q2,50); %五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
HP6.plot(q0);%动画演示
TT2=HP6.fkine(q0);%根据插值，得到末端执行器位姿
JTA2=transl(TT2); % 利用Rbt.fkine函数求得笛卡尔空间中机器人每个轨迹控制位形所对应的末端执行器位置坐标，并将该位置坐标值赋给JTA；其中，Rbt为利用SerialLink函数建立的机器人模型函数名；fkine为进行正运动学分析的函数，Rbt.fkine函数所输出的是4×4的末端执行器位姿矩阵；transl函数从4×4位姿矩阵中提出位置矩阵。
plot2(JTA2,'b'),hold on; % 利用蓝色的点绘制所有轨迹。

t=50;
Tc=ctraj(T11,T1,t);
qq=HP6.ikine6s(Tc,'b');
tt=transl(Tc);
plot2(tt,'b');
grid on;
HP6.plot(qq);%动画演示

q3=HP6.ikine6s(T1);%根据起始点位姿，得到起始点关节角
q4=HP6.ikine6s(T2);%根据终止点位姿，得到终止点关节角
[q01 ,qd11, qdd11]=jtraj(q3,q4,50); %五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
HP6.plot(q01),hold on;%动画演示
T=HP6.fkine(q01);%根据插值，得到末端执行器位姿
JTA=transl(T); % 利用Rbt.fkine函数求得笛卡尔空间中机器人每个轨迹控制位形所对应的末端执行器位置坐标，并将该位置坐标值赋给JTA；其中，Rbt为利用SerialLink函数建立的机器人模型函数名；fkine为进行正运动学分析的函数，Rbt.fkine函数所输出的是4×4的末端执行器位姿矩阵；transl函数从4×4位姿矩阵中提出位置矩阵。
plot2(JTA,'b'); % 利用蓝色的点绘制所有轨迹。

Tc1=ctraj(T2,T21,t);
qq1=HP6.ikine6s(Tc1,'b');
tt1=transl(Tc1);
plot2(tt1,'r'),hold on;
grid on;
HP6.plot(qq1);%动画演示

q5=HP6.ikine6s(T21);%根据起始点位姿，得到起始点关节角
q6=HP6.ikine6s(T0);%根据终止点位姿，得到终止点关节角
[q011 ,qd111, qdd111]=jtraj(q5,q6,50); %五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
HP6.plot(q011),hold on;%动画演示
TT0=HP6.fkine(q011);%根据插值，得到末端执行器位姿
JTA1=transl(TT0); % 利用Rbt.fkine函数求得笛卡尔空间中机器人每个轨迹控制位形所对应的末端执行器位置坐标，并将该位置坐标值赋给JTA；其中，Rbt为利用SerialLink函数建立的机器人模型函数名；fkine为进行正运动学分析的函数，Rbt.fkine函数所输出的是4×4的末端执行器位姿矩阵；transl函数从4×4位姿矩阵中提出位置矩阵。
plot2(JTA1,'b'); % 利用蓝色的点绘制所有轨迹。



