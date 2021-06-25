%% initialize
clc; close all; clear all;
format shortg; format compact;

%% DH方法
% DH分为标准DH和改进DH，建模的平移旋转变换顺序有少许不同，一次关节变换包含4个最基础的变换矩阵，且为两个不同的关节的变换参数
% 本文使用变换矩阵 T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);为自定义矩阵，4个参数全为前关节的变换参数

%  'standard'表示标准DH，参数设在i和i+1之间，'modified'改进DH，参数设在i-1和i之间；
% L = Link([THETAi Di Ai-1 ALPHAi-1 SIGMA],'modified');
% % theta：绕Zi轴，从Xi-1旋转到Xi的角度
% % D：沿Zi轴，从Xi-1移动到Xi的距离
% % A：沿Xi轴，从Zi移动到Zi+1的距离
% % alpha：绕Xi轴，从Zi旋转到Zi+1的角度

% L = Link([THETA D A ALPHA SIGMA],'standard');
% % theta：绕Zi轴，从Xi旋转到Xi+1的角度
% % D：沿Zi轴，从Xi移动到Xi+1的距离
% % A：沿Xi轴，从Zi移动到Zi+1的距离
% % alpha：绕Xi+1轴，从Zi旋转到Zi+1的角度
% Li = Link([关节角度 连杆偏置 连杆长度 连杆扭转角 sigma旋转0移动1 关节偏移量],'standard');
% Li = Link([theta(1) d(1) a(1) alpha(1) 0 offset],'standard');


%% 标准UR机器人
    a=[0 -425.00 -392.25 0 0 0];
    d=[89.159 0 0 109.15 94.65 82.30];
    alpha=[pi/2 0 0 pi/2 -pi/2 0];
    theta=[0 0 0 0 0 0];
L1 = Link([theta(1) d(1) a(1) alpha(1) 0 0]); 
L2 = Link([theta(2) d(2) a(2) alpha(2) 0 0]);%-pi/2]);
L3 = Link([theta(3) d(3) a(3) alpha(3) 0 0]);
L4 = Link([theta(4) d(4) a(4) alpha(4) 0 0]);%-pi/2]);
L5 = Link([theta(5) d(5) a(5) alpha(5) 0 0]);
L6 = Link([theta(6) d(6) a(6) alpha(6) 0 0]);

% L1 = Link('d', 0.089159, 'a', 0,        'alpha', pi/2 ,'standard' );
% L2 = Link('d', 0,        'a', -0.42500, 'alpha',   0  ,'offset', -pi/2,'standard' );
% L3 = Link('d', 0,        'a', -0.39225, 'alpha',   0  ,'standard' );
% L4 = Link('d', 0.10915,  'a', 0,        'alpha', pi/2 ,'offset', -pi/2,'standard' );
% L5 = Link('d', 0.09465,  'a', 0,        'alpha',-pi/2 ,'standard');
% L6 = Link('d', 0.08230,  'a', 0,        'alpha',   0  ,'standard');
L=([L1 L2 L3 L4 L5 L6]);

R6=SerialLink(L, 'name', 'UR5');
q0=[0 0 0 0 0 0];%初始值
% R6.plot(T) ;%3d Figure
R6.display();
R6.teach(q0);%Teach panel,(x,y,z,R,P,Y)


%% UR5
theta=[0 0 0 0 0 0]; %两公垂线之间的夹角；
% theta=[0 -pi/2 0 -pi/2 0 0]; %初始偏移版本
d=[90 -100 100 90 90 90]; %两公垂线之间的距离；
a=[0 -420 -400 0 0 0]; %两关节轴线之间公垂线长度；
alpha=[pi/2 0 0 pi/2 -pi/2 0]; %两关节轴线之间夹角；
offset=[0 0 0 0 0 0]; %[0 -pi/2 0 -pi/2 0 0]; %初始角度theta的偏转量；
sigma=0; mdh=0;%旋转关节；向后建模，默认T1为T01；

L1 = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
L2 = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
L3 = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
L4 = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
L5 = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
L6 = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);
robot_UR5=SerialLink([L1,L2,L3,L4,L5,L6],'name','UR5'); 
robot_UR5.display();
figure('NumberTitle', 'off', 'Name', 'UR5');
robot_UR5.teach(theta); 
% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|         90|          0|     1.5708|          0|
% |  2|         q2|          0|       -420|          0|          0|
% |  3|         q3|          0|       -400|          0|          0|
% |  4|         q4|         90|          0|     1.5708|          0|
% |  5|         q5|         90|          0|    -1.5708|          0|
% |  6|         q6|         90|          0|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+


%% AK5
% AK5平行轴坐标系建模
% L(1)=Link([0 100 0 pi/2 0 pi]);%使L2绕L1x转90，L2沿L1z平移100；
% L(2)=Link([0 120 480 0 0 pi/2]);%使L3沿L2z平移120，沿L2x平移480，之后L3绕L2z转90；
% L(3)=Link([0 -120 400 0 0 0]);%使L4沿L3z平移-120，沿L3x平移400；
% L(4)=Link([0 100 0 pi/2 0 pi/2]);%使L5绕L4x转90，绕L4z转90，沿L4z平移100；
% L(5)=Link([0 100 0 -pi/2 0 0]);%使L6绕L5x转-90，沿L5z平移100；
% L(6)=Link([0 100 0 0 0 0]);%使末端坐标系TO，沿L6z平移100；

% syms th1 th2 th3 th4 th5 th6;
% % th1=0; th2=0; th3=0; th4=0;th5=0;th6=0;
% syms d2 d4 d6 l2 l3 l4 l5;% syms a b c d e f;
% d2=120;d4=-20;d6=100;l2=100;l3=480;l4=400;l5=100;
% T1=[cos(th1) -sin(th1) 0 0;sin(th1) cos(th1) 0 0;0 0 1 0;0 0 0 1];
% T2=[cos(th2) 0 sin(th2) 0;0 1 0 d2;-sin(th2) 0 cos(th2) l2;0 0 0 1];
% T3=[cos(th3) 0 sin(th3) 0;0 1 0 0;-sin(th3) 0 cos(th3) l3;0 0 0 1];
% T4=[cos(th4) 0 sin(th4) 0;0 1 0 d4;-sin(th4) 0 cos(th4) l4;0 0 0 1];
% T5=[cos(th5) -sin(th5) 0 0;sin(th5) cos(th5) 0 0;0 0 1 l5;0 0 0 1];
% T6=[cos(th6) 0 sin(th6) 0;0 1 0 d6;-sin(th6) 0 cos(th6) 0;0 0 0 1];
% T0=T1*T2*T3*T4*T5*T6;
% T0=[1 0 0 0;0 1 0 200;0 0 1 1080;0 0 0 1];
% [a b c d e f]=solve(T==T0,th1,th2,th3,th4,th5,th6)
% [e f]=solve(T==T0,th5,th6)
%% AK5恒绕Z坐标系建模
% RotX=[1 0 0 0;0 cos(theta) -sin(theta) 0;0 sin(theta) cos(theta) 0;0 0 0 1];
% RotY=[cos(theta) 0 sin(theta) 0;0 1 0 0;-sin(theta) 0 cos(theta) 0;0 0 0 1];
% RotZ=[cos(theta) -sin(theta) 0 0;sin(theta) cos(theta) 0 0;0 0 1 0;0 0 0 1];
% Trans(a b c)=[1 0 0 a;0 1 0 b;0 0 1 c;0 0 0 1];%Trans函数的R阵为单位阵，右乘顺序无关；
% l1=100;l2=120;l3=480;l4=-120;l5=400;l6=100;l7=100;l8=100;
% 每个矩阵T代表该坐标系，由于Link建模使用了Standard指定下一个坐标系，此处共7个T；
% 检验矩阵坐标方向，第123竖列分别代表新的xyz，所在第123横排代表对应上一个坐标系的xyz方向；
% syms th1 th2 th3 th4 th5 th6;
% % th1=0;th2=0;th3=0; th4=0;th5=0;th6=0;
% T1=[cos(th1) -sin(th1) 0 0;sin(th1) cos(th1) 0 0;0 0 1 0;0 0 0 1];%T=RotZ;
% T2=[cos(th2) -sin(th2) 0 0;0 0 -1 0;sin(th2) cos(th2) 0 100; 0 0 0 1];%RotX*T+Trans;
% T3=[-sin(th3) -cos(th3) 0 0;cos(th3) -sin(th3) 0 480;0 0 1 120;0 0 0 1];%RotZ,offset修改了初始角度；
% T4=[cos(th4) -sin(th4) 0 400; sin(th4) cos(th4) 0 0;0 0 1 -120;0 0 0 1];%Trans;
% T5=[0 0 1 0;cos(th5) -sin(th5) 0 0;sin(th5) cos(th5) 0 100;0 0 0 1];%RotZ*RotX*Tz,顺序相关；
% T6=[cos(th6) -sin(th6) 0 0;0 0 1 100;-sin(th6) -cos(th6) 0 100;0 0 0 1];%RotX,ZY轴移动合并；
% T=T1*T2*T3*T4*T5*T6;

%% HP6
% mdl_motomanHP6;
%            theta      d      a      alpha
% L(1) = Link([ 0       0      0.15   -pi/2   0]);
% L(2) = Link([ 0       0      0.57    pi     0]);
% L(3) = Link([ 0       0      0.155  -pi/2   0]);
% L(4) = Link([ 0      -0.635  0       pi/2   0]);
% L(5) = Link([ 0       0      0      -pi/2   0]);
% L(6) = Link([ 0      -0.095  0       pi     0]);
% q0 =[0   -pi/2   0   0   -pi/2   0];
% HP6 = SerialLink(L, 'name', 'Motoman HP6');
% HP6.display();
% HP6.teach(q0); 
%{
% %            theta    d      a      alpha sigma offset
% L(1) = Link([ 0      100     0       0      0     0]);
% L(2) = Link([ 0       0     100      pi/2   0     0]);
% L(3) = Link([ 0       0     500      0      0     0]);
% L(4) = Link([ 0      600    100      pi/2   0     0]);
% L(5) = Link([ 0       0      0      -pi/2   0     0]);
% L(6) = Link([ 0       0      0       pi/2   0     0]);
% q0 =[0   pi/2   0   0   0   0];
% J = SerialLink(L, 'name', 'jungle');
% J.display();
% J.teach(q0); 
%}

%% SAR
% L1 = Link('d', 0, 'a', 0, 'alpha', pi/2);   
% L2 = Link('d', 0, 'a', 0.5, 'alpha', 0,'offset',pi/2);
% L3 = Link('d', 0, 'a', 0, 'alpha', pi/2,'offset',pi/4);
% L4 = Link('d', 1, 'a', 0, 'alpha', -pi/2);
% L5 = Link('d', 0, 'a', 0, 'alpha', pi/2);
% L6 = Link('d', 1, 'a', 0, 'alpha', 0);
% b=isrevolute(L1);  %Link 类函数
% robot=SerialLink([L1,L2,L3,L4,L5,L6]);   %SerialLink 类函数
% robot.name='SAR';     %SerialLink 属性值
% obot.manuf='plgk';     %SerialLink 属性值
% robot.display();  %Link 类函数
% theta=[0 0 0 0 0 0];
% robot.plot(theta);   %SerialLink 类函数
% theta1=[pi/4,-pi/3,pi/6,pi/4,-pi/3,pi/6];
% p0=robot.fkine(theta);
% p1=robot.fkine(theta1);
% s=robot.A([4 5 6],theta);
% cchain=robot.trchain;
% q=robot.getpos();



%% zwh
%  L1 = Link('d', 0, 'a', 0, 'alpha',-pi/2, 'offset', pi/2,'qlim',[0 pi/2]);%定义连杆
%  L2 = Link('theta', 0, 'a', 0, 'alpha',0,'qlim',[20 30]);
%  L3 = Link('d', 0, 'a', 20, 'alpha', pi/2);
%  L4 = Link('d',0, 'a', 0, 'alpha', pi/2,'qlim',[0 pi]);
%  L5 = Link('theta', 0, 'a', 0, 'alpha', 0,'qlim',[0 20]);
%  L6 = Link('d', 20, 'a', 0, 'alpha', 0);
% 
%  robot = SerialLink([L1 L2 L3 L4 L5 L6],'base',troty(pi/2));%连接连杆
%  robot.display();%显示D-H参数表
%  robot.name = 'robot';
%  theta = [0,0,0,0,0,0];
%  robot.plot(theta,'workspace',[-50,50,-50,50,-50,50]); %显示机器人的图像
%  robot.teach;
% para=[0 20 0 0 0 0];
% T=forward_kine(para)

%% 蜗牛
% L1=Link([0 0.085 0 0 0],'standard');
% L2=Link([0 0 0 -pi/2 1],'standard');
% L3=Link([0 0 -0.5 pi/2 0],'standard');
% L2.qlim=[0.2 0.6];
% bot=SerialLink([L1 L2 L3],'name','单腿仿真');
% bot.display();%显示D-H参数表
% bot.teach(); 
% qA=[0 0 0];
% plot(bot,qA);

% t=[0:0.1:3];
% qA=[0 0.2 0];
% qB=[pi/6 0.6 pi/6];
% q=jtraj(qA,qB,t);%生成关节运动轨迹
% T=fkine(bot,q)%正向运动学仿真函数 
% plot(bot,q);%生成机器人的运动
% figure('Name','单腿末端位移图')
% subplot(3,1,1); 
% T1=squeeze(T(1,4,:))'
% plot(t, T1); 
% xlabel('Time (s)'); 
% ylabel('X (m)'); 
% subplot(3,1,2); 
% plot(t, squeeze(T(2,4,:))'); 
% xlabel('Time (s)');
% ylabel('Y (m)'); 
% subplot(3,1,3); 
% plot(t, squeeze(T(3,4,:))'); 
% xlabel('Time (s)'); 
% ylabel('Z (m)');

% plot(bot,q);
%  x=squeeze(T(1,4,:));
%  y=squeeze(T(2,4,:));
%  z=squeeze(T(3,4,:)); 
% figure('Name','up6机器人末端轨迹图');

% yang, math mdeling match;
% sin(theta)=12.5/400;
% m=12.5;n=400;
% % c=sqrt(n^2-m^2);
% theta=asin(m/n);
% 
% x=0.685*tan(theta);
% y=1.68-x;
% a=0.685/cos(theta);
% b=y*sin(theta);
% d=2*(a+b)

% theta=atan2()
% syms a x;
% [a,x]=solve(a*(1.01-a)-x*(1.68-x),a^2-x^2-0.685^2,a,x)
% a=vpa(a,8)
% x=vpa(x,8)
% R=12.5*a./x

%% word

% 
% 要建立机器人对象，首先我们要设定机器人的D-H参数，之后我们可以利用Robotics Toolbox工具箱中的link和robot函数来建立
% 
% 其中link函数的调用格式：
% L = LINK([ alpha A thetaD])
% L =LINK ([ alpha A theta D sigma ])
% L =LINK([ alpha A theta D sigma offset ])
% L =LINK([ alpha A theta D ], CONVENTION  ])
% L =LINK([alpha A theta D sigma],CONVENTION  ])
% L =LINK([ alpha A theta D sigma offset ],CONVENTION )

% 
% 参数CONVENTION可以取‘standard’和‘modified’，其中‘standard’代表采用标准的D-H参数，‘modified’代表采用改进的D-H参数。参数‘alpha’代表扭转角 ，参数‘A’代表杆件长度，参数‘theta’代表关节角，参数‘D’代表横距，参数‘sigma’代表关节类型：0代表旋转关节，非0代表移动关节。另外LINK还有一些数据域：
% 	LINK.alpha	    %返回扭转角
% 	LINK.A        %返回杆件长度
% 	LINK.theta       %返回关节角
% 	LINK.D        %返回横距
% 	LINK.sigma     %返回关节类型
% 	LINK.RP	      %返回‘R’(旋转)或‘P’(移动)
% 	LINK.mdh      %若为标准D-H参数返回0，否则返回1
% 	LINK.offset	    %返回关节变量偏移
% 	LINK.qlim	     %返回关节变量的上下限 (min max)
% 	LINK.islimit(q)	%如果关节变量超限，返回 -1, 0, +1
% 	LINK.I		%返回一个3×3 对称惯性矩阵
% 	LINK.m		%返回关节质量
% 	LINK.r		%返回3×1的关节齿轮向量
%     LINK.G	%返回齿轮的传动比
% 	LINK.Jm	%返回电机惯性
% 	LINK.B		%返回粘性摩擦
% 	LINK.Tc	%返回库仑摩擦
% 	LINK.dh		return legacy DH row
% 	LINK.dyn	    return legacy DYN row
% % 其中robot函数的调用格式：
% 	ROBOT			      %创建一个空的机器人对象
% 	ROBOT(robot)		   %创建robot的一个副本
% 	ROBOT(robot, LINK)	%用LINK来创建新机器人对象来代替robot
% 	ROBOT(LINK, ...)	    %用LINK来创建一个机器人对象
% 	ROBOT(DH, ...)		    %用D-H矩阵来创建一个机器人对象
% 	ROBOT(DYN, ...)		%用DYN矩阵来创建一个机器人对象
% 2．变换矩阵
% 利用MATLAB中Robotics Toolbox工具箱中的transl(x,y,z)、rotx(α)、roty(β)和rotz(θ)可以实现用齐次变换矩阵表示平移变换和旋转变换。
% 3 轨迹规划
% 利用Robotics Toolbox提供的ctraj、jtraj和trinterp函数可以实现笛卡尔规划、关节空间规划和变换插值。
% 其中ctraj函数的调用格式：
% 	TC = CTRAJ(T0, T1, N)
% 	TC = CTRAJ(T0, T1, R)
% 参数TC为从T0到T1的笛卡尔规划轨迹，N为点的数量，R为给定路径距离向量，R的每个值必须在0到1之间。
% 其中jtraj函数的调用格式：
%  
% [Q QD QDD] = JTRAJ(Q0, Q1,N)
% [Q QD QDD] = JTRAJ(Q0, Q1, N, QD0, QD1)
% [Q QD QDD] = JTRAJ(Q0, Q1,T)
% [Q QD QDD] = JTRAJ(Q0, Q1, T, QD0, QD1)


% 参数Q为从状态Q0到Q1的关节空间规划轨迹，N为规划的点数，T为给定的时间向量的长度，速度非零边界可以用QD0和QD1来指定。QD和QDD为返回的规划轨迹的速度和加速度。
% 其中trinterp函数的调用格式：
% TR = TRINTERP(T0, T1, R)
% 参数TR为在T0和T1之间的坐标变化插值，R需在0和1之间。
% 要实现轨迹规划，首先我们要创建一个时间向量，假设在两秒内完成某个动作，采样间隔是56ms，那么可以用如下的命令来实现多项式轨迹规划：t=0:0.056:2; 
%  
% 其中t为时间向量，qz为机器人的初始位姿，qr为机器人的最终位姿，q为经过的路径点，qd为运动的速度，qdd为运动的加速度。其中q、qd、qdd都是六列的矩阵，每列代表每个关节的位置、速度和加速度。如q(:,3)代表关节3的位置，qd(:,3)代表关节3的速度，qdd(:,3)代表关节3的加速度。
% 4 运动学的正问题
% 利用Robotics Toolbox中的fkine函数可以实现机器人运动学正问题的求解。
% 其中fkine函数的调用格式：
% TR = FKINE(ROBOT, Q)
% 参数ROBOT为一个机器人对象，TR为由Q定义的每个前向运动学的正解。
% 以PUMA560为例，定义关节坐标系的零点qz=(0 0 0 0 0 0)，那么fkine(p560,qz)将返回最后一个关节的平移的齐次变换矩阵。如果有了关节的轨迹规划之后，我们也可以用fkine来进行运动学的正解。比如：
% t=0:0.056:2; q=jtraj(qz,qr,t); T=fkine(p560,q);
% 返回的矩阵T是一个三维的矩阵，前两维是4×4的矩阵代表坐标变化，第三维是时间。
% 
% 5 运动学的逆问题
% 利用Robotics Toolbox中的ikine函数可以实现机器人运动学逆问题的求解。
% 其中ikine函数的调用格式：
% 	Q = IKINE(ROBOT, T)
% 	Q = IKINE(ROBOT, T, Q)
% 	Q = IKINE(ROBOT, T, Q, M)
% 参数ROBOT为一个机器人对象，Q为初始猜测点（默认为0），T为要反解的变换矩阵。当反解的机器人对象的自由度少于6时，要用M进行忽略某个关节自由度。
% 有了关节的轨迹规划之后，我们也可以用ikine函数来进行运动学逆问题的求解。比如：
% t=0:0.056:2; T1=transl(0.6,-0.5,0); T2=transl(0.4,0.5,0.2); T=ctraj(T1,T2,length(t)); q=ikine(p560,T); 
% 
% T=fkine(p560,q); qi=ikine(p560,T);
% 6 动画演示
% 有了机器人的轨迹规划之后，我们就可以利用Robotics Toolbox中的plot函数来实现对规划路径的仿真。
% puma560;T=0:0.056:2; q=jtraj(qz,qr,T); plot(p560,q);
% 当然，我们也可以来调节PUMA560的六个旋转角，来实现动画演示。
% drivebot(p560)
% 标准



