%% initialize
clc; close all; clear all;
format shortg; format compact;

%% 工具箱函数使用
% L(1)= Link('d', 0, 'a', 0, 'alpha', pi/2,'qlim',[-pi pi]);
% L(2)= Link('d', 0, 'a', 10.15, 'alpha', 0,'qlim',[-pi/6 pi/6]);
% L(3)= Link('d', 0, 'a', 12.4, 'alpha', 0,'qlim',[-pi/3 pi/3]);
% L(4)= Link('d', 0, 'a', 13, 'alpha', 0,'qlim',[-pi/3 pi/3]);
% F4=SerialLink(L,'name','F4'); 
% T=[0 0 0 0];
% F4.plot(T);
% teach(F4);
% bot = SerialLink([L(1) L(2) L(3) L(4)], 'name', 'F4')
% % q=[pi/2 pi/2 0 0];
% % T=bot.fkine(q)
% % qz=[0 0 0 0];
% % M=[1 1 1 1 0 0];
% % qr=ikine(bot,T,qz,M)
% % t=[0:.5:5];         % generate a time vector0
% % q1=jtraj(qz,qr,t); % generate joint coordinate trajectory
% % bot.plot(q1)  %生成动态图形

%% 工具箱函数 robotics toolbox for matlab

% Q2=[ 0 0 0 pi/2 pi/2 pi/2];% Q3=[-pi/6 -pi/6 -pi/6 pi/3 0 pi];
% T0=[-1 0 0 0;0 0 -1 -200;0 -1 0 1080;0 0 0 1];%关节角为0，默认初始位置；
% T2=[1 0 0 -580;0 1 0 -100;0 0 1 600;0 0 0 1];%q2=[0 pi/2 -pi/2 pi/2 pi/2 pi/2];
% T2=[-1 0 0 -480;0 0 -1 -200;0 -1 0 600;0 0 0 1];%q2=[0 pi/2 -pi/2 0 0 0];
% Q0=ikine(R6,T0)
% Q1=ikine(R6,T2)%矩阵求逆，大概率求出空解；
% T0=[-1 0 0 0;0 0 -1 -200;0 -1 0 1080;0 0 0 1];
% T1=[0.5 -0.866 0 673;0 0 -1 -200;0.8660 0.5 0 665.7;0 0 0 1]
% q1=[0 -pi/6 -pi/6 -pi/3 0 0];
% q0=[0 0 0 0 0 0];
% T= fkine(R6,q0)%正解
% qi=ikine(R6,T) %反解

% qr=[0 0 0 pi/2 pi/2 -pi/2 ];
% %qz=[0 0 0 0 0 0];%给定各关节的角度
% fkine(R6,qr) %运动学正问题的求解
% t=0:0.1:1; 
% % q=jtraj(qz,qr,t);%产生N个角度数据；
% T=fkine(R6,qz)
%--------------------------------------------------------------------------
N=20;T=0:0.05:2; %N/T选其一
% q=ctraj(T0,T2,N);%位姿矩阵空间轨迹规划（笛卡尔坐标）
% q=jtraj(Q0,Q1,N);%关节角空间轨迹规划；
Q0=[0 0 0 0 0 0]; Q1=[0 pi/2 -pi/2 pi/2 pi/2 pi/2];
[q qd qdd]=jtraj(Q0,Q1,T); %角位移 角速度 角加速度，返回3组N*6数组；
% q=jtraj(Q0,Q1,N);
% plot(robot_UR5,q);%动画演示,(目前仅jtraj关节角)
% i=2;
% grid on;
% plot(T,q(:,i),'b-','linewidth',1);%仅T参数可用；
% hold on;
% plot(T,qd(:,i),'k--','linewidth',1);
% hold on;
% plot(T,qdd(:,i),'c:','linewidth',1.5);
% leg1=legend('q','qd','qdd','Location','Best');%图列说明,线型顺序依次；
% title('i轴'); 
% xlabel('t'), ylabel('\theta \omega \alpha');
% set(plot,'Location','Best');%'North' 'South' 'East' 'West' 'Best' 'BestOutside'   
% text(1.4,-0.7,'\leftarrow关节2');%设定指定位置的标签；
% gtext('鼠标指定'); 
%\alpha \beta \gamma \delta \epsilon \zeta \eta \theta \iota \kappa \lambda \mu \nu \xi \omicron \pi \rho \sigma \tau \upsilon \phi \chi \psi \omega
%蓝色'b' 绿色'g'，红色'r' 蓝绿'c' 紫红'm' 黄色'y' 黑色'k' 
%实线'-' 虚线'--' 点线':' '.' 点划线'-.' '+-*o<>v^'
% 'square'='s' 'pentagram'='p' 'diamond' 'hexagram' 
% axis([xmin,xmax,ymin,ymax]);%调整图轴的范围


%% MATLAB算法基础RTB
% Robotics,Vision and Control;
% 机器人学、机器视觉与控制——MATLAB算法基础
% theta=pi/6;
% T1=SE2(1,2,pi/6);  %齐次变换函数，xy移动，z旋转；
% T01=[1 0 1;0 1 2; 0 0 1];%inv(T01)  %invtrans()
% T02=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
% %标准正交阵；% inv(T02)==%invrot()==(T02)'
% T01*T02;%==T1
% T2=SE2(2,1,0);
% axis([0 5 0 5]);
% grid on;
% T3=T1*T2;
% T4=T2*T1;
% trplot2(T1,'frame','1','color','b');  %静态图像
% hold on
% trplot2(T2,'frame','2','color','r');
% hold on
% trplot2(T3,'frame','3','color','g');
% hold on
% trplot2(T4,'frame','4','color','c');
% hold on
% p=[3;2];
% plot_point(p,'*');
% inv(T1);%==inv(T01*T02)==inv(T02)*inv(T01);
% % p1=inv(T1)*[p;1];  % 齐次，三维
% h2e([p;1]);%==homtrans函数,三维转二维
% e2h(p);


%% 提纲

% %机械结构-运动学
% 虚拟设计，静态动态进行受力分析，有限元优化机械机构；
% %动力学仿真
% 运用simulink建立运动学仿真模型，实时控制末端变化；
% matlab Adams 3dmax联合仿真；
% %轨迹规划
% 关节轨迹三次多项式插值，空间任意直线圆弧轨迹拟合插值计算；
% 受速度，加速度，力矩约束时进行时间最优解的轨迹算法；
% B样条曲线进行规划，拟合无碰撞离散路径；
% 3-5-3样条函数，保证规划准确度和精度；
% 连续曲线路径再笛卡尔空间规划，曲线拐角处采用五次多项式过度法进行规划，使曲线路径连续平滑；
% 关节空间和笛卡尔空间两个方面提出轨迹规划算法；计算出机器人的位移速度加速度；

% 轨迹规划
% 1，平滑一维轨迹：五次多项式，改进五次多项式
% 2，多维轨迹，分段轨迹

% 运动学
% 1，正运动学
% 2，逆运动学
% 编写算法，求出多解（避开奇异点）
% 关节插补，笛卡尔直线插补
% 
% 动力学
% 1，关节速度，雅可比矩阵（奇异性）
% 2，力和力矩传递
% 3，运动方程，惯量矩阵，控制simulink仿真
% ----------------------------------------------------
% 速度雅可比矩阵
% [dx dy]'=J*[dθ1 dθ2]'
% J=[(?X/?θ1)*dθ1 (?X/?θ1)*dθ2;(?Y/?θ1)*dθ1 (?Y/?θ1)*dθ2];
% v=dx/dt=J*dθ=J*ω;
% ω=inv(J)*v;
% 奇异性，inv(J)=(J^*)/det(J);当det(J)==0时，inv(J)=∞；速度无穷大；
% 
% 静力学，力矩雅可比矩阵  J'
% τ=J'*F
% τ=[τ1 τ2 τ3]',F=[Fx Fy Fz]';

% 动力学正问题——已知关节的驱动力矩，求机器人系统相应的运动参数（包括关节位移、速度和加速度）。
% 动力学逆问题——已知运动轨迹点上的关节位移、速度和加速度，求出所需要的关节力矩。
% 机器人是由多个连杆和多个关节组成的复杂的动力学系统，具有多个输入和多个输出，存在着错综复杂的耦合关系和严重的非线性。
% 常用的方法：
% 拉格朗日(Lagrange)方法
% 牛顿—欧拉方法(Newton-Euler)方法,% 高斯(Gauss)方法,% 凯恩(Kane)方法
% 拉格朗日函数L：一个机械系统的动能Ek和势能Eq之差；Ek=f(ω);Eq=f(θ);
% L＝Ek-Eq; 
% 拉格朗日方程，力矩M
% M[]=d(?L/?ω)/dt-?L/?θ;
% -----------------------------------
% R^.(t)=S(ω)*R(t)
S=skew([1 2 3])
omega=vex(S)











