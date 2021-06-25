clc;clear all;close all;
format short g;format compact;
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
学习机器人学的部分代码，主要参考书为《机器人学、机器视觉与控制》
网址https://petercorke.com/
G站https://github.com/petercorke

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

%% 位姿转换
% 旋转矩阵
% 轴角法/旋转矢量
% RPY
% 欧拉角
% 四元数

% R=rotx(pi/2);
% trplot(R);  
% tranimate(R);  %动态图像
% S=R*roty(pi/2); %绕新y轴旋转
% S=rotx(pi/2)*roty(pi/2);
% trplot(S);
% tranimate(S); 
% R=rotz(0.1)*roty(0.2)*rotz(0.3) %ZYZ序列欧拉角
% EU=eul2r(0.1,0.2,0.3)%正运算
% tr2eul(EU) %逆运算
rpy2tr(pi,-pi,pi/2);  %面板RPY→旋转矩阵 函数
% rpy2tr(0,0,0);
% R=rpy2r(0.1,0.2,0.3)% 横滚俯仰偏航角，莱特布莱恩角，卡尔丹角
% tr2rpy(R) %逆运算
% tr2rpy(T)*180/pi
% tr2rpy(t2r(T))*180/pi
% tr2eul(T)*180/pi
% quaternion(T)
% tr2rotvec(T)
% tr2rpy(T(1:3,1:3))

%q=Quaternion() %四元数，q=s<v1,v2,v3>,q=s+v=s+v1*i+v2*j+v3*k,i^2=j^2=k^2=i*j*k=-1;
% T=rpy2tr(0.1,0.2,0.3)
% q=Quaternion(T)
%位姿描述：Rxyz,Euler,RPY,oavec,angvec,T,R,Q

% 笛卡尔坐标系，方向矢量noa；默认T1为T01，以此类推，标准DH；
% 欧拉角Euler(phi,theta,psi)=rot(z,phi)*rot(y,theta)*rot(z,psi)
% 滚仰偏RPY(phi,theta,psi)=rot(z,phi)*rot(y,theta)*rot(x,psi);%左乘；
% 标准DH，只修改 d a alpha；

% function rotvec(a b c)
% 旋转矢量 k(a b c)=(ka kb kc)=(rx ry rz)
% k=sqrt(rx^2+ry^2+rz^2);% k(rx/k ry/k rz/k)
% theta=0时病态；
% 
% [theta,v]=tr2angvec(R);
% R=angvec2tr(0,[1 0 0])
% [theta,v]=tr2angvec(R)
% rxyz=theta*v
% t2r(R)
% R=angvec2r(pi/2,[1 0 0])
% r2t(R)  

% R=rpy2tr(1,2,3)
% [theta,v]=tr2angvec(R)
% [theta,v]=tr2angvec(R) %only tr2,no r2;
% T=angvec2tr(pi/4,[0 1/2^0.5 1/2^0.5])
% T=angvec2tr(2*pi*rand(1),[rand(1) rand(1) rand(1)]);
% T(1,4)=500;T(2,4)=400;T(3,4)=400;
% T=rpy2tr(10*pi/180,0,0)

% % P(1:3)=theta*v
% tr2angvec(R)
% q=quaternion(T)
% quaternion([1 2 3])
% angvec2tr(theta,v)
% theta*180/pi
% tr2rotvec(R) error

%% puma560_tutorials
% C:\Users\wangkingsheng\Documents\MATLAB\Add-Ons\Toolboxes\Robotics Toolbox for MATLAB(2)\code\robot\models
% mdl_puma560;
% p560  %输出参数
% % p561 = SerialLink(p560, 'base', transl(-0.5, 0.5, 0) )
% T=[0 0 0 0 0 0];
% p560.plot(T);
% teach(p560);
% mdl_fanuc10L;
% mdl_irb140;
% mdl_motomanHP6;
% % motomanHP6
% T=[0 0 0 0 0 0];
% motomanHP6.plot(T);
% teach(motomanHP6);
% mdl_nao;nao.plot();

% T=0:0.056:2;
% q=jtraj(qz,qr,T);
% plot(p560,q);
% qz：（0,0,0,0,0,0） 零角度 
% qr：（0，pi/2,-pi/2,0,0,0） 就绪状态，机械臂伸直切垂直 
% qs：（0,0，-pi/2，0,0,0） 伸展状态，机械臂伸直且水平 
% qn：（0，pi/4,-pi,0,pi/4,0） 标准状态，机械臂处于一个灵巧工作状态。

% q=[0 pi/6 -2*pi/3 0 0 0];
% T= fkine(p560,q);%正解
% qi=ikine(p560,T) %反解
% T0 = transl([0 0 0]); T1 = transl([1 1 1]);
% t= [0:0.1:2];
% r = jtraj(0, 1, t)
% TC = ctraj(T0, T1, r)
% plot(t, transl(TC))



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


%% vision
% 10 光与色彩
%{
lambda=[300:10:1000]*1e-9; %波长的定义范围；
for T=1000:1000:6000 %可见光310nm-750nm
%     plot(lambda*1e9,blackbody(lambda,T));hold all; %黑体光谱函数；
end
% 光谱的表示
lamp=blackbody(lambda,2600);
sun=blackbody(lambda,6500);
% plot(lambda*1e9,[lamp/max(lamp),sun/max(sun)]);
% 地球大气层的吸收-物体表面的吸收；
% 色彩，眼睛的两种视觉细胞，视锥形细胞对特别的颜色有反应，视感细胞对强光又反应；
human=luminos(lambda);
% plot(lambda*1e9,human);
% 显示器每个像素产生可变三原色，标准CIE三原色
% 色度空间
% 三色刺激的值不仅描述了颜色，还包含亮度
% C=RR+GG+BB;
lambda=[400:10:700]*1e-9;
cmf=cmfrgb(lambda);
% plot(lambda*1e9,cmf);
orange=cmfrgb(600e-9); %创造600nm的橙光，需要的原色
[r,g]=lambda2rg([400:700]*1e9);
% plot(r,g);
rg_addticks;
% XYZ 虚构的非物理三色系统；XZ具有0亮度，亮度完全由Y提供，
cmf=cmfxyz(lambda);
% plot(lambda*1e9,cmf);%光谱轨迹绘制
% xycolorspace
lambda2xy(550e-9);
colorname('blue');colorname('blue','xy');
colorspace('RGB->HSV',[1 0 0]);% 实现不同颜色空间之间的转换，RGB->色相饱和度强度；
% 白平衡，调整光源色温，rgb1=J*rgb
% -------------------------------
flowers=iread('flowers4.png','double','gamma','sRGB'); %640*426
hsv=colorspace('RGB->HSV',flowers);
% idisp(hsv(:,:,1)); %色相
% idisp(hsv(:,:,2)); %饱和度
XYZ=colorspace('RGB->XYZ',flowers);
[x,y]=tristim2cc(XYZ);
xbins=[0 0.01 100];
ybins=[0 0.01 100];
% [h vx vy]=hist2d(x,y,xbins,ybins); %图
% xycolorspace; %bug
% hold on;
% contour(vx,vy,h);
% [cls,cxy]=colorkmeans(flowers,7);
%}
% 11图像形成
% 小孔成像，相机成像过程是把三维世界投影在二维平面上，失去了深度信息，称为透视投影
% cam=CentralCamera('focal',0.015) %中央投影照相机模型，15mm透镜
% P=[0.3 0.4 3]';
% cam.project(P)
% 
% 
%% 
% im=imread('IMG_20170619_121931.jpg'); 
% figure,imshow(im);
% title('源图像');
%  
% %选取图像上的一个正方形的ROI区域；  
% im0 = imcrop(im,[1198 54 2210 2210]);  
% figure,
% imshow(im0,'DisplayRange',[])  
% title('选取ROI后的图像');
 
%%%%%高斯滤波%%%%%
sigma = 1.6;
gausFilter = fspecial('gaussian',[5 5],sigma);
blur=imfilter(im0,gausFilter,'replicate');
figure,imshow(blur);
title('高斯滤波后的图像');
 
level = graythresh(blur);   %%%ostu算法求阈值进行二值化
im1 = im2bw(blur,level);
figure,imshow(im1);
title('二值图像');
 
bw1=bwlabel(im1,8);
stats=regionprops(bw1,'Area');
bw2 = ismember(bw1, find([stats.Area]) == max([stats.Area])); %%找到面积最大的对象
figure,imshow(bw2);
title('二值图像');
[B L] = bwboundaries(bw2);  %寻找边缘，不包括孔,L是标记矩阵
figure,   %%%创建空白的图像
hold on
for k = 1:length(B)
    boundary = B{k};  %B是一个胞元数组，所以是B{k}
    plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2);%% 画出边缘
end%整个循环表示的是描边
 
 
%% 
%%%%%寻找hough圆的圆心%%%%%%%
Rmin = 100;
Rmax = 200;
[centersBright, radiiBright] = imfindcircles(im1,[Rmin Rmax],'ObjectPolarity' ,'bright');
viscircles(centersBright,radiiBright,'EdgeColor','b');
hold on 
plot(centersBright(1),centersBright(2),'*');
hold off;
 
%% 等分圆
R=1050; t=0:pi/20:2*pi;
x=R*cos(t);
y=R*sin(t);
axis equal
n=36;a=2*pi/n;
for k=0:n-1
    hold on         %%%%%其中(1077,1055)是半径，1090是半径，即直线的长度
    plot([1077-1090*cos(pi+k*a),1077+1090*cos(pi+k*a)],[1114-1090*sin(pi+k*a),1114+1090*sin(pi+k*a)])
end

