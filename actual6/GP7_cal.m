%% initialize
clc; close all; clear all;
format shortg; format compact;
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
这是另外一个经典六轴串联模型的求解过程；鼎鼎大名的安川GP系列；YASKAWA MOTOMAN GP7；
GP7与UR5是两种常见的模型结构，各有特点；个人更喜欢UR的外形，是个颜粉；
在perc实验室两种模型都是实体机器人，当时在选择的时候纠结了一下还是选择了UR5；
一是网上的资料更多，且开源，二是坐标系比较分散，整体布局更加合理；
cal是计算推导，sim是有用代码提取；
这些计算也是早期的方法，后来深入研究之后进行了更全面精准的理论参数推导，
但是时间有限，就没有把方法应用到此程序

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

%% model
theta=[0 0 0 0 0 0];
% d1=100;d4=-440;d6=-80;  a1=40;a2=460;a3=40;
% d=[d1 0 0 d4 0 d6];  %d(1)=330;   
% a=[a1 a2 a3 0 0 0];  %a(2)=445;
% alpha=[-pi/2 pi -pi/2 pi/2 -pi/2 pi];
% offset=[0 0 0 0 0 0]; sigma=0; mdh=0;
% L1 = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
% L2 = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
% L3 = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
% L4 = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
% L5 = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
% L6 = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);
% 全新精简建模，GP7；
% d1=100;d4=400;d6=80;  a2=-500;
% d=[d1 0 0 d4 0 d6]; a=[0 a2 0 0 0 0];
alpha=[pi/2 0 pi/2 -pi/2 pi/2 0]; 
%       theta  d   a  alpha  sigma offset
L1 = Link([0  100  0   pi/2    0    0]); %mdh默认0
L2 = Link([0   0 -500    0     0    0]);
L3 = Link([0   0   0   pi/2    0    0]);
L4 = Link([0  400  0  -pi/2    0    0]);
L5 = Link([0   0   0   pi/2    0    0]);
L6 = Link([0   80  0     0     0    0]);
q1=[0 -pi/2 0 0 -pi/2 0];
q0=[0 0 0 0 0 0];
% robot_GP7=SerialLink([L1,L2,L3,L4,L5,L6],'name','GP7'); 
% robot_GP7.display();
% robot_GP7.teach(q0);

syms th1 th2 th3 th4 th5 th6;
theta=[th1 th2 th3 th4 th5 th6];
% 建立等式 T=T1*T2*T3*T4*T5*T6;
% forward_16(theta)
% T25=T2*T3*T4*T5==T1i*Tq*T6i
forward_16(theta); %修改函数，求 T25
% [ cos(th2 + th3)*cos(th4)*cos(th5) - sin(th2 + th3)*sin(th5), -cos(th2 + th3)*sin(th4), sin(th2 + th3)*cos(th5) + cos(th2 + th3)*cos(th4)*sin(th5), d4*sin(th2 + th3) + a2*cos(th2)]
% [ cos(th2 + th3)*sin(th5) + sin(th2 + th3)*cos(th4)*cos(th5), -sin(th2 + th3)*sin(th4), sin(th2 + th3)*cos(th4)*sin(th5) - cos(th2 + th3)*cos(th5), a2*sin(th2) - d4*cos(th2 + th3)]
% [                                          cos(th5)*sin(th4),                 cos(th4),                                          sin(th4)*sin(th5),                               0]

syms  d1 d6 nx ox ax px ny oy ay py nz oz az pz;
Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];
T1i=DH_inverse(th1,d1,0,pi/2); 
T6i=DH_inverse(th6,d6,0,0);
Ti=T1i*Tq*T6i;
%% GP7 noap
% nx=-sin(th6)*(cos(th4)*sin(th1)+sin(th4)*(cos(th1)*sin(th2)*sin(th3)+cos(th1)*cos(th2)*cos(th3)))-cos(th6)*(cos(th5)*(sin(th1)*sin(th4)-cos(th4)*(cos(th1)*sin(th2)*sin(th3)+cos(th1)*cos(th2)*cos(th3)))+sin(th5)*(cos(th1)*cos(th2)*sin(th3)-cos(th1)*cos(th3)*sin(th2)));
% ny=sin(th6)*(cos(th1)*cos(th4)-sin(th4)*(sin(th1)*sin(th2)*sin(th3)+cos(th2)*cos(th3)*sin(th1)))+cos(th6)*(cos(th5)*(cos(th1)*sin(th4)+cos(th4)*(sin(th1)*sin(th2)*sin(th3)+cos(th2)*cos(th3)*sin(th1)))-sin(th5)*(cos(th2)*sin(th1)*sin(th3)-cos(th3)*sin(th1)*sin(th2)));
% nz=cos(th6)*(cos(th2-th3)*sin(th5)-sin(th2-th3)*cos(th4)*cos(th5))+sin(th2-th3)*sin(th4)*sin(th6);
% 
% ox=cos(th6)*(cos(th4)*sin(th1)+sin(th4)*(cos(th1)*sin(th2)*sin(th3)+cos(th1)*cos(th2)*cos(th3)))-sin(th6)*(cos(th5)*(sin(th1)*sin(th4)-cos(th4)*(cos(th1)*sin(th2)*sin(th3)+cos(th1)*cos(th2)*cos(th3)))+sin(th5)*(cos(th1)*cos(th2)*sin(th3)-cos(th1)*cos(th3)*sin(th2)));
% oy=sin(th6)*(cos(th5)*(cos(th1)*sin(th4)+cos(th4)*(sin(th1)*sin(th2)*sin(th3)+cos(th2)*cos(th3)*sin(th1)))-sin(th5)*(cos(th2)*sin(th1)*sin(th3)-cos(th3)*sin(th1)*sin(th2)))-cos(th6)*(cos(th1)*cos(th4)-sin(th4)*(sin(th1)*sin(th2)*sin(th3)+cos(th2)*cos(th3)*sin(th1)));
% oz=sin(th6)*(cos(th2-th3)*sin(th5)-sin(th2-th3)*cos(th4)*cos(th5))-sin(th2-th3)*cos(th6)*sin(th4);
% 
% ax=cos(th5)*(cos(th1)*cos(th2)*sin(th3)-cos(th1)*cos(th3)*sin(th2))-sin(th5)*(sin(th1)*sin(th4)-cos(th4)*(cos(th1)*sin(th2)*sin(th3)+cos(th1)*cos(th2)*cos(th3)));
% ay=sin(th5)*(cos(th1)*sin(th4)+cos(th4)*(sin(th1)*sin(th2)*sin(th3)+cos(th2)*cos(th3)*sin(th1)))+cos(th5)*(cos(th2)*sin(th1)*sin(th3)-cos(th3)*sin(th1)*sin(th2));
% az=-cos(th2-th3)*cos(th5)-sin(th2-th3)*cos(th4)*sin(th5);
% 
% px=a1*cos(th1)+a2*cos(th1)*cos(th2)+d4*sin(th2-th3)*cos(th1)+a3*cos(th1)*cos(th2)*cos(th3)+a3*cos(th1)*sin(th2)*sin(th3)+d6*sin(th1)*sin(th4)*sin(th5)+d6*sin(th2-th3)*cos(th1)*cos(th5)-d6*cos(th1)*cos(th2)*cos(th3)*cos(th4)*sin(th5)-d6*cos(th1)*cos(th4)*sin(th2)*sin(th3)*sin(th5);
% py=a1*sin(th1)+a2*cos(th2)*sin(th1)+d4*sin(th2-th3)*sin(th1)+a3*cos(th2)*cos(th3)*sin(th1)-d6*cos(th1)*sin(th4)*sin(th5)+a3*sin(th1)*sin(th2)*sin(th3)+d6*sin(th2-th3)*cos(th5)*sin(th1)-d6*cos(th2)*cos(th3)*cos(th4)*sin(th1)*sin(th5)-d6*cos(th4)*sin(th1)*sin(th2)*sin(th3)*sin(th5);
% pz=d1-a2*sin(th2)+d4*cos(th2-th3)-a3*sin(th2-th3)+(d6*sin(th2-th3)*sin(th4+th5))/2+d6*cos(th2-th3)*cos(th5)-(d6*sin(th2-th3)*sin(th4-th5))/2;

% m6=tan(th2-th3).*(ox*cos(th1)+oy*sin(th1))+oz;
% n6=tan(th2-th3).*(nx*cos(th1)+ny*sin(th1))+nz;
% pq=ax*cos(th1)+ay*sin(th1);
% a=(-az*sin(th2-th3)-(-ax*cos(th1)-ay*sin(th1))*cos(th2-th3));
% b=sin(th6)*(ny*cos(th1)-nx*sin(th1))-cos(th6)*(oy*cos(th1)-ox*sin(th1));
% c=-az*cos(th2-th3)+(-ax*cos(th1)-ay*sin(th1))*sin(th2-th3);
% p1=sin(th6)*(ox*cos(th1)+oy*sin(th1))+cos(th6)*(nx*cos(th1)+ny*sin(th1)); 
% q1=nz*cos(th6)+oz*sin(th6);  
% p2=-ax*cos(th1)-ay*sin(th1);  
% q2=-az;
% T=simplify(p2*sin(th2-th3)+q2*cos(th2-th3))
% T=simplify(b)
% T=simplify(c)
% cos(th2)*cos(th5)*sin(th3)-cos(th3)*cos(th5)*sin(th2)+cos(th2)*cos(th3)*cos(th4)*sin(th5)+cos(th4)*sin(th2)*sin(th3)*sin(th5)
% 0==-cos(th2-th3)*cos(th5)-sin(th2-th3)*cos(th4)*sin(th5);

% simplify(Ti)
% [ cos(th6)*(nx*cos(th1) + ny*sin(th1)) - sin(th6)*(ox*cos(th1) + oy*sin(th1)),   sin(th6)*(nx*cos(th1) + ny*sin(th1)) + cos(th6)*(ox*cos(th1) + oy*sin(th1)), ax*cos(th1) + ay*sin(th1), px*cos(th1) - d6*(ax*cos(th1) + ay*sin(th1)) + py*sin(th1)]
% [                                                   nz*cos(th6) - oz*sin(th6),                                                     oz*cos(th6) + nz*sin(th6),                        az,                                            pz - d1 - az*d6]
% [ sin(th6)*(oy*cos(th1) - ox*sin(th1)) - cos(th6)*(ny*cos(th1) - nx*sin(th1)), - sin(th6)*(ny*cos(th1) - nx*sin(th1)) - cos(th6)*(oy*cos(th1) - ox*sin(th1)), ax*sin(th1) - ay*cos(th1), d6*(ay*cos(th1) - ax*sin(th1)) - py*cos(th1) + px*sin(th1)]
% -------------------------------------------------------------------------
% 方程组

% -sin(th2+th3)*sin(th5) - cos(th2+th3)*cos(th4)*cos(th5) = cos(th6)*(nx*cos(th1)+ny*sin(th1))-sin(th6)*(ox*cos(th1)+oy*sin(th1))
%  cos(th2+th3)*sin(th5) + sin(th2+th3)*cos(th4)*cos(th5) = nz*cos(th6)-oz*sin(th6)
%                                       sin(th4)*cos(th5) = sin(th6)*(oy*cos(th1)-ox*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1))
% 
% -cos(th2+th3)*sin(th4) = sin(th6)*(nx*cos(th1)+ny*sin(th1))+cos(th6)*(ox*cos(th1)+oy*sin(th1))
% -sin(th2+th3)*sin(th4) = oz*cos(th6)+nz*sin(th6)
%               cos(th4) = -sin(th6)*(ny*cos(th1)-nx*sin(th1))-cos(th6)*(oy*cos(th1)-ox*sin(th1))
% 
%  sin(th2+th3)*cos(th5) + cos(th2+th3)*cos(th4)*sin(th5) = ax*cos(th1)+ay*sin(th1)
% -cos(th2+th3)*cos(th5) + sin(th2+th3)*cos(th4)*sin(th5) = az
%                                       sin(th4)*sin(th5) = ax*sin(th1)-ay*cos(th1)
% 
% a2*cos(th2) + d4*sin(th2+th3) = -d6*(ax*cos(th1)+ay*sin(th1)) + px*cos(th1)+py*sin(th1)
% a2*sin(th2) - d4*cos(th2+th3) = pz-d1-az*d6
%                             0 = d6*(ay*cos(th1)-ax*sin(th1)) + px*sin(th1)-py*cos(th1)

%% 末端三系合一模型，GP7；
% d1=100;  a2=-500;a3=-400;
% d=[d1 0 0 0 0 0]; a=[0 a2 a3 0 0 0];
alpha=[pi/2 0 pi/2 -pi/2 pi/2 0]; 
%       theta  d   a  alpha  sigma offset  %mdh默认0
L1 = Link([0  100  0   pi/2    0    0]); 
L2 = Link([0   0 -500    0     0    0]);
L3 = Link([0   0 -400  pi/2    0    0]);
L4 = Link([0   0   0  -pi/2    0    0]);
L5 = Link([0   0   0   pi/2    0    0]);
L6 = Link([0   0   0     0     0    0]);
% q1=[0 -pi/2 0 0 -pi/2 0];
q0=[0 0 0 0 0 0];
robot_GP7=SerialLink([L1,L2,L3,L4,L5,L6],'name','GP7'); 
% robot_GP7.display();
% robot_GP7.teach(q0);

% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|        100|          0|     1.5708|          0|
% |  2|         q2|          0|       -500|          0|          0|
% |  3|         q3|          0|       -400|     1.5708|          0|
% |  4|         q4|          0|          0|    -1.5708|          0|
% |  5|         q5|          0|          0|     1.5708|          0|
% |  6|         q6|          0|          0|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+

syms th1 th2 th3 th4 th5 th6;
theta=[th1 th2 th3 th4 th5 th6];
% forward_16(theta)
% T25=T2*T3*T4*T5==T1i*Tq*T6i
% T36=T3*T4*T5*T6==T2i*T1i*Tq
T=forward_16(theta);  %修改函数，求 T25
simplify(T)
% pretty(T)
% T2*T3*T4*T5
% [ cos(th2 + th3)*cos(th4)*cos(th5) - sin(th2 + th3)*sin(th5), -cos(th2 + th3)*sin(th4), sin(th2 + th3)*cos(th5) + cos(th2 + th3)*cos(th4)*sin(th5), a3*cos(th2 + th3) + a2*cos(th2)]
% [ cos(th2 + th3)*sin(th5) + sin(th2 + th3)*cos(th4)*cos(th5), -sin(th2 + th3)*sin(th4), sin(th2 + th3)*cos(th4)*sin(th5) - cos(th2 + th3)*cos(th5), a3*sin(th2 + th3) + a2*sin(th2)]
% [                                          cos(th5)*sin(th4),                 cos(th4),                                          sin(th4)*sin(th5),                               0]

% T1i*Tq*T6i
syms  d1 a2 a3 nx ox ax px ny oy ay py nz oz az pz;
Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];
T1i=DH_inverse(th1,d1,0,pi/2); 
T6i=DH_inverse(th6,0,0,0);
Ti=T1i*Tq*T6i;
simplify(Ti)
% [ cos(th6)*(nx*cos(th1) + ny*sin(th1)) - sin(th6)*(ox*cos(th1) + oy*sin(th1)),   sin(th6)*(nx*cos(th1) + ny*sin(th1)) + cos(th6)*(ox*cos(th1) + oy*sin(th1)), ax*cos(th1) + ay*sin(th1), px*cos(th1) + py*sin(th1)]
% [                                                   nz*cos(th6) - oz*sin(th6),                                                     oz*cos(th6) + nz*sin(th6),                        az,                   pz - d1]
% [ sin(th6)*(oy*cos(th1) - ox*sin(th1)) - cos(th6)*(ny*cos(th1) - nx*sin(th1)), - sin(th6)*(ny*cos(th1) - nx*sin(th1)) - cos(th6)*(oy*cos(th1) - ox*sin(th1)), ax*sin(th1) - ay*cos(th1), px*sin(th1) - py*cos(th1)]
% ----------------------------------------------------------------
% -sin(th2+th3)*sin(th5)+cos(th2+th3)*cos(th4)*cos(th5) == cos(th6)*(nx*cos(th1)+ny*sin(th1))-sin(th6)*(ox*cos(th1)+oy*sin(th1))
%  cos(th2+th3)*sin(th5)+sin(th2+th3)*cos(th4)*cos(th5) == nz*cos(th6)-oz*sin(th6)
% cos(th5)*sin(th4) == sin(th6)*(oy*cos(th1)-ox*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1))
% 
% -cos(th2+th3)*sin(th4) == sin(th6)*(nx*cos(th1)+ny*sin(th1))+cos(th6)*(ox*cos(th1)+oy*sin(th1))
% -sin(th2+th3)*sin(th4) == oz*cos(th6)+nz*sin(th6)
% cos(th4) == -sin(th6)*(ny*cos(th1)-nx*sin(th1))-cos(th6)*(oy*cos(th1)-ox*sin(th1))
% 
% sin(th2+th3)*cos(th5)+cos(th2+th3)*cos(th4)*sin(th5) == ax*cos(th1)+ay*sin(th1)
% sin(th2+th3)*cos(th4)*sin(th5)-cos(th2+th3)*cos(th5) == az
% sin(th4)*sin(th5) == ax*sin(th1)-ay*cos(th1)
% 
% a2*cos(th2)+a3*cos(th2+th3) == px*cos(th1)+py*sin(th1)
% a2*sin(th2)+a3*sin(th2+th3) == pz-d1
% 0 == px*sin(th1)-py*cos(th1)



%% main
% syms th1 th2 th3 th4 th5 th6 py; %global py;
% theta=[th1 th2 th3 th4 th5 th6]; %note them to avoid format long;
% theta_deg=[10 20 -70 40 50 60]
% theta_deg=[10 0 90 40 50 60]
theta_deg=[0 37.4313 -34.2981 0 0 0];
% theta_deg=[-90 90 30 40 0 60]
% theta_deg=[180*(rand*2-1) 180*(rand*2-1) 180*(rand*2-1) 180*(rand*2-1) 180*(rand*2-1) 180*(rand*2-1) ]
theta=theta_deg/180*pi;
% T=forward_16(theta) %正运动
% Q_deg=inverse_GP7(T) %逆运动
% Ti=inverse_kine(theta)
% T=forward_16(th(2,1:6))
% th=Q_deg*pi/180;
for i=1:8
%     eval(['Q',num2str(i),'=','forward_16(th(i,1:6))']);
%     eval(['Q',num2str(i),'=','N_zero(forward_16(th(i,1:6))-T)']);
end

%% ikine
function theta_deg=inverse_GP7(T)

d1=100;d4=-440;d6=-80;a1=40;a2=460;a3=40;
d=[d1 0 0 d4 0 d6];    a=[a1 a2 a3 0 0 0];
nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
% sin(th2-th3)*sin(th5)+cos(th2-th3)*cos(th4)*cos(th5)=sin(th6)*(ox*cos(th1)+oy*sin(th1))+cos(th6)*(nx*cos(th1)+ny*sin(th1)) ①
% cos(th2-th3)*sin(th5)-sin(th2-th3)*cos(th4)*cos(th5)=nz*cos(th6)+oz*sin(th6)  ②
% sin(th4)*cos(th5)=sin(th6)*(oy*cos(th1)-ox*sin(th1))+cos(th6)*(ny*cos(th1)-nx*sin(th1))  ③×
% 
% cos(th2-th3)*sin(th4)=cos(th6)*(ox*cos(th1)+oy*sin(th1))-sin(th6)*(nx*cos(th1)+ny*sin(th1)) ④
% sin(th2-th3)*sin(th4)=nz*sin(th6)-oz*cos(th6)  ⑤
% cos(th4)=sin(th6)*(ny*cos(th1)-nx*sin(th1))-cos(th6)*(oy*cos(th1)-ox*sin(th1))  ⑥
% 
% sin(th2-th3)*cos(th5)-cos(th2-th3)*cos(th4)*sin(th5)=-ax*cos(th1)-ay*sin(th1)  ⑦
% cos(th2-th3)*cos(th5)+sin(th2-th3)*cos(th4)*sin(th5)=-az  ⑧
% sin(th4)*sin(th5)=ay*cos(th1)-ax*sin(th1)  ⑨×
% 
% a2*cos(th2)+a3*cos(th2-th3)+d4*sin(th2-th3)=-a1+d6*(ax*cos(th1)+ay*sin(th1))+px*cos(th1)+py*sin(th1)  ⑩
% a2*sin(th2)+a3*sin(th2-th3)-d4*cos(th2-th3)=-d6*az-(pz-d1)  ①①
% 0=d6*(ay*cos(th1)-ax*sin(th1))+py*cos(th1)-px*sin(th1)  ①②
%-------------------------------------------------
% th1
% 0=d6*(ay*cos(th1)-ax*sin(th1))+py*cos(th1)-px*sin(th1)  ①②
% 0=(d6*ay+py)*cos(th1)-(d6*ax+px)*sin(th1)
n1=d6*ax+px;  n1=N_zero(n1);
m1=d6*ay+py;  m1=N_zero(m1);
if m1==0 && n1==0 %Singularity①
    th1(1,1)=0;% th1(1,1)=theta(1);
    th1(1,2)=pi;% th1(1,2)=theta(1)-pi;
else
    th1(1,1)=atan2(m1,n1);
    th1(1,2)=th1(1,1)-pi; % atan2(-m1,-n1);
end
th10=th1*180/pi;

% a2*cos(th2)+a3*cos(th2-th3)+d4*sin(th2-th3)=-a1+d6*(ax*cos(th1)+ay*sin(th1))+px*cos(th1)+py*sin(th1)  ⑩
% a2*sin(th2)+a3*sin(th2-th3)-d4*cos(th2-th3)=-d6*az-(pz-d1)  ①①
m23=-a1+d6*(ax*cos(th1)+ay*sin(th1))+px*cos(th1)+py*sin(th1); %m23=N_zero(m23);
n23=-d6*az-(pz-d1); %n23=N_zero(n23);atan(m23,n23)不奇异，不需全零判定；
% th2
%{ 
% departure version
k2=(2*a2*a3*cos(th3(1:2,1:2))+a2^2+a3^2-m23.^2-n23^2-d4^2)./sqrt((2*n23*d4).^2+(2*m23*d4).^2); %这个计算精度，怎么回事；
k2=K_one(k2);
% if n23==0 && m23==0 %不存在，对应a2*cos(th2)+a3*cos(th2-th3)+d4*sin(th2-th3)的几何意义；
% 0<(a2^2-sqrt(a3^2+d4^2))^2<=m23^2+m23^2<=(a2^2+sqrt(a3^2+d4^2))^2
th23(1:2,1:2)=atan2(2*n23*d4,2*m23*d4)-atan2(k2,-sqrt(1-k2.^2));  %error th23
th23(3:4,1:2)=atan2(2*n23*d4,2*m23*d4)-atan2(k2,sqrt(1-k2.^2));

th2=th23+th3; %correction;
j=1; %persistent j;
for i=1:4
    cs1(i,1:2)=(a2*cos(th2(i,1:2))+a3*cos(th23(i,1:2))+d4*sin(th23(i,1:2)))-(-a1+d6*(ax*cos(th1)+ay*sin(th1))+px*cos(th1)+py*sin(th1));
    cs2(i,1:2)=(a2*sin(th2(i,1:2))+a3*sin(th23(i,1:2))-d4*cos(th23(i,1:2)))-(-d6*az-(pz-d1));
    cs1(i,1:2)=N_zero(cs1(i,1:2));
    cs2(i,1:2)=N_zero(cs2(i,1:2));
    if cs1(i,1:2)==[0 0]&cs2(i,1:2)==[0 0]  %zeros(1,2)
        th2(j,1:2)=th2(i,1:2);
        j=j+1;
    end
end
th2(3:4,1:2)=th2(1:2,1:2);
th23=th2-th3;
th20=th2*180/pi;
%}
k2=(m23.^2+n23^2+a2^2-a3^2-d4^2)./(2*a2*sqrt(m23.^2+n23.^2));
k2=K_one(k2); 
th2(1,1:2)=atan2(k2,sqrt(1-k2.^2))-atan2(m23,n23); %a2=460>0;
th2(2,1:2)=atan2(k2,-sqrt(1-k2.^2))-atan2(m23,n23);
th20=th2*180/pi;

% th3
k3=(m23.^2+n23^2-a2^2-a3^2-d4^2)/(2*a2*sqrt(a3^2+d4^2));
k3=K_one(k3); 
th3(1,1:2)=atan2(a3,d4)-atan2(k3,-sqrt(1-k3.^2));
th3(2,1:2)=atan2(a3,d4)-atan2(k3,sqrt(1-k3.^2));
th30=th3*180/pi;
th23=th2-th3;
% -------------------------------

% th6
% cos(th2-th3)*sin(th4)=cos(th6)*(ox*cos(th1)+oy*sin(th1))-sin(th6)*(nx*cos(th1)+ny*sin(th1))  ④
% sin(th2-th3)*sin(th4)=nz*sin(th6)-oz*cos(th6)  ⑤ 
% =>tan(th2-th3)=(nz*s6-oz*c6)/(c6*(ox*c1+oy*s1)-s6*(nx*c1+ny*s1));
m6=tan(th2-th3).*(ox*cos(th1)+oy*sin(th1))+oz;  m6=N_zero(m6);
% m6=(sin(th5)*sin(th6))/cos(th2-th3)%化简结果
n6=tan(th2-th3).*(nx*cos(th1)+ny*sin(th1))+nz;  n6=N_zero(n6);
% n6=(sin(th5)*cos(th6))/cos(th2-th3)
th6(1:2,1:2)=atan2(m6,n6);
th6(3:4,1:2)=th6(1:2,1:2)-pi;
% if sin(th5)==0;% 奇异点2；
if m6(1,1)==0&&n6(1,1)==0 %是否全矩阵判断；
    th6(1:2:3,1)=0;
end
% if th2-th3==pi/2,tan(th23)=inf;自带数据处理，忽略非无穷大，约去无穷大；
% atan2((ox*cos(th1)+oy*sin(th1)),(nx*cos(th1)+ny*sin(th1)))*180/pi
% if abs(sin(th23(1,1)))==1
%     s=sign(tan(th23(1,1)));%提取正负号；
%     th6(1:2:3,1)=atan2(s*(ox*cos(th1)+oy*sin(th1)),s*(nx*cos(th1)+ny*sin(th1)));
% end
% sin(th4)==0时,不影响th6；
% th61=atan2(ox*cos(th1)+oy*sin(th1),nx*cos(th1)+ny*sin(th1))*180/pi;
% th62=atan2(oz,nz)*180/pi; %证实，与atan2(m6,n6)计算结果相同；
th60=th6*180/pi;
%% th45
th3(3:4,1:2)=th3(1:2,1:2);
th2(3:4,1:2)=th2(1:2,1:2);

% th4
% cos(th4)=sin(th6)*(ny*cos(th1)-nx*sin(th1))-cos(th6)*(oy*cos(th1)-ox*sin(th1))  ⑥
% cos(th2-th3)*sin(th4)=cos(th6)*(ox*cos(th1)+oy*sin(th1))-sin(th6)*(nx*cos(th1)+ny*sin(th1)) ④
% sin(th2-th3)*sin(th4)=nz*sin(th6)-oz*cos(th6)  ⑤
n4=sin(th6).*(ny*cos(th1)-nx*sin(th1))-cos(th6).*(oy*cos(th1)-ox*sin(th1));
n4=N_zero(n4);% 运用原理sin^2+cos^2=1,得到正负号，需要取舍。
% n4=K_one(n4); %系统计算bug；
th23=N_zero(th2-th3);
if th23(1,1)==0||th23(2,1)==0%when sin(th2-th3)==0,using equation 6;
    m4=(cos(th6).*(ox*cos(th1)+oy*sin(th1))-sin(th6).*(nx*cos(th1)+ny*sin(th1)))./(cos(th2-th3));
else
    m4=(nz*sin(th6)-oz*cos(th6))./(sin(th2-th3));
end
th4=atan2(m4,n4);
th40=th4*180/pi;
%{
% departure version
 th4(1:4,1:2)=atan2(-sqrt(1-n4.^2),n4);
th4(5:8,1:2)=-th4(1:4,1:2); 
th4(1:4,1)=atan2(sqrt(1-n4(1:4,1).^2),n4(1:4,1));
th4(5:8,1)=-th4(1:4,1); 
th4(1:4,2)=atan2(-sqrt(1-n4(1:4,2).^2),n4(1:4,2));
th4(5:8,2)=-th4(1:4,2);  %这里，所有th4都满足等式6；
th23(5:8,1:2)=th23(1:4,1:2);%矩阵放大；
th6(5:8,1:2)=th6(1:4,1:2);
j=1;
for i=1:8
    cs1(i,1:2)=(cos(th23(i,1:2)).*sin(th4(i,1:2)))-(cos(th6(i,1:2)).*(ox*cos(th1)+oy*sin(th1))-sin(th6(i,1:2)).*(nx*cos(th1)+ny*sin(th1)));
    cs2(i,1:2)=(sin(th23(i,1:2)).*sin(th4(i,1:2)))-(nz*sin(th6(i,1:2))-oz*cos(th6(i,1:2)));
    cs1(i,1:2)=N_zero(cs1(i,1:2));
    cs2(i,1:2)=N_zero(cs2(i,1:2));
    if cs1(i,1:2)==[0 0]&cs2(i,1:2)==[0 0]
        th4(j,1:2)=th4(i,1:2);
        j=j+1;
    end
end
th23=th23(1:4,1:2); %矩阵缩小
th6=th6(1:4,1:2);
th4=th4(1:4,1:2);
% m4=(nz*sin(th6)-oz*cos(th6))./sin(th23); % sin(th4)
% m4=N_zero(m4);
% % m4==0&&n4==0 %Singularity O1 O4同轴
% th4(1:4,1:2)=atan2(m4,n4);
% th40=th4*180/pi
%}

% -------------------------------
% th5
%{
% departure version
% sin(th4)*cos(th5)=sin(th6)*(oy*cos(th1)-ox*sin(th1))+cos(th6)*(ny*cos(th1)-nx*sin(th1))  
% sin(th4)*sin(th5)=ay*cos(th1)-ax*sin(th1)  
% m5=(ay*cos(th1)-ax*sin(th1))./sin(th4) %sin(th4)==0,将导致cos(th5)sin(th5)取值无效；
% n5=(sin(th6).*(oy*cos(th1)-ox*sin(th1))+cos(th6).*(ny*cos(th1)-nx*sin(th1)))./sin(th4)
% m5=(ay*cos(th1)-ax*sin(th1)); %矩阵蠕变，运算精度，导致无穷小的非零；
% n5=(sin(th6).*(oy*cos(th1)-ox*sin(th1))+cos(th6).*(ny*cos(th1)-nx*sin(th1)));
% cos4==0,theta4=±pi/2,m5=0/0；矩阵蠕变，
% (-az*sin(th23)-(-ax*cos(th1)-ay*sin(th1)).*cos(th23));
% cos4=sin(th6).*(ny*cos(th1)-nx*sin(th1))-cos(th6).*(oy*cos(th1)-ox*sin(th1)); n4=N_zero(n4)
% -ax*cos(th1)-ay*sin(th1)==0
% m5=(-az*sin(th2-th3)-(-ax*cos(th1)-ay*sin(th1)).*cos(th2-th3))./cos(th4)
%}
% sin(th2-th3)*sin(th5)+cos(th2-th3)*cos(th4)*cos(th5)=sin(th6)*(ox*cos(th1)+oy*sin(th1))+cos(th6)*(nx*cos(th1)+ny*sin(th1)) ①
% cos(th2-th3)*sin(th5)-sin(th2-th3)*cos(th4)*cos(th5)=nz*cos(th6)+oz*sin(th6)  ②
% sin(th2-th3)*cos(th5)-cos(th2-th3)*cos(th4)*sin(th5)=-ax*cos(th1)-ay*sin(th1)  ⑦
% cos(th2-th3)*cos(th5)+sin(th2-th3)*cos(th4)*sin(th5)=-az  ⑧
% sin(th5)=p1*sin(th2-th3)+q2*cos(th2-th3);
% cos(th5)=p7*sin(th2-th3)+q8*cos(th2-th3);
m5=(sin(th6).*(ox*cos(th1)+oy*sin(th1))+cos(th6).*(nx*cos(th1)+ny*sin(th1))).*sin(th2-th3)+(nz*cos(th6)+oz*sin(th6)).*cos(th2-th3)
n5=(-ax*cos(th1)-ay*sin(th1)).*sin(th2-th3)+(-az)*cos(th2-th3)
th5=atan2(m5,n5); 
% cos4=N_zero(cos(th4))% cs0(1,1)-cs0(3,1)=pi;
% if cos4(1,1)==0&&cos4(3,1)==0 
%     m5=sqrt(1-(-ax*cos(th1)-ay*sin(th1)).^2-(-az)^2);
%     th5(1,1)=atan2(m5(1,1),n5(1,1));
%     th5(3,1)=-th5(1,1);
% end
th50=th5*180/pi
%% sort out;
for i=1:2 
    for j=1:4
        th(4*(i-1)+j,1)=In_pi(th1(1,i)); %th1;
        th(4*(i-1)+j,2)=In_pi(th2(j,i)); %th2;
        th(4*(i-1)+j,3)=In_pi(th3(j,i)); %th3;
        th(4*(i-1)+j,6)=In_pi(th6(j,i)); %th6;
        th(4*(i-1)+j,4)=In_pi(th4(j,i)); %th4;
        th(4*(i-1)+j,5)=In_pi(th5(j,i)); %th5;
    end
end
theta_deg=th*180/pi;
end
%% function1
function T = forward_16(theta)
    syms d1 d4 d6 a1 a2 a3; 
%     d1=100;d4=-440;d6=-80;a1=40;a2=460;a3=40;
%     d=[d1 0 0 d4 0 d6];    a=[a1 a2 a3 0 0 0];
%     alpha=[-pi/2 pi -pi/2 pi/2 -pi/2 pi];

%     d=[d1 0 0 d4 0 d6]; a=[0 a2 0 0 0 0];
%     alpha=[pi/2 0 pi/2 -pi/2 pi/2 0]; 
    
    % d1=100;  a2=-500;a3=-400;
    d=[d1 0 0 0 0 0]; a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 pi/2 -pi/2 pi/2 0]; 
    
    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
%     T=T1*T2*T3*T4*T5*T6;
%     T=T3*T4*T5*T6;
    T=simplify(T2*T3*T4*T5);
end

function T = DH_forward(theta,d,a,alpha)
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end
%% function2
function Ti = inverse_kine(theta)% 
global py; %函数内不允许有 py；
    syms d1 d4 d6 a1 a2 a3 nx ox ax px ny oy ay py nz oz az pz;
%     T = forward_16(theta);
%     nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
%     ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
%     nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];
%     d1=100;d4=-440;d6=-80;a1=40;a2=460;a3=40;
%     d=[d1 0 0 d4 0 d6];    a=[a1 a2 a3 0 0 0];
%     alpha=[-pi/2 pi -pi/2 pi/2 -pi/2 pi];  
    
    d=[d1 0 0 0 0 0]; a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 pi/2 -pi/2 pi/2 0]; 
    
    T1i=DH_inverse(theta(1),d(1),a(1),alpha(1)); 
    T2i=DH_inverse(theta(2),d(2),a(2),alpha(2));
    T3i=DH_inverse(theta(3),d(3),a(3),alpha(3));
    T4i=DH_inverse(theta(4),d(4),a(4),alpha(4));
    T5i=DH_inverse(theta(5),d(5),a(5),alpha(5));
    T6i=DH_inverse(theta(6),d(6),a(6),alpha(6));
%     Ti=simplify(T3i*T2i*T1i*Tq);
%     Ti=simplify(Tq*T6i*T5i*T4i);
    Ti=simplify(T1i*Tq*T6i);

end
function T = DH_inverse(theta,d,a,alpha)
    T=invtrot(trotx(alpha))*invtrans(transl(a,0,0))*invtrot(trotz(theta))*invtrans(transl(0,0,d));
end

function [Ti]=invtrot(T)
    for i=1:2
        for j=i+1:3
            temp=T(i,j);
            T(i,j)=T(j,i);
            T(j,i)=temp;
        end
    end
    Ti=T;
end
function [Ti]=invtrans(T)
    for i=1:3
        T(i,4)=-T(i,4);
    end
    Ti=T;
end

%------------------------------------------------
function mn = N_zero(mn)
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<2E-6
                mn(i,j)=0;
            end
        end
    end
end   
% 
function k = K_one(k)
    S=size(k);
    for i=1:S(1)
        for j=1:S(2)
            if abs(k(i,j))>1
                k(i,j)=NaN;
            end
        end
    end
end
% 
function theta = In_pi(theta)
    while (abs(theta)>pi)
        if (theta>pi)
            theta=theta-2*pi;
        elseif (theta<-pi)
            theta=theta+2*pi;
        end
    end
end
