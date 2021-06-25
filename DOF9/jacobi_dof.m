%% initialize
clc; close all; clear all;
format shortg; format compact;
%% jacobi matrix
% 第8章 速度关系-雅可比矩阵 jacob
% 操作空间-关节空间，映射关系。T[4*4]<=>q[1*6]
% 操作空间速度-关节空间速度，映射关系。T[4*4]<=>q[1*6]
% 


%% Virtual_6 方法一，微分变换法
syms th4 th5 th6 d7 th8 th9; 
theta=[th4 th5 th6 d7 th8 th9];%虚拟六轴必须专用
syms a8 a9; 
d=[0 0 0 0 0 0]; 
a=[0 0 0 0 a8 a9]; 
% alpha=[pi/2 -pi/2 0 pi/2 -pi/2 0];%old
alpha=[-pi/2 pi/2 0 pi/2 -pi/2 0];%zyz
T1=DH_forward(theta(1),d(1),a(1),alpha(1));
T2=DH_forward(theta(2),d(2),a(2),alpha(2));
T3=DH_forward(theta(3),d(3),a(3),alpha(3));
T4=DH_forward(0,theta(4),a(4),alpha(4)); 
T5=DH_forward(theta(5),d(5),a(5),alpha(5));
T6=DH_forward(theta(6),d(6),a(6),alpha(6));
A1=T1*T2*T3*T4*T5*T6; J1=jacob(A1);
A2=T2*T3*T4*T5*T6; J2=jacob(A2);
A3=T3*T4*T5*T6; J3=jacob(A3);
A4=T4*T5*T6; J4=[A4(3,1) A4(3,2) A4(3,3) 0 0 0].';% 平移关节专用
A5=T5*T6; J5=jacob(A5);
A6=T6; J6=jacob(A6);
% J_virtual=[J1 J2 J3 J4 J5 J6].'; simplify(J_virtual) %输出其转置，方便word处理公式
% J_virtual=[J1 J2 J3 J4 J5 J6]; simplify(J_virtual) 

%% Actual_6
syms d1 d4 d5 d6 a2 a3 a8 a9;
syms th1 th2 th3 th4 th5 th6; %
theta=[th1 th2 th3 th4 th5 th6];
d=[d1 0 0 d4 d5 d6];
a=[0 a2 a3 0 0 0];
alpha=[pi/2 0 0 pi/2 -pi/2 0];

T1=DH_forward(theta(1),d(1),a(1),alpha(1));
T2=DH_forward(theta(2),d(2),a(2),alpha(2));
T3=DH_forward(theta(3),d(3),a(3),alpha(3));
T4=DH_forward(theta(4),d(4),a(4),alpha(4)); 
T5=DH_forward(theta(5),d(5),a(5),alpha(5));
T6=DH_forward(theta(6),d(6),a(6),alpha(6));

% A1=T1*T2*T3*T4*T5*T6; J1=jacob(A1); 
% A2=T2*T3*T4*T5*T6; J2=jacob(A2);
% A3=T3*T4*T5*T6; J3=jacob(A3);
% A4=T4*T5*T6; J4=jacob(A4);
% A5=T5*T6; J5=jacob(A5);
% A6=T6; J6=jacob(A6);

% J_actual=[J1 J2 J3 J4 J5 J6].'; simplify(J_actual) %输出其转置，方便word处理公式
%求行列式表达式；
% De=det(J_actual); simplify(De) %虚拟六轴 -d7*sin(th5)*(a8 + d7*sin(th8))
% f=factor(De);
% simplify(f) %实体六轴 [ sin(th5), 1, 1, 1, 1, a2*a3*sin(th3), a3*cos(th2 + th3) + a2*cos(th2) + d5*sin(th2 + th3 + th4)]

%% DOF9
    syms d1 d4 d5 d6 a2 a3 a8 a9;
    syms th1 th2 th3 th4 th5 th6 d7 th8 th9; %
    theta=[th1 th2 th3 th4 th5 th6 d7 th8 th9];
    d=[d1 0 0 d4 d5 d6 d7 0 0];
    a=[0 a2 a3 0 0 0 0 a8 a9];
    alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];

    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(0,theta(7),a(7),alpha(7)); 
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    
    A1=T1*T2*T3*T4*T5*T6*T7*T8*T9; J1=jacob(A1); simplify(J1.');
    A2=T2*T3*T4*T5*T6*T7*T8*T9; J2=jacob(A2);simplify(J2.');
    A3=T3*T4*T5*T6*T7*T8*T9; J3=jacob(A3);simplify(J3.');
    A4=T4*T5*T6*T7*T8*T9; J4=jacob(A4);simplify(J4.');
    A5=T5*T6*T7*T8*T9; J5=jacob(A5);simplify(J5.');
    A6=T6*T7*T8*T9; J6=jacob(A6);simplify(J6.');
    A7=T7*T8*T9; J7=[A7(3,1) A7(3,2) A7(3,3) 0 0 0].';simplify(J7.');
    A8=T8*T9; J8=jacob(A8);simplify(J8.');
    A9=T9; J9=jacob(A9);simplify(J9.')
    
% J=[J1 J2 J3 J4 J5 J6 J7 J8 J9].'% simplify(J)
% J=[J1 J2 J3 J4 J5 J6 J7 J8 J9];
% De=det(J); simplify(De)错误，只有方阵有行列式值；


%% 方法二，矢量法
syms th1 th2 th3 th4 th5 th6 d1 d4 d5 d6 a1 a2 a3;
th=[th1 th2 th3 th4 th5 th6];
T16=fkine_UR5(th,6);
for i=1:6 %速度雅可比矩阵；v w与th6无关；
    J(1,i)=diff(T16(1,4),th(i)); %px;
    J(2,i)=diff(T16(2,4),th(i)); %py;
    J(3,i)=diff(T16(3,4),th(i)); %pz;
end

for i=1:5
    J(4:6,1)=[0 0 1]';
    R=fkine_UR5(th,i);
    Jw=R(1:3,1:3)*[0 0 1]'; 
    J(4:6,i+1)=Jw;
end


% J_UR=simplify(J)
% simplify(det(J))% -a2*a3*sin(th5)*((d5*cos(th2 + 2*th3 + th4))/2 - (d5*cos(th2 + th4))/2 - (a2*sin(th2 + th3))/2 + (a3*sin(th2))/2 + (a2*sin(th2 - th3))/2 - (a3*sin(th2 + 2*th3))/2)

% --------------------------------------------------
T16=fkine_GP7(th,6);
for i=1:6 %速度雅可比矩阵；v w与th6无关；
    J(1,i)=diff(T16(1,4),th(i)); %px;
    J(2,i)=diff(T16(2,4),th(i)); %py;
    J(3,i)=diff(T16(3,4),th(i)); %pz;
end
for i=1:5
    R=fkine_GP7(th,i);
    J(4:6,1)=[0 0 1]';
    Jw=R(1:3,1:3)*[0 0 1]'; 
    J(4:6,i+1)=Jw;
end
% J_GP=simplify(J)

%% function
function Ji=jacob(A) %旋转轴
    nx=A(1,1);ox=A(1,2);ax=A(1,3);px=A(1,4);
    ny=A(2,1);oy=A(2,2);ay=A(2,3);py=A(2,4);
    nz=A(3,1);oz=A(3,2);az=A(3,3);pz=A(3,4);
    Ji=[px*ny-py*nx; px*oy-py*ox; px*ay-py*ax; nz; oz; az];
end


%% draft
%{
% simplify(det(J))
% simplify(J)
% J1=subs(J,{th1 th2 th3 th4 th5 th6},{0 0 0 0 0 0}) %替换函数
% J2=subs(J1,{d1 d4 d5 d6 a2 a3},{0.089459, 0.10915, 0.09465, 0.0823, -0.42500, -0.39225});
% vpa(J2,6)
% simplify(inv(J))
% J30=subs(J_UR,th5,0); %th3
% simplify(det(J30))

% J30=subs(J_GP,th5,0);
% J30=subs(J_GP,{th2 th3},{-pi/2 pi/2});
% simplify(det(J30))
%     
% det_UR=-a2*a3*sin(th5)*((d5*cos(th2+2*th3+th4))/2-(d5*cos(th2+th4))/2-(a2*sin(th2+th3))/2+(a3*sin(th2))/2+(a2*sin(th2-th3))/2-(a3*sin(th2+2*th3))/2);
% det_GP=-a2*sin(th5)*((a3^2*sin(th2))/2+(d4^2*sin(th2))/2+a1*a3*sin(th3)-(a3^2*cos(2*th3)*sin(th2))/2+(a3^2*sin(2*th3)*cos(th2))/2+(d4^2*cos(2*th3)*sin(th2))/2
% -(d4^2*sin(2*th3)*cos(th2))/2+a1*d4*cos(th3)+a3*d4*cos(2*th3)*cos(th2)+a3*d4*sin(2*th3)*sin(th2)+a2*d4*cos(th2)*cos(th3)+a2*a3*cos(th2)*sin(th3));
det_UR=-(a2*a3/2)*sin(th5)*(d5*(cos(th2+2*th3+th4)-cos(th2+th4))-a2*(sin(th2+th3)-sin(th2-th3))+a3*(sin(th2)-sin(th2+2*th3)));
det_GP=-a2*sin(th5)*((a3^2/2+d4^2/2)*sin(th2)+(d4^2/2-a3^2/2)*sin(th2-2*th3)+a3*d4*cos(th2-2*th3)+(a1+a2*cos(th2))*(a3*sin(th3)+d4*cos(th3)));
F=(a3^2/2+d4^2/2)*sin(th2)+(d4^2/2-a3^2/2)*sin(th2-2*th3)+a3*d4*cos(th2-2*th3+(a1+a2*cos(th2))*(a3*sin(th3)+d4*cos(th3)));
% subs(det_GP,th5,0)
% det=simplify(subs(det_UR,th3,pi)) %th3=0||pi;th5=0||pi;
% F=(a3^2/2+d4^2/2)*sin(th2)+(-a3^2/2+d4^2/2)*sin(th2-2*th3)+a3*d4*cos(th2-2*th3)+a2*(a3+d4)*cos(th2)*cos(th3)+a1*(a3*sin(th3)+d4*cos(th3));
% subs(F,{th2 th3},{-pi/ 2 pi/2}); %-a3^2+a1*a3=>a3(-a3+a1)=0； 特殊奇异;
% simplify(subs(F,{a3},{0})) %F(a3=0)=d4*cos(th3)*(a1+a2*cos(th2)+d4*sin(th2-th3)),此时th3=pi/2奇异点，a3~=0以避开；
% F3=simplify(subs(F,{th2},{-pi/2}))  %=>a3*sin(th3)+d4*cos(th3)=0;%% th3=atan(-d4/a3)+k*pi;
% F3=simplify(subs(F,{th2},{pi*rand(1)}));% vpa(subs(F3,{th3},{atan(-d4/a3)}))
% F3=simplify(subs(F,{th3},{atan(d4/a3)}))
% th20=solve(F3,th2,'Real', true);vpa(th20*180/pi,6)

% [the2 the3]=solve(F,th2,th3)% [ 37.4313, -34.2981]无意义，肩部奇异点1
% th30=solve(F3,th3,'Real', true);% vpa(th30*180/pi,6) %共线奇异
%}

%% 书中代码，p169
% mdl_puma560;
% J=p560.jacob0(qn);
% J=J(1:3,:);
% % plot_ellipse(J*J')
% J=p560.jacob0(qr);
% J=J(4:6,:);
% % plot_ellipse(J*J')
% sl_rrmc;
% r=sim('sl_rrmc');
% t=r.find('tout');
% q=r.find('yout');
% T=p560.fkine(q);
% xyz=transl(T);
% mplot(t,xyz(:,1:3))
% hold on;
% mplot(t,q(:,1:3))


%% function0
function T = fkine_UR5(theta,n)% 正运动
    syms d1 d4 d5 d6 a2 a3; 
%     d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    d=[d1 0 0 d4 d5 d6];    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 0];
    for i=1:6
      Ta(:,:,i)=DH_forward(theta(i),d(i),a(i),alpha(i));
    end
    T=eye(4); 
    for j=1:n
        T=T*Ta(:,:,j);
    end  % T=simplify(T);
end

function T = fkine_GP7(theta,n)
    syms d1 d4 d6 a1 a2 a3;   
    d=[d1 0 0 d4 0 d6];    a=[a1 a2 a3 0 0 0];
%     d1=100;d4=-440;d6=-80;a1=40;a2=460;a3=40; 
    alpha=[-pi/2 pi -pi/2 pi/2 -pi/2 pi];
    for i=1:6
      Te(:,:,i)=DH_forward(theta(i),d(i),a(i),alpha(i));
    end
    T=eye(4); 
    for j=1:n
        T=T*Te(:,:,j);
    end 
end

function T = DH_forward(theta,d,a,alpha)
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end

%{
% dq=1e-6;
% mdl_puma560;
% T0=p560.fkine(qn);
% Tq=p560.fkine(qn+[dq 0 0 0 0 0]);
% dTdq1=(Tq-T0)/dq;
% J=p560.jacob0(qn);
 
% mdl_ur5;
% qn=[pi*rand(1) 0 0 0 0 0];
% T0=ur5.fkine(qn);
% Tq=ur5.fkine(qn+[dq 0 0 0 0 0]);
% dTdq2=(Tq-T0)/dq;
% R(1:4)=T0 %R(4)=T0
% R=T0(1)
% RR=R(1:3,1:3)
% dRdq2=dTdq2(1:3,1:3);
% R=T0(1:3,1:3) %i give up
% S=dRdq2*R'
% mdl_ur5;
% J=ur5.jacob0(qz)
% w1=[1 0 0 0 0 0]'; %v=J*w1
%}
%{
% R(1:3,1:3)*[0 0 1]'
% syms th1 th2 th3 th4 th5 th6 py;%global py;
% theta=[th1 th2 th3 th4 th5 th6];
% theta_deg=[0 0 0 0 0 0];
% theta=theta_deg/180*pi;
% Tq=forward_kine(theta,6)
% Tp=forward_kine(theta+[dq 0 0 0 0 0],1);
% dTdq=(Tp-Tq)/dq;

% delta(1:3,1)=[dTdq(1,4) dTdq(2,4) dTdq(3,4)]'*dq;
% dt=1e-6;
% delta_dot(1:3,1)=[dTdq(1,4) dTdq(2,4) dTdq(3,4)]'*dq/dt;
% dRdq=dTdq(1:3,1:3);
% R=Tq(1:3,1:3);
% S=dRdq*R'
% omega=vex(S)  %w=vex(S)*q_dot
%}
%{
function r=mdl_ur5()
    deg = pi/180;
    a = [0, -0.42500, -0.39225, 0, 0, 0]';    d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823]';
    alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]';    theta = zeros(6,1);
    DH = [theta d a alpha];
    robot = SerialLink(DH,'name', 'UR5', 'manufacturer', 'Universal Robotics');
    links = robot.links;
    mass = [3.7000, 8.3930, 2.33, 1.2190, 1.2190, 0.1897];
    center_of_mass = [
        0,-0.02561, 0.00193
        0.2125, 0, 0.11336
        0.15, 0, 0.0265
        0, -0.0018, 0.01634
        0, -0.0018, 0.01634
        0, 0, -0.001159];
    for i=1:6
        links(i).m = mass(i);
        links(i).r = center_of_mass(i,:);
    end
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('caller', 'ur5', robot);
        assignin('caller', 'qz', [0 0 0 0 0 0]); % zero angles
        assignin('caller', 'qr', [180 0 0 0 90 0]*deg); % vertical pose as per Fig 2;
    end
end
function y=test(a,b) %c=test(1,1)
    if nargin==0 %判断输入变量个数，针对不同的情况执行不同的功能;
        a=0;b=0; %若test(),没有输入
    elseif nargin==1 %只输入一个数；
        b=0;
    end
    y=a+b;
end

%}

