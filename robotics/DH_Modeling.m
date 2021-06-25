%% initialize
clc; close all; clear all;
format shortg; format compact;

%% DH����
% DH��Ϊ��׼DH�͸Ľ�DH����ģ��ƽ����ת�任˳��������ͬ��һ�ιؽڱ任����4��������ı任������Ϊ������ͬ�Ĺؽڵı任����
% ����ʹ�ñ任���� T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);Ϊ�Զ������4������ȫΪǰ�ؽڵı任����

%  'standard'��ʾ��׼DH����������i��i+1֮�䣬'modified'�Ľ�DH����������i-1��i֮�䣻
% L = Link([THETAi Di Ai-1 ALPHAi-1 SIGMA],'modified');
% % theta����Zi�ᣬ��Xi-1��ת��Xi�ĽǶ�
% % D����Zi�ᣬ��Xi-1�ƶ���Xi�ľ���
% % A����Xi�ᣬ��Zi�ƶ���Zi+1�ľ���
% % alpha����Xi�ᣬ��Zi��ת��Zi+1�ĽǶ�

% L = Link([THETA D A ALPHA SIGMA],'standard');
% % theta����Zi�ᣬ��Xi��ת��Xi+1�ĽǶ�
% % D����Zi�ᣬ��Xi�ƶ���Xi+1�ľ���
% % A����Xi�ᣬ��Zi�ƶ���Zi+1�ľ���
% % alpha����Xi+1�ᣬ��Zi��ת��Zi+1�ĽǶ�
% Li = Link([�ؽڽǶ� ����ƫ�� ���˳��� ����Ťת�� sigma��ת0�ƶ�1 �ؽ�ƫ����],'standard');
% Li = Link([theta(1) d(1) a(1) alpha(1) 0 offset],'standard');


%% ��׼UR������
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
q0=[0 0 0 0 0 0];%��ʼֵ
% R6.plot(T) ;%3d Figure
R6.display();
R6.teach(q0);%Teach panel,(x,y,z,R,P,Y)


%% UR5
theta=[0 0 0 0 0 0]; %��������֮��ļнǣ�
% theta=[0 -pi/2 0 -pi/2 0 0]; %��ʼƫ�ư汾
d=[90 -100 100 90 90 90]; %��������֮��ľ��룻
a=[0 -420 -400 0 0 0]; %���ؽ�����֮�乫���߳��ȣ�
alpha=[pi/2 0 0 pi/2 -pi/2 0]; %���ؽ�����֮��нǣ�
offset=[0 0 0 0 0 0]; %[0 -pi/2 0 -pi/2 0 0]; %��ʼ�Ƕ�theta��ƫת����
sigma=0; mdh=0;%��ת�ؽڣ����ģ��Ĭ��T1ΪT01��

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
% AK5ƽ��������ϵ��ģ
% L(1)=Link([0 100 0 pi/2 0 pi]);%ʹL2��L1xת90��L2��L1zƽ��100��
% L(2)=Link([0 120 480 0 0 pi/2]);%ʹL3��L2zƽ��120����L2xƽ��480��֮��L3��L2zת90��
% L(3)=Link([0 -120 400 0 0 0]);%ʹL4��L3zƽ��-120����L3xƽ��400��
% L(4)=Link([0 100 0 pi/2 0 pi/2]);%ʹL5��L4xת90����L4zת90����L4zƽ��100��
% L(5)=Link([0 100 0 -pi/2 0 0]);%ʹL6��L5xת-90����L5zƽ��100��
% L(6)=Link([0 100 0 0 0 0]);%ʹĩ������ϵTO����L6zƽ��100��

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
%% AK5����Z����ϵ��ģ
% RotX=[1 0 0 0;0 cos(theta) -sin(theta) 0;0 sin(theta) cos(theta) 0;0 0 0 1];
% RotY=[cos(theta) 0 sin(theta) 0;0 1 0 0;-sin(theta) 0 cos(theta) 0;0 0 0 1];
% RotZ=[cos(theta) -sin(theta) 0 0;sin(theta) cos(theta) 0 0;0 0 1 0;0 0 0 1];
% Trans(a b c)=[1 0 0 a;0 1 0 b;0 0 1 c;0 0 0 1];%Trans������R��Ϊ��λ���ҳ�˳���޹أ�
% l1=100;l2=120;l3=480;l4=-120;l5=400;l6=100;l7=100;l8=100;
% ÿ������T���������ϵ������Link��ģʹ����Standardָ����һ������ϵ���˴���7��T��
% ����������귽�򣬵�123���зֱ�����µ�xyz�����ڵ�123���Ŵ����Ӧ��һ������ϵ��xyz����
% syms th1 th2 th3 th4 th5 th6;
% % th1=0;th2=0;th3=0; th4=0;th5=0;th6=0;
% T1=[cos(th1) -sin(th1) 0 0;sin(th1) cos(th1) 0 0;0 0 1 0;0 0 0 1];%T=RotZ;
% T2=[cos(th2) -sin(th2) 0 0;0 0 -1 0;sin(th2) cos(th2) 0 100; 0 0 0 1];%RotX*T+Trans;
% T3=[-sin(th3) -cos(th3) 0 0;cos(th3) -sin(th3) 0 480;0 0 1 120;0 0 0 1];%RotZ,offset�޸��˳�ʼ�Ƕȣ�
% T4=[cos(th4) -sin(th4) 0 400; sin(th4) cos(th4) 0 0;0 0 1 -120;0 0 0 1];%Trans;
% T5=[0 0 1 0;cos(th5) -sin(th5) 0 0;sin(th5) cos(th5) 0 100;0 0 0 1];%RotZ*RotX*Tz,˳����أ�
% T6=[cos(th6) -sin(th6) 0 0;0 0 1 100;-sin(th6) -cos(th6) 0 100;0 0 0 1];%RotX,ZY���ƶ��ϲ���
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
% b=isrevolute(L1);  %Link �ຯ��
% robot=SerialLink([L1,L2,L3,L4,L5,L6]);   %SerialLink �ຯ��
% robot.name='SAR';     %SerialLink ����ֵ
% obot.manuf='plgk';     %SerialLink ����ֵ
% robot.display();  %Link �ຯ��
% theta=[0 0 0 0 0 0];
% robot.plot(theta);   %SerialLink �ຯ��
% theta1=[pi/4,-pi/3,pi/6,pi/4,-pi/3,pi/6];
% p0=robot.fkine(theta);
% p1=robot.fkine(theta1);
% s=robot.A([4 5 6],theta);
% cchain=robot.trchain;
% q=robot.getpos();



%% zwh
%  L1 = Link('d', 0, 'a', 0, 'alpha',-pi/2, 'offset', pi/2,'qlim',[0 pi/2]);%��������
%  L2 = Link('theta', 0, 'a', 0, 'alpha',0,'qlim',[20 30]);
%  L3 = Link('d', 0, 'a', 20, 'alpha', pi/2);
%  L4 = Link('d',0, 'a', 0, 'alpha', pi/2,'qlim',[0 pi]);
%  L5 = Link('theta', 0, 'a', 0, 'alpha', 0,'qlim',[0 20]);
%  L6 = Link('d', 20, 'a', 0, 'alpha', 0);
% 
%  robot = SerialLink([L1 L2 L3 L4 L5 L6],'base',troty(pi/2));%��������
%  robot.display();%��ʾD-H������
%  robot.name = 'robot';
%  theta = [0,0,0,0,0,0];
%  robot.plot(theta,'workspace',[-50,50,-50,50,-50,50]); %��ʾ�����˵�ͼ��
%  robot.teach;
% para=[0 20 0 0 0 0];
% T=forward_kine(para)

%% ��ţ
% L1=Link([0 0.085 0 0 0],'standard');
% L2=Link([0 0 0 -pi/2 1],'standard');
% L3=Link([0 0 -0.5 pi/2 0],'standard');
% L2.qlim=[0.2 0.6];
% bot=SerialLink([L1 L2 L3],'name','���ȷ���');
% bot.display();%��ʾD-H������
% bot.teach(); 
% qA=[0 0 0];
% plot(bot,qA);

% t=[0:0.1:3];
% qA=[0 0.2 0];
% qB=[pi/6 0.6 pi/6];
% q=jtraj(qA,qB,t);%���ɹؽ��˶��켣
% T=fkine(bot,q)%�����˶�ѧ���溯�� 
% plot(bot,q);%���ɻ����˵��˶�
% figure('Name','����ĩ��λ��ͼ')
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
% figure('Name','up6������ĩ�˹켣ͼ');

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
% Ҫ���������˶�����������Ҫ�趨�����˵�D-H������֮�����ǿ�������Robotics Toolbox�������е�link��robot����������
% 
% ����link�����ĵ��ø�ʽ��
% L = LINK([ alpha A thetaD])
% L =LINK ([ alpha A theta D sigma ])
% L =LINK([ alpha A theta D sigma offset ])
% L =LINK([ alpha A theta D ], CONVENTION  ])
% L =LINK([alpha A theta D sigma],CONVENTION  ])
% L =LINK([ alpha A theta D sigma offset ],CONVENTION )

% 
% ����CONVENTION����ȡ��standard���͡�modified�������С�standard��������ñ�׼��D-H��������modified��������øĽ���D-H������������alpha������Ťת�� ��������A������˼����ȣ�������theta������ؽڽǣ�������D�������࣬������sigma������ؽ����ͣ�0������ת�ؽڣ���0�����ƶ��ؽڡ�����LINK����һЩ������
% 	LINK.alpha	    %����Ťת��
% 	LINK.A        %���ظ˼�����
% 	LINK.theta       %���عؽڽ�
% 	LINK.D        %���غ��
% 	LINK.sigma     %���عؽ�����
% 	LINK.RP	      %���ء�R��(��ת)��P��(�ƶ�)
% 	LINK.mdh      %��Ϊ��׼D-H��������0�����򷵻�1
% 	LINK.offset	    %���عؽڱ���ƫ��
% 	LINK.qlim	     %���عؽڱ����������� (min max)
% 	LINK.islimit(q)	%����ؽڱ������ޣ����� -1, 0, +1
% 	LINK.I		%����һ��3��3 �Գƹ��Ծ���
% 	LINK.m		%���عؽ�����
% 	LINK.r		%����3��1�Ĺؽڳ�������
%     LINK.G	%���س��ֵĴ�����
% 	LINK.Jm	%���ص������
% 	LINK.B		%����ճ��Ħ��
% 	LINK.Tc	%���ؿ���Ħ��
% 	LINK.dh		return legacy DH row
% 	LINK.dyn	    return legacy DYN row
% % ����robot�����ĵ��ø�ʽ��
% 	ROBOT			      %����һ���յĻ����˶���
% 	ROBOT(robot)		   %����robot��һ������
% 	ROBOT(robot, LINK)	%��LINK�������»����˶���������robot
% 	ROBOT(LINK, ...)	    %��LINK������һ�������˶���
% 	ROBOT(DH, ...)		    %��D-H����������һ�������˶���
% 	ROBOT(DYN, ...)		%��DYN����������һ�������˶���
% 2���任����
% ����MATLAB��Robotics Toolbox�������е�transl(x,y,z)��rotx(��)��roty(��)��rotz(��)����ʵ������α任�����ʾƽ�Ʊ任����ת�任��
% 3 �켣�滮
% ����Robotics Toolbox�ṩ��ctraj��jtraj��trinterp��������ʵ�ֵѿ����滮���ؽڿռ�滮�ͱ任��ֵ��
% ����ctraj�����ĵ��ø�ʽ��
% 	TC = CTRAJ(T0, T1, N)
% 	TC = CTRAJ(T0, T1, R)
% ����TCΪ��T0��T1�ĵѿ����滮�켣��NΪ���������RΪ����·������������R��ÿ��ֵ������0��1֮�䡣
% ����jtraj�����ĵ��ø�ʽ��
%  
% [Q QD QDD] = JTRAJ(Q0, Q1,N)
% [Q QD QDD] = JTRAJ(Q0, Q1, N, QD0, QD1)
% [Q QD QDD] = JTRAJ(Q0, Q1,T)
% [Q QD QDD] = JTRAJ(Q0, Q1, T, QD0, QD1)


% ����QΪ��״̬Q0��Q1�Ĺؽڿռ�滮�켣��NΪ�滮�ĵ�����TΪ������ʱ�������ĳ��ȣ��ٶȷ���߽������QD0��QD1��ָ����QD��QDDΪ���صĹ滮�켣���ٶȺͼ��ٶȡ�
% ����trinterp�����ĵ��ø�ʽ��
% TR = TRINTERP(T0, T1, R)
% ����TRΪ��T0��T1֮�������仯��ֵ��R����0��1֮�䡣
% Ҫʵ�ֹ켣�滮����������Ҫ����һ��ʱ�����������������������ĳ�����������������56ms����ô���������µ�������ʵ�ֶ���ʽ�켣�滮��t=0:0.056:2; 
%  
% ����tΪʱ��������qzΪ�����˵ĳ�ʼλ�ˣ�qrΪ�����˵�����λ�ˣ�qΪ������·���㣬qdΪ�˶����ٶȣ�qddΪ�˶��ļ��ٶȡ�����q��qd��qdd�������еľ���ÿ�д���ÿ���ؽڵ�λ�á��ٶȺͼ��ٶȡ���q(:,3)����ؽ�3��λ�ã�qd(:,3)����ؽ�3���ٶȣ�qdd(:,3)����ؽ�3�ļ��ٶȡ�
% 4 �˶�ѧ��������
% ����Robotics Toolbox�е�fkine��������ʵ�ֻ������˶�ѧ���������⡣
% ����fkine�����ĵ��ø�ʽ��
% TR = FKINE(ROBOT, Q)
% ����ROBOTΪһ�������˶���TRΪ��Q�����ÿ��ǰ���˶�ѧ�����⡣
% ��PUMA560Ϊ��������ؽ�����ϵ�����qz=(0 0 0 0 0 0)����ôfkine(p560,qz)���������һ���ؽڵ�ƽ�Ƶ���α任����������˹ؽڵĹ켣�滮֮������Ҳ������fkine�������˶�ѧ�����⡣���磺
% t=0:0.056:2; q=jtraj(qz,qr,t); T=fkine(p560,q);
% ���صľ���T��һ����ά�ľ���ǰ��ά��4��4�ľ����������仯������ά��ʱ�䡣
% 
% 5 �˶�ѧ��������
% ����Robotics Toolbox�е�ikine��������ʵ�ֻ������˶�ѧ���������⡣
% ����ikine�����ĵ��ø�ʽ��
% 	Q = IKINE(ROBOT, T)
% 	Q = IKINE(ROBOT, T, Q)
% 	Q = IKINE(ROBOT, T, Q, M)
% ����ROBOTΪһ�������˶���QΪ��ʼ�²�㣨Ĭ��Ϊ0����TΪҪ����ı任���󡣵�����Ļ����˶�������ɶ�����6ʱ��Ҫ��M���к���ĳ���ؽ����ɶȡ�
% ���˹ؽڵĹ켣�滮֮������Ҳ������ikine�����������˶�ѧ���������⡣���磺
% t=0:0.056:2; T1=transl(0.6,-0.5,0); T2=transl(0.4,0.5,0.2); T=ctraj(T1,T2,length(t)); q=ikine(p560,T); 
% 
% T=fkine(p560,q); qi=ikine(p560,T);
% 6 ������ʾ
% ���˻����˵Ĺ켣�滮֮�����ǾͿ�������Robotics Toolbox�е�plot������ʵ�ֶԹ滮·���ķ��档
% puma560;T=0:0.056:2; q=jtraj(qz,qr,T); plot(p560,q);
% ��Ȼ������Ҳ����������PUMA560��������ת�ǣ���ʵ�ֶ�����ʾ��
% drivebot(p560)
% ��׼



