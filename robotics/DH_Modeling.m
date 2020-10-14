%% initialize
clc; close all; clear all;
format shortg; format compact;

%% ��׼UR������
    a=[0 -425.00 -392.25 0 0 0];
    d=[89.159 0 0 109.15 94.65 82.30];
    alpha=[pi/2 0 0 pi/2 -pi/2 0];
    theta=[0 0 0 0 0 0];
L1 = Link([theta(1) d(1) a(1) alpha(1) 0 0]); 
L2 = Link([theta(2) d(2) a(2) alpha(2) 0 -pi/2]);
L3 = Link([theta(3) d(3) a(3) alpha(3) 0 0]);
L4 = Link([theta(4) d(4) a(4) alpha(4) 0 -pi/2]);
L5 = Link([theta(5) d(5) a(5) alpha(5) 0 0]);
L6 = Link([theta(6) d(6) a(6) alpha(6) 0 0]);

% L1 = Link('d', 0.089159, 'a', 0,        'alpha', pi/2 ,'standard' );
% L2 = Link('d', 0,        'a', -0.42500, 'alpha',   0  ,'offset', -pi/2,'standard' );
% L3 = Link('d', 0,        'a', -0.39225, 'alpha',   0  ,'standard' );
% L4 = Link('d', 0.10915,  'a', 0,        'alpha', pi/2 ,'offset', -pi/2,'standard' );
% L5 = Link('d', 0.09465,  'a', 0,        'alpha',-pi/2 ,'standard');
% L6 = Link('d', 0.08230,  'a', 0,        'alpha',   0  ,'standard');
L=([L1 L2 L3 L4 L5 L6]);

% R6=SerialLink(L, 'name', 'UR5');
% T=[0 0 0 0 0 0];%��ʼֵ
% R6.plot(T) ;%3d Figure
% teach(R6);%Teach panel,(x,y,z,R,P,Y)
% bot=SerialLink(L, 'name', 'UR5');%�����б�



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
