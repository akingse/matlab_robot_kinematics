%% initialize
clc; close all; clear all;
format shortg; format compact;

%% �����亯��ʹ��
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
% % bot.plot(q1)  %���ɶ�̬ͼ��

%% �����亯�� robotics toolbox for matlab

% Q2=[ 0 0 0 pi/2 pi/2 pi/2];% Q3=[-pi/6 -pi/6 -pi/6 pi/3 0 pi];
% T0=[-1 0 0 0;0 0 -1 -200;0 -1 0 1080;0 0 0 1];%�ؽڽ�Ϊ0��Ĭ�ϳ�ʼλ�ã�
% T2=[1 0 0 -580;0 1 0 -100;0 0 1 600;0 0 0 1];%q2=[0 pi/2 -pi/2 pi/2 pi/2 pi/2];
% T2=[-1 0 0 -480;0 0 -1 -200;0 -1 0 600;0 0 0 1];%q2=[0 pi/2 -pi/2 0 0 0];
% Q0=ikine(R6,T0)
% Q1=ikine(R6,T2)%�������棬���������ս⣻
% T0=[-1 0 0 0;0 0 -1 -200;0 -1 0 1080;0 0 0 1];
% T1=[0.5 -0.866 0 673;0 0 -1 -200;0.8660 0.5 0 665.7;0 0 0 1]
% q1=[0 -pi/6 -pi/6 -pi/3 0 0];
% q0=[0 0 0 0 0 0];
% T= fkine(R6,q0)%����
% qi=ikine(R6,T) %����

% qr=[0 0 0 pi/2 pi/2 -pi/2 ];
% %qz=[0 0 0 0 0 0];%�������ؽڵĽǶ�
% fkine(R6,qr) %�˶�ѧ����������
% t=0:0.1:1; 
% % q=jtraj(qz,qr,t);%����N���Ƕ����ݣ�
% T=fkine(R6,qz)
%--------------------------------------------------------------------------
N=20;T=0:0.05:2; %N/Tѡ��һ
% q=ctraj(T0,T2,N);%λ�˾���ռ�켣�滮���ѿ������꣩
% q=jtraj(Q0,Q1,N);%�ؽڽǿռ�켣�滮��
Q0=[0 0 0 0 0 0]; Q1=[0 pi/2 -pi/2 pi/2 pi/2 pi/2];
[q qd qdd]=jtraj(Q0,Q1,T); %��λ�� ���ٶ� �Ǽ��ٶȣ�����3��N*6���飻
% q=jtraj(Q0,Q1,N);
% plot(robot_UR5,q);%������ʾ,(Ŀǰ��jtraj�ؽڽ�)
% i=2;
% grid on;
% plot(T,q(:,i),'b-','linewidth',1);%��T�������ã�
% hold on;
% plot(T,qd(:,i),'k--','linewidth',1);
% hold on;
% plot(T,qdd(:,i),'c:','linewidth',1.5);
% leg1=legend('q','qd','qdd','Location','Best');%ͼ��˵��,����˳�����Σ�
% title('i��'); 
% xlabel('t'), ylabel('\theta \omega \alpha');
% set(plot,'Location','Best');%'North' 'South' 'East' 'West' 'Best' 'BestOutside'   
% text(1.4,-0.7,'\leftarrow�ؽ�2');%�趨ָ��λ�õı�ǩ��
% gtext('���ָ��'); 
%\alpha \beta \gamma \delta \epsilon \zeta \eta \theta \iota \kappa \lambda \mu \nu \xi \omicron \pi \rho \sigma \tau \upsilon \phi \chi \psi \omega
%��ɫ'b' ��ɫ'g'����ɫ'r' ����'c' �Ϻ�'m' ��ɫ'y' ��ɫ'k' 
%ʵ��'-' ����'--' ����':' '.' �㻮��'-.' '+-*o<>v^'
% 'square'='s' 'pentagram'='p' 'diamond' 'hexagram' 
% axis([xmin,xmax,ymin,ymax]);%����ͼ��ķ�Χ


%% MATLAB�㷨����RTB
% Robotics,Vision and Control;
% ������ѧ�������Ӿ�����ơ���MATLAB�㷨����
% theta=pi/6;
% T1=SE2(1,2,pi/6);  %��α任������xy�ƶ���z��ת��
% T01=[1 0 1;0 1 2; 0 0 1];%inv(T01)  %invtrans()
% T02=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
% %��׼������% inv(T02)==%invrot()==(T02)'
% T01*T02;%==T1
% T2=SE2(2,1,0);
% axis([0 5 0 5]);
% grid on;
% T3=T1*T2;
% T4=T2*T1;
% trplot2(T1,'frame','1','color','b');  %��̬ͼ��
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
% % p1=inv(T1)*[p;1];  % ��Σ���ά
% h2e([p;1]);%==homtrans����,��άת��ά
% e2h(p);


%% ���

% %��е�ṹ-�˶�ѧ
% ������ƣ���̬��̬������������������Ԫ�Ż���е������
% %����ѧ����
% ����simulink�����˶�ѧ����ģ�ͣ�ʵʱ����ĩ�˱仯��
% matlab Adams 3dmax���Ϸ��棻
% %�켣�滮
% �ؽڹ켣���ζ���ʽ��ֵ���ռ�����ֱ��Բ���켣��ϲ�ֵ���㣻
% ���ٶȣ����ٶȣ�����Լ��ʱ����ʱ�����Ž�Ĺ켣�㷨��
% B�������߽��й滮���������ײ��ɢ·����
% 3-5-3������������֤�滮׼ȷ�Ⱥ;��ȣ�
% ��������·���ٵѿ����ռ�滮�����߹սǴ�������ζ���ʽ���ȷ����й滮��ʹ����·������ƽ����
% �ؽڿռ�͵ѿ����ռ�������������켣�滮�㷨������������˵�λ���ٶȼ��ٶȣ�

% �켣�滮
% 1��ƽ��һά�켣����ζ���ʽ���Ľ���ζ���ʽ
% 2����ά�켣���ֶι켣

% �˶�ѧ
% 1�����˶�ѧ
% 2�����˶�ѧ
% ��д�㷨�������⣨�ܿ�����㣩
% �ؽڲ岹���ѿ���ֱ�߲岹
% 
% ����ѧ
% 1���ؽ��ٶȣ��ſɱȾ��������ԣ�
% 2���������ش���
% 3���˶����̣��������󣬿���simulink����
% ----------------------------------------------------
% �ٶ��ſɱȾ���
% [dx dy]'=J*[d��1 d��2]'
% J=[(?X/?��1)*d��1 (?X/?��1)*d��2;(?Y/?��1)*d��1 (?Y/?��1)*d��2];
% v=dx/dt=J*d��=J*��;
% ��=inv(J)*v;
% �����ԣ�inv(J)=(J^*)/det(J);��det(J)==0ʱ��inv(J)=�ޣ��ٶ������
% 
% ����ѧ�������ſɱȾ���  J'
% ��=J'*F
% ��=[��1 ��2 ��3]',F=[Fx Fy Fz]';

% ����ѧ�����⡪����֪�ؽڵ��������أ��������ϵͳ��Ӧ���˶������������ؽ�λ�ơ��ٶȺͼ��ٶȣ���
% ����ѧ�����⡪����֪�˶��켣���ϵĹؽ�λ�ơ��ٶȺͼ��ٶȣ��������Ҫ�Ĺؽ����ء�
% ���������ɶ�����˺Ͷ���ؽ���ɵĸ��ӵĶ���ѧϵͳ�����ж������Ͷ������������Ŵ��۸��ӵ���Ϲ�ϵ�����صķ����ԡ�
% ���õķ�����
% ��������(Lagrange)����
% ţ�١�ŷ������(Newton-Euler)����,% ��˹(Gauss)����,% ����(Kane)����
% �������պ���L��һ����еϵͳ�Ķ���Ek������Eq֮�Ek=f(��);Eq=f(��);
% L��Ek-Eq; 
% �������շ��̣�����M
% M[]=d(?L/?��)/dt-?L/?��;
% -----------------------------------
% R^.(t)=S(��)*R(t)
S=skew([1 2 3])
omega=vex(S)











