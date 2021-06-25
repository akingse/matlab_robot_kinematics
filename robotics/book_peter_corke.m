clc;clear all;close all;
format short g;format compact;
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
ѧϰ������ѧ�Ĳ��ִ��룬��Ҫ�ο���Ϊ��������ѧ�������Ӿ�����ơ�
��ַhttps://petercorke.com/
Gվhttps://github.com/petercorke

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

%% λ��ת��
% ��ת����
% ��Ƿ�/��תʸ��
% RPY
% ŷ����
% ��Ԫ��

% R=rotx(pi/2);
% trplot(R);  
% tranimate(R);  %��̬ͼ��
% S=R*roty(pi/2); %����y����ת
% S=rotx(pi/2)*roty(pi/2);
% trplot(S);
% tranimate(S); 
% R=rotz(0.1)*roty(0.2)*rotz(0.3) %ZYZ����ŷ����
% EU=eul2r(0.1,0.2,0.3)%������
% tr2eul(EU) %������
rpy2tr(pi,-pi,pi/2);  %���RPY����ת���� ����
% rpy2tr(0,0,0);
% R=rpy2r(0.1,0.2,0.3)% �������ƫ���ǣ����ز������ǣ���������
% tr2rpy(R) %������
% tr2rpy(T)*180/pi
% tr2rpy(t2r(T))*180/pi
% tr2eul(T)*180/pi
% quaternion(T)
% tr2rotvec(T)
% tr2rpy(T(1:3,1:3))

%q=Quaternion() %��Ԫ����q=s<v1,v2,v3>,q=s+v=s+v1*i+v2*j+v3*k,i^2=j^2=k^2=i*j*k=-1;
% T=rpy2tr(0.1,0.2,0.3)
% q=Quaternion(T)
%λ��������Rxyz,Euler,RPY,oavec,angvec,T,R,Q

% �ѿ�������ϵ������ʸ��noa��Ĭ��T1ΪT01���Դ����ƣ���׼DH��
% ŷ����Euler(phi,theta,psi)=rot(z,phi)*rot(y,theta)*rot(z,psi)
% ����ƫRPY(phi,theta,psi)=rot(z,phi)*rot(y,theta)*rot(x,psi);%��ˣ�
% ��׼DH��ֻ�޸� d a alpha��

% function rotvec(a b c)
% ��תʸ�� k(a b c)=(ka kb kc)=(rx ry rz)
% k=sqrt(rx^2+ry^2+rz^2);% k(rx/k ry/k rz/k)
% theta=0ʱ��̬��
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
% p560  %�������
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
% qz����0,0,0,0,0,0�� ��Ƕ� 
% qr����0��pi/2,-pi/2,0,0,0�� ����״̬����е����ֱ�д�ֱ 
% qs����0,0��-pi/2��0,0,0�� ��չ״̬����е����ֱ��ˮƽ 
% qn����0��pi/4,-pi,0,pi/4,0�� ��׼״̬����е�۴���һ�����ɹ���״̬��

% q=[0 pi/6 -2*pi/3 0 0 0];
% T= fkine(p560,q);%����
% qi=ikine(p560,T) %����
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

%% �ð�
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
q1=HP6.ikine6s(T0);%������ʼ��λ�ˣ��õ���ʼ��ؽڽ�
q2=HP6.ikine6s(T11);%������ֹ��λ�ˣ��õ���ֹ��ؽڽ�
[q0 ,qd1, qdd1]=jtraj(q1,q2,50); %��ζ���ʽ�켣���õ��ؽڽǶȣ����ٶȣ��Ǽ��ٶȣ�50Ϊ���������
HP6.plot(q0);%������ʾ
TT2=HP6.fkine(q0);%���ݲ�ֵ���õ�ĩ��ִ����λ��
JTA2=transl(TT2); % ����Rbt.fkine������õѿ����ռ��л�����ÿ���켣����λ������Ӧ��ĩ��ִ����λ�����꣬������λ������ֵ����JTA�����У�RbtΪ����SerialLink���������Ļ�����ģ�ͺ�������fkineΪ�������˶�ѧ�����ĺ�����Rbt.fkine�������������4��4��ĩ��ִ����λ�˾���transl������4��4λ�˾��������λ�þ���
plot2(JTA2,'b'),hold on; % ������ɫ�ĵ�������й켣��

t=50;
Tc=ctraj(T11,T1,t);
qq=HP6.ikine6s(Tc,'b');
tt=transl(Tc);
plot2(tt,'b');
grid on;
HP6.plot(qq);%������ʾ

q3=HP6.ikine6s(T1);%������ʼ��λ�ˣ��õ���ʼ��ؽڽ�
q4=HP6.ikine6s(T2);%������ֹ��λ�ˣ��õ���ֹ��ؽڽ�
[q01 ,qd11, qdd11]=jtraj(q3,q4,50); %��ζ���ʽ�켣���õ��ؽڽǶȣ����ٶȣ��Ǽ��ٶȣ�50Ϊ���������
HP6.plot(q01),hold on;%������ʾ
T=HP6.fkine(q01);%���ݲ�ֵ���õ�ĩ��ִ����λ��
JTA=transl(T); % ����Rbt.fkine������õѿ����ռ��л�����ÿ���켣����λ������Ӧ��ĩ��ִ����λ�����꣬������λ������ֵ����JTA�����У�RbtΪ����SerialLink���������Ļ�����ģ�ͺ�������fkineΪ�������˶�ѧ�����ĺ�����Rbt.fkine�������������4��4��ĩ��ִ����λ�˾���transl������4��4λ�˾��������λ�þ���
plot2(JTA,'b'); % ������ɫ�ĵ�������й켣��

Tc1=ctraj(T2,T21,t);
qq1=HP6.ikine6s(Tc1,'b');
tt1=transl(Tc1);
plot2(tt1,'r'),hold on;
grid on;
HP6.plot(qq1);%������ʾ

q5=HP6.ikine6s(T21);%������ʼ��λ�ˣ��õ���ʼ��ؽڽ�
q6=HP6.ikine6s(T0);%������ֹ��λ�ˣ��õ���ֹ��ؽڽ�
[q011 ,qd111, qdd111]=jtraj(q5,q6,50); %��ζ���ʽ�켣���õ��ؽڽǶȣ����ٶȣ��Ǽ��ٶȣ�50Ϊ���������
HP6.plot(q011),hold on;%������ʾ
TT0=HP6.fkine(q011);%���ݲ�ֵ���õ�ĩ��ִ����λ��
JTA1=transl(TT0); % ����Rbt.fkine������õѿ����ռ��л�����ÿ���켣����λ������Ӧ��ĩ��ִ����λ�����꣬������λ������ֵ����JTA�����У�RbtΪ����SerialLink���������Ļ�����ģ�ͺ�������fkineΪ�������˶�ѧ�����ĺ�����Rbt.fkine�������������4��4��ĩ��ִ����λ�˾���transl������4��4λ�˾��������λ�þ���
plot2(JTA1,'b'); % ������ɫ�ĵ�������й켣��


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


%% vision
% 10 ����ɫ��
%{
lambda=[300:10:1000]*1e-9; %�����Ķ��巶Χ��
for T=1000:1000:6000 %�ɼ���310nm-750nm
%     plot(lambda*1e9,blackbody(lambda,T));hold all; %������׺�����
end
% ���׵ı�ʾ
lamp=blackbody(lambda,2600);
sun=blackbody(lambda,6500);
% plot(lambda*1e9,[lamp/max(lamp),sun/max(sun)]);
% ��������������-�����������գ�
% ɫ�ʣ��۾��������Ӿ�ϸ������׶��ϸ�����ر����ɫ�з�Ӧ���Ӹ�ϸ����ǿ���ַ�Ӧ��
human=luminos(lambda);
% plot(lambda*1e9,human);
% ��ʾ��ÿ�����ز����ɱ���ԭɫ����׼CIE��ԭɫ
% ɫ�ȿռ�
% ��ɫ�̼���ֵ������������ɫ������������
% C=RR+GG+BB;
lambda=[400:10:700]*1e-9;
cmf=cmfrgb(lambda);
% plot(lambda*1e9,cmf);
orange=cmfrgb(600e-9); %����600nm�ĳȹ⣬��Ҫ��ԭɫ
[r,g]=lambda2rg([400:700]*1e9);
% plot(r,g);
rg_addticks;
% XYZ �鹹�ķ�������ɫϵͳ��XZ����0���ȣ�������ȫ��Y�ṩ��
cmf=cmfxyz(lambda);
% plot(lambda*1e9,cmf);%���׹켣����
% xycolorspace
lambda2xy(550e-9);
colorname('blue');colorname('blue','xy');
colorspace('RGB->HSV',[1 0 0]);% ʵ�ֲ�ͬ��ɫ�ռ�֮���ת����RGB->ɫ�౥�Ͷ�ǿ�ȣ�
% ��ƽ�⣬������Դɫ�£�rgb1=J*rgb
% -------------------------------
flowers=iread('flowers4.png','double','gamma','sRGB'); %640*426
hsv=colorspace('RGB->HSV',flowers);
% idisp(hsv(:,:,1)); %ɫ��
% idisp(hsv(:,:,2)); %���Ͷ�
XYZ=colorspace('RGB->XYZ',flowers);
[x,y]=tristim2cc(XYZ);
xbins=[0 0.01 100];
ybins=[0 0.01 100];
% [h vx vy]=hist2d(x,y,xbins,ybins); %ͼ
% xycolorspace; %bug
% hold on;
% contour(vx,vy,h);
% [cls,cxy]=colorkmeans(flowers,7);
%}
% 11ͼ���γ�
% С�׳��������������ǰ���ά����ͶӰ�ڶ�άƽ���ϣ�ʧȥ�������Ϣ����Ϊ͸��ͶӰ
% cam=CentralCamera('focal',0.015) %����ͶӰ�����ģ�ͣ�15mm͸��
% P=[0.3 0.4 3]';
% cam.project(P)
% 
% 
%% 
% im=imread('IMG_20170619_121931.jpg'); 
% figure,imshow(im);
% title('Դͼ��');
%  
% %ѡȡͼ���ϵ�һ�������ε�ROI����  
% im0 = imcrop(im,[1198 54 2210 2210]);  
% figure,
% imshow(im0,'DisplayRange',[])  
% title('ѡȡROI���ͼ��');
 
%%%%%��˹�˲�%%%%%
sigma = 1.6;
gausFilter = fspecial('gaussian',[5 5],sigma);
blur=imfilter(im0,gausFilter,'replicate');
figure,imshow(blur);
title('��˹�˲����ͼ��');
 
level = graythresh(blur);   %%%ostu�㷨����ֵ���ж�ֵ��
im1 = im2bw(blur,level);
figure,imshow(im1);
title('��ֵͼ��');
 
bw1=bwlabel(im1,8);
stats=regionprops(bw1,'Area');
bw2 = ismember(bw1, find([stats.Area]) == max([stats.Area])); %%�ҵ�������Ķ���
figure,imshow(bw2);
title('��ֵͼ��');
[B L] = bwboundaries(bw2);  %Ѱ�ұ�Ե����������,L�Ǳ�Ǿ���
figure,   %%%�����հ׵�ͼ��
hold on
for k = 1:length(B)
    boundary = B{k};  %B��һ����Ԫ���飬������B{k}
    plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2);%% ������Ե
end%����ѭ����ʾ�������
 
 
%% 
%%%%%Ѱ��houghԲ��Բ��%%%%%%%
Rmin = 100;
Rmax = 200;
[centersBright, radiiBright] = imfindcircles(im1,[Rmin Rmax],'ObjectPolarity' ,'bright');
viscircles(centersBright,radiiBright,'EdgeColor','b');
hold on 
plot(centersBright(1),centersBright(2),'*');
hold off;
 
%% �ȷ�Բ
R=1050; t=0:pi/20:2*pi;
x=R*cos(t);
y=R*sin(t);
axis equal
n=36;a=2*pi/n;
for k=0:n-1
    hold on         %%%%%����(1077,1055)�ǰ뾶��1090�ǰ뾶����ֱ�ߵĳ���
    plot([1077-1090*cos(pi+k*a),1077+1090*cos(pi+k*a)],[1114-1090*sin(pi+k*a),1114+1090*sin(pi+k*a)])
end

