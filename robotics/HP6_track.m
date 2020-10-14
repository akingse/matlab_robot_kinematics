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


