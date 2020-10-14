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

%% UR5
theta=[0 0 0 0 0 0]; %��������֮��ļнǣ�
d=[90 -100 100 90 90 90]; %��������֮��ľ��룻
a=[0 -420 -400 0 0 0]; %���ؽ�����֮�乫���߳��ȣ�
alpha=[pi/2 0 0 pi/2 -pi/2 0]; %���ؽ�����֮��нǣ�
offset=[0 0 0 0 0 0]; %[0 -pi/2 0 -pi/2 0 0]; %��ʼ�Ƕ�theta��ƫת����
sigma=0; mdh=0;%��ת�ؽڣ����ģ��Ĭ��T1ΪT01��

% L1 = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
% L2 = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
% L3 = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
% L4 = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
% L5 = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
% L6 = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);
% robot_UR5=SerialLink([L1,L2,L3,L4,L5,L6],'name','UR5'); 
% robot_UR5.display();  
% robot_UR5.teach(); 

% theta=[0 0 0 0 0 0];
% theta=[0 -pi/2 0 -pi/2 0 0]; %��ʼƫ�ư汾
% d=[90 0 0 90 90 90]; % d=[90 -100 100 90 90 90]; 
% % a=[0 420 400 0 0 0]; 
% a=[0 -420 -400 0 0 0]; 
% alpha=[pi/2 0 0 pi/2 -pi/2 0];
% offset=[0 0 0 0 0 0]; % offset=[0 -pi/2 0 -pi/2 0 0]; 
% sigma=0; mdh=0;

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
    
    
    