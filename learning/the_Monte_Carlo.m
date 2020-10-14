%% setting
clc;clear;close all;
format shortg;format compact;



%% ���ؿ���

%����������ģ��
%        theta    d           a          alpha     offset
ML1=Link([0       0           0           0          0     ],'modified'); 
ML2=Link([0       0           0.180      -pi/2       0     ],'modified');
ML3=Link([0       0           0.600       0          0     ],'modified');
ML4=Link([0       0.630       0.130      -pi/2       0     ],'modified');
ML5=Link([0       0           0           pi/2       0     ],'modified');
ML6=Link([0       0           0          -pi/2       0     ],'modified');
modrobot=SerialLink([ML1 ML2 ML3 ML4 ML5 ML6],'name','modified');
modrobot.plot([0,0,0,0,0,0]);

N=100;                                              %�������
theta1=-165/180*pi+(165/180*pi+165/180*pi)*rand(N,1); %�ؽ�1����
theta2=-90/180*pi+(155/180*pi+90/180*pi)*rand(N,1) ;  %�ؽ�2����
theta3=-200/180*pi+(70/180*pi+200/180*pi)*rand(N,1) ; %�ؽ�3����
theta4=-170/180*pi+(170/180*pi+170/180*pi)*rand(N,1); %�ؽ�4����
theta5=-135/180*pi+(135/180*pi+135/180*pi)*rand(N,1); %�ؽ�5����
theta6=-360/180*pi+(360/180*pi+360/180*pi)*rand(N,1) ;%�ؽ�6����

% for n=1:10000
% % modmyt06=modrobot.fkine(theta1(n),theta2(n),theta3(n),theta4(n),theta5(n),theta6(n));
% % plot3(modmyt06(1,4),modmyt06(2,4),modmyt06(3,4),'b.','MarkerSize',0.5);
% modmyt06=modrobot.fkine([theta1(n),theta2(n),theta3(n),theta4(n),theta5(n),theta6(n)]);
% plot3(modmyt06.t(1),modmyt06.t(2),modmyt06.t(3),'b.','MarkerSize',1); 
% view(0,90);  %�趨�۲��ӽ�
% end

















