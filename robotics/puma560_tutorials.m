clc;clear all;close all;
format short g;format compact;

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



