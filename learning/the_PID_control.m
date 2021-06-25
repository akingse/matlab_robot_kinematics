%% initialize
clc; close all; clear all;
format shortg; format compact;


%% PID����ԭ����㷨
% https://www.cnblogs.com/cv-pr/p/4785195.html
%   �ջ������Ǹ��ݿ��ƶ����������������У���Ŀ��Ʒ�ʽ�������ڲ�����ʵ����ƻ�����ƫ��ʱ����������׼�����о����ġ�
%   �������һ�������ת�٣��͵���һ������ת�ٵĴ������������������������·���ϡ��ᵽ�ջ������㷨�����ò���PID��
%   ���Ǳջ������㷨����򵥵�һ�֡�PID�Ǳ��� (Proportion) ���� ,(Integral) ΢�� ,(Differential coefficient) ����д��
%   �ֱ���������ֿ����㷨��ͨ���������㷨����Ͽ���Ч�ؾ��������ƶ����ƫ��Ӷ�ʹ��ﵽһ���ȶ���״̬��


ts=0.001; %���ݺ���
sys=tf(50,[0.125,7, 0]); %tf�Ǵ��ݺ�������˼��һ��ѧ�Զ�����ԭ���ʱ�򾭳���.��s���У�����Ҫ����G��dus��=1/��s^2+2s+1�����Ϳ�����matlab������G=tf��[1],[1 2 1]����
dsys=c2d(sys,ts,'z'); %c2d()�����������ǽ�s��ı��ʽת����z��ı��ʽ��s=0��Ӧz=1��
[num,den]=tfdata(dsys,'v'); %����'v'�������������ֵ��Ԫ�������Ϊ����ֱ�������

u_1=0.0;u_2=0.0;  y_1=0.0;y_2=0.0;
x=[0,0,0]; %x(1);x(2);x(3) ��xΪһά�л���ʱ


error_1=0;  error_2=0;

kp=20;ki=0.1;kd=20;

for k=1:1:2000
    time(k)=k*ts;
%     S=2; %������ѡ��
%     if S==1
%         rin(k)=5;                       %Step Signal
%     elseif S==2
        %kp=20;ki=0.0;kd=0;          %Sine Signal
%     kp=10;ki=0.1;kd=15;          %Sine Signal
    rin(k)=0.5*cos(2*pi*k*ts)+0.5; %���������뺯��
    

    du(k)=kp*x(1)+kd*x(2)+ki*x(3);    %PID Controller
    u(k)=u_1+du(k);
    %Restricting the output of controller ���������Χ
%     if u(k)>=5
%         u(k)=5;
%     end
%     if u(k)<=-5 
%         u(k)=-5;
%     end
    
    %Linear model
    yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;
    error(k)=rin(k)-yout(k);
    %Return of parameters
    u_2=u_1;u_1=u(k);
    y_2=y_1;y_1=yout(k);
    
    x(1)=error(k)-error_1;          %Calculating P
    x(2)=error(k)-2*error_1+error_2;   %Calculating D
    x(3)=error(k);      %Calculating I
    
    error_2=error_1;
    error_1=error(k);
end

% rin ����r
% yout ���y

figure(1);
plot(time,rin,'b',time,yout,'r');
xlabel('time(s)'),ylabel('rin,yout');
title(['kp=10,ki=0.1,kd=15'],'FontSize',14,'Color','k');
% saveas(1,'pid','fig');

% figure(2);
% plot(time,error,'r')
% title(['���仯'],'FontSize',14,'Color','k');
% xlabel('time(s)');ylabel('error');
% saveas(2,'pid_err','fig');



