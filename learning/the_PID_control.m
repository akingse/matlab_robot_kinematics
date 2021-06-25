%% initialize
clc; close all; clear all;
format shortg; format compact;


%% PID控制原理和算法
% https://www.cnblogs.com/cv-pr/p/4785195.html
%   闭环控制是根据控制对象输出反馈来进行校正的控制方式，它是在测量出实际与计划发生偏差时，按定额或标准来进行纠正的。
%   比如控制一个电机的转速，就得有一个测量转速的传感器，并将结果反馈到控制路线上。提到闭环控制算法，不得不提PID，
%   它是闭环控制算法中最简单的一种。PID是比例 (Proportion) 积分 ,(Integral) 微分 ,(Differential coefficient) 的缩写，
%   分别代表了三种控制算法。通过这三个算法的组合可有效地纠正被控制对象的偏差，从而使其达到一个稳定的状态。


ts=0.001; %传递函数
sys=tf(50,[0.125,7, 0]); %tf是传递函数的意思，一般学自动控制原理的时候经常用.在s域中，比如要输入G（dus）=1/（s^2+2s+1），就可以在matlab中输入G=tf（[1],[1 2 1]）。
dsys=c2d(sys,ts,'z'); %c2d()函数的作用是将s域的表达式转化成z域的表达式，s=0对应z=1。
[num,den]=tfdata(dsys,'v'); %加上'v'，可以让输出的值由元胞数组改为数组直接输出。

u_1=0.0;u_2=0.0;  y_1=0.0;y_2=0.0;
x=[0,0,0]; %x(1);x(2);x(3) 当x为一维行或列时


error_1=0;  error_2=0;

kp=20;ki=0.1;kd=20;

for k=1:1:2000
    time(k)=k*ts;
%     S=2; %函数的选择
%     if S==1
%         rin(k)=5;                       %Step Signal
%     elseif S==2
        %kp=20;ki=0.0;kd=0;          %Sine Signal
%     kp=10;ki=0.1;kd=15;          %Sine Signal
    rin(k)=0.5*cos(2*pi*k*ts)+0.5; %控制器输入函数
    

    du(k)=kp*x(1)+kd*x(2)+ki*x(3);    %PID Controller
    u(k)=u_1+du(k);
    %Restricting the output of controller 限制输出范围
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

% rin 输入r
% yout 输出y

figure(1);
plot(time,rin,'b',time,yout,'r');
xlabel('time(s)'),ylabel('rin,yout');
title(['kp=10,ki=0.1,kd=15'],'FontSize',14,'Color','k');
% saveas(1,'pid','fig');

% figure(2);
% plot(time,error,'r')
% title(['误差变化'],'FontSize',14,'Color','k');
% xlabel('time(s)');ylabel('error');
% saveas(2,'pid_err','fig');



