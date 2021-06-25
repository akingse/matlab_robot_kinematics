%% initialize
clc; close all; clear all;
format shortg; format compact;



%% 改进五次多项式
%全体注释；
%五次多项式插值法,没有匀速段，无法提供匀速控制。
syms u;
sT=50;vmax=70;t=1;v0=375/4;%k=0.2;
% int(fun,u,0,k)*2+(1-2*k)*vmax==sT==50;
% int_v=(sT-(1-2*k)*vmax)/2;
% v0(93.75)*t(0.5)*c(1/2*0.9375)==25
% v*k*c*2+vmax*(1-2*k)=50
% (vmax/v0)*(sT/2)*(k/(t/2))*2+vmax*(1-2*k)=50
k=t*(vmax-sT)/(2*t*vmax-2*sT*(vmax/v0))  %k=10/56;
% fun50(u)=funs5(u)  %原始位移曲线
% fun51(u)=(vmax/v0)*subs(fun50(u),u,u/(2*k));
% fun52(u)=vmax*u;
% fun53(u)=subs(fun51(u),u,u-(t-2*k)); 
fun40(u)=funs4(u);  %原始速度曲线
fun41(u)=(vmax/v0)*subs(fun40(u),u,u/(2*k)); 
fun42=vmax;
fun43(u)=subs(fun41(u),u,u-(t-2*k));

fun51(u)=int(fun41(u),u,0,u);
fun52(u)=int(fun41(u),u,0,k)+vmax*(u-k);
fun53(u)=50-int(fun41(u),u,0,k)+int(fun43(u),u,(t-k),u); 

% fun41(u)=diff(fun51(u));  %新速度曲线1
% fun42=vmax;
% fun43(u)=diff(fun53(u));  %新速度曲线3
fun31(u)=diff(fun41(u));  %新加速度曲线1
fun32=0;
fun33(u)=diff(fun43(u));  %新加速度曲线3

% sub(f,old,new) %sub函数

% v0=subs(fun40(u),1/2); %v0==375/4
% v1=subs(fun41(u),k); %v1==375/4
% l0=int(fun40(u),u,0,0.5)  %l0=25
length=int(fun41(u),u,0,k)*2+vmax*(1-2*k) %length==sT==50
% funs30(u)=funs3(u);  %原始加速度曲线
% funs31(u)=subs(funs3(u),u,u/(2*k));

u=0:0.01:1;
ys=fun51(u).*(u>=0&u<k)+fun52(u).*(u>=k&u<(t-k))+fun53(u).*(u>=(t-k)&u<=t);
yv=fun41(u).*(u>=0&u<k)+fun42.*(u>=k&u<(t-k))+fun43(u).*(u>=(t-k)&u<=t);
ya=fun31(u).*(u>=0&u<k)+fun32.*(u>=k&u<(t-k))+fun33(u).*(u>=(t-k)&u<=t);
% plot(u,ys);
% hold on
% plot(u,yv);
% hold on
% plot(u,ya/8);
fg1 = figure('numbertitle','off','name','改进五段'); 
subplot(3,1,1),plot(u,ys,'r');xlabel('t'),ylabel('position(°)');grid on;
subplot(3,1,2),plot(u,yv,'g');xlabel('t'),ylabel('velocity(°/s)');grid on;
subplot(3,1,3),plot(u,ya,'b');xlabel('t'),ylabel('accelerate(°/s^2)');grid on;

% plot(u,funs5(u));
% hold on
% plot(u,[0,50*funs4(u)]);
% hold on
% plot(u,[0,2000*funs3(u),0]);


%% 五段S曲线
clc;
clear;
%初始条件
x_arry=[0,10,30,40];
 x_arry=[0,10];%单个
v_arry=[2,2,2];
A_arry=[3,3,3];
weiyi=[x_arry(1)];sudu=[0];shijian=[0];timeall=0;jiasudu=[0];jerk=[0];


for i=1:1:length(x_arry)-1;
%清空
    a=[];v=[];s=[];j=[];
%计算加减速段的时间和位移
    L=x_arry(i+1)-x_arry(i);
    A=A_arry(i);
    vs=v_arry(i);
    Ta=sqrt(vs/A);
    L1=A*(Ta^3)/6;
    L2=A*(Ta^3)*(5/6); 
%计算整段轨迹的总位移
    T=4*Ta+(L-2*L1-2*L2)/vs;
    for t=0:0.001:T
        if t<=Ta;%加加速度阶段
            jd=A;%jerk
            ad=A*t;
            vd=0.5*A*t^2;
            sd=(1/6)*A*t^3;
            a=[a,ad];v=[v,vd];s=[s,sd];j=[j,jd];
        elseif t>Ta && t<=2*Ta;%加减速阶段
            jd=-A;
            ad=-A*(t-2*Ta);
            vd=-0.5*A*(t-2*Ta)^2+A*Ta^2;
            sd=-(1/6)*A*(t-2*Ta)^3+A*Ta^2*t-A*Ta^3;
            a=[a,ad];v=[v,vd];s=[s,sd];j=[j,jd];
         elseif t>2*Ta && t<=T-2*Ta;%匀速阶段
             jd=0;
            ad=0;
            vd=vs;
            sd=A*Ta^2*t-A*Ta^3;  
            a=[a,ad];v=[v,vd];s=[s,sd];j=[j,jd];
        elseif t>T-2*Ta && t<=T-Ta;%减加度阶段
            jd=-A;
            ad=-A*(t-(T-2*Ta));
            vd=-0.5*A*(t-T+2*Ta)^2+A*Ta^2;
            sd=-(1/6)*A*(t-T+2*Ta)^3+A*Ta^2*t-A*Ta^3;
            a=[a,ad];v=[v,vd];s=[s,sd];j=[j,jd];
         elseif t>T-Ta && t<=T;%减减阶段
             jd=A;
            ad=A*(t-T);
            vd=0.5*A*(t-T)^2;
            sd=(1/6)*A*(t-T)^3-2*A*Ta^3+A*Ta^2*T;
            a=[a,ad];v=[v,vd];s=[s,sd];j=[j,jd];
        end
    end
%时间
    time=[timeall:0.001:timeall+T];
    timeall=timeall+T;
%连接每一段轨迹
    weiyi=[weiyi,s(2:end)+x_arry(i)];
    sudu=[sudu,v(2:end)];
    jiasudu=[jiasudu,a(2:end)];
    jerk=[jerk,j(2:end)];
    
    shijian=[shijian,time(2:end)];
end
fg2 = figure('numbertitle','off','name','五段S'); 

% subplot(3,1,1),plot(shijian,weiyi,'r');xlabel('t'),ylabel('position(rad)');grid on;
% subplot(3,1,2),plot(shijian,sudu,'g');xlabel('t'),ylabel('velocity(rad/s)');grid on;
% subplot(3,1,3),plot(shijian,jiasudu,'b');xlabel('t'),ylabel('accelerate(rad/s^2)');grid on;
subplot(4,1,1),plot(shijian,weiyi,'r');xlabel('t'),ylabel('position(rad)');grid on;
subplot(4,1,2),plot(shijian,sudu,'g');xlabel('t'),ylabel('velocity(rad/s)');grid on;
subplot(4,1,3),plot(shijian,jiasudu,'b');xlabel('t'),ylabel('accelerate(rad/s^2)');grid on;
subplot(4,1,4),plot(shijian,jerk);xlabel('t'),ylabel('jerk(rad/s^3)');grid on;


%% func
function [y5]=funs5(u)
s0=0; sT=50; sd0=0; sdT=0; sdd0=0; sddT=0;t=1;
k=0.2;vmax=50;
% x=[A B C D E F].';
% solve(s-T*x)
s=[s0 sT sd0 sdT sdd0 sddT].';
T=[0 0 0 0 0 1;t^5 t^4 t^3 t^2 t 1;0 0 0 0 1 0;5*t^4 4*t^3 3*t^2 2*t 1 0;0 0 0 2 0 0;20*t^3 12*t^2 6*t 2 0 0];
x=inv(T)*s;
y5=x(1,1).*u.^5+x(2,1).*u.^4+x(3,1).*u.^3+x(4,1).*u.^2+x(5,1).*u.^1+x(6,1);%位移
end
function [y4]=funs4(u)
y4=diff(funs5(u));%微分，速度

end
function [y3]=funs3(u)
y3=diff(funs4(u));%加速度
end

% 
%% 改进五次多项式，(k:1-2*k:k(k=2))
% % if u<t/5
% % tt=linspace(0,2,100);
% % x=0:0.1:2*pi;
% % y=x.*x+x;
% % plot(x,y);
% % a=0:0.01:2;
% % s4=a.*a+2.*a;
% % plot(a,s4)
% 
% % s=tpoly(0,1,50);
% % [s,sd,sdd]=tpoly(0,1,50)
% % s=tpoly(0,1,50,0,5,0);
% % plot([s,sd,sdd])
% % s=lspb(0,1,50);
% % [s,sd,sdd]=lspb(0,1,50)
% % plot(sd)
% % plot([s,sd,sdd])

%%










