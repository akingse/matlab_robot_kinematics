%% initialize
clc; close all; clear all;
format shortg; format compact;

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
8自由度计算程序
调用 ikine_Virtual_2 ikine_Actual_8函数
如论文所述，脚本有两个版本，一是利用原DOF9的Actual_8函数，计算时乘T_A6=trotx(pi/2)；
二是为Actual6添加alpha6，重新Actual_8a函数；
两个版本均通过 T_RCM T_end验证；

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}


%% main
% 随机位姿% 初始化位姿
% T_RCM=transl(-500,-90,17.321) %实体六轴的
T_RCM =[
            1            0            0      -600.94;
            0            1            0     -0.58306;
            0            0            1        99.91;
            0            0            0            1]
T_end =[
     -0.80163     -0.20506     -0.56156      -1153.7+100*(2*rand(1)-1); %设置随机位置
      0.33367     -0.93288     -0.13566      -5.0095+100*(2*rand(1)-1);
     -0.49605     -0.29612      0.81624      -438.82+100*(2*rand(1)-1);
            0            0            0            1]

% 函数调用测试
% q_deg=[10 20 30 40 50 60]
% q=q_deg/180*pi;
% T= forward_kine_a(q)
% Q=ikine_Actual_8a(T);
% Q_deg=Q*180/pi %print()
% for i=1:8
%     T8= forward_kine_a(Q(i,1:6))-T;
%     T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4))
% end

%% 使用 Actual_8 函数
T_AV=trotx(pi/2);
T_A6=trotx(pi/2);
T_V=invtrot(T_AV)*invtrans(T_RCM)*T_end;
thv=ikine_Virtual_2(T_V)
% 注意，调用逆解函数时:
% Actual6的d6=800，两个函数的区别在于alpha6
% Virtual6的3个参数为(d7) th8 th9，与DOF8所需参数th7 th8对应；

% thv对应两组解
T1=T_end*inverse_kine98(thv(1,1:9))*invtrot(T_A6);
T2=T_end*inverse_kine98(thv(2,1:9))*invtrot(T_A6);
tha1=ikine_Actual_8(T1);  tha1_deg=tha1*180/pi
tha2=ikine_Actual_8(T2);  tha2_deg=tha2*180/pi
for i=1:8 %check RCM 
    T_c=forward_kine_16(tha1(i,1:6))*invtrans(transl(0,0,thv(1,7)));
    T_veri(i,1:3)=[T_c(1,4) T_c(2,4) T_c(3,4)];
    T_c=forward_kine_16(tha2(i,1:6))*invtrans(transl(0,0,thv(2,7)));
    T_veri(8+i,1:3)=[T_c(1,4) T_c(2,4) T_c(3,4)];
end
T_veri %验证RCM点

% 合并
    theta(1:8,1:6)=tha1;
    theta(9:16,1:6)=tha2;
    for i=1:8 % 区别于变量名整体赋值，索引逐个赋值
        theta(i,7:8)=thv(1,8:9);
        theta(8+i,7:8)=thv(2,8:9);
    end
    theta*180/pi

for i=1:16
    forward_kine(theta(i,1:8)); %验证T_end
    T=forward_kine(theta(i,1:8))-T_end;
    Tc(i)=abs(T(1,4))+abs(T(2,4))+abs(T(3,4));
end
Tc
fprintf("--------------------------\n");  

%% 使用 Actual_8a 函数
T_AV=trotx(pi/2);
T_V=invtrot(T_AV)*invtrans(T_RCM)*T_end;
thv=ikine_Virtual_2(T_V);

% thv对应两组解
T1=T_end*inverse_kine98(thv(1,1:9));
T2=T_end*inverse_kine98(thv(2,1:9));
tha1=ikine_Actual_8a(T1);  tha1_deg=tha1*180/pi;
tha2=ikine_Actual_8a(T2);  tha2_deg=tha2*180/pi;
for i=1:8 %check RCM 
    T_c=forward_kine_16(tha1(i,1:6))*invtrans(transl(0,0,thv(1,7)));
    T_veri(i,1:3)=[T_c(1,4) T_c(2,4) T_c(3,4)];
    T_c=forward_kine_16(tha2(i,1:6))*invtrans(transl(0,0,thv(2,7)));
    T_veri(8+i,1:3)=[T_c(1,4) T_c(2,4) T_c(3,4)];
end
T_veri %验证RCM点

% 合并
    theta(1:8,1:6)=tha1;
    theta(9:16,1:6)=tha2;
    for i=1:8 % 区别于变量名整体赋值，索引逐个赋值
        theta(i,7:8)=thv(1,8:9);
        theta(8+i,7:8)=thv(2,8:9);
    end
    theta*180/pi

for i=1:16
    forward_kine(theta(i,1:8)); %验证T_end
    T=forward_kine(theta(i,1:8))-T_end;
    Tc(i)=abs(T(1,4))+abs(T(2,4))+abs(T(3,4));
end
Tc
%    


%% function
function T = inverse_kine98(theta)
    d=[90 0 0 90 90 90 0 0 0]; 
    a=[0 -420 -400 0 0 0 0 50 50];
    alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
    T9i=DH_inverse(theta(9),d(9),a(9),alpha(9));
    T8i=DH_inverse(theta(8),d(8),a(8),alpha(8));
    T=T9i*T8i;
end

function T = forward_kine_16(theta) %原DOF9版本，alpha_6=0
    d1=90;d4=90;d5=90;d6=800;     a2=-420;a3=-400;
    d=[d1 0 0 d4 d5 d6];
    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 0];
    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T=T1*T2*T3*T4*T5*T6;
end

function T = forward_kine_a(theta)% DOF8版本，alpha_6=pi/2
    d1=90;d4=90;d5=90;d6=800; a2=-420;a3=-400;
    d=[d1 0 0 d4 d5 d6];
    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 pi/2];
    
    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T=T1*T2*T3*T4*T5*T6;
end


function T = forward_kine(theta)% DOF8
    d1=90;d4=90;d5=90;d6=800; a2=-420;a3=-400; a7=50;a8=50;
    d=[d1 0 0 d4 d5 d6 0 0];
    a=[0 a2 a3 0 0 0 a7 a8];
    alpha=[pi/2 0 0 pi/2 -pi/2 pi/2 -pi/2 0];
    
    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(theta(7),d(7),a(7),alpha(7));
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T=T1*T2*T3*T4*T5*T6*T7*T8;
end


% ---------------------------------------------------------------------
function T = DH_forward(theta,d,a,alpha) % 正运动函数
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end
function T = DH_inverse(theta,d,a,alpha) % 逆运动函数
    T=invtrot(trotx(alpha))*invtrans(transl(a,0,0))*invtrot(trotz(theta))*invtrans(transl(0,0,d));
end

function Ti=invtrot(T) %旋转矩阵，逆矩阵
    for i=1:2
        for j=i+1:3
            temp=T(i,j);
            T(i,j)=T(j,i);
            T(j,i)=temp;
        end
    end
    Ti=T;
end
function Ti=invtrans(T) %平移矩阵，逆矩阵
    for i=1:3
        T(i,4)=-T(i,4);
    end
    Ti=T;
end


%% standard
function mn = N_zero(mn)
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<1E-6
                mn(i,j)=0;
            end
        end
    end
end   
function theta = In_pi(theta)
    S=size(theta);
    for i=1:S(1)
        for j=1:S(2)
                while (abs(theta(i,j))>pi)
                    if (theta(i,j)>pi)
                        theta(i,j)=theta(i,j)-2*pi;
                    elseif (theta(i,j)<-pi)
                        theta(i,j)=theta(i,j)+2*pi;
                    end
                end
        end
    end
end
