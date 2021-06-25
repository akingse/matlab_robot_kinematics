%% 函数说明 ikine_UR5
%******************************
%函数名称：ikine_UR5
%函数功能：UR5机器人逆解
%函数输入：末端姿态，为4×4矩阵
%函数输出：各个关节角度，单位是角度，为n×6向量
%修改时间：201800403
%******************************
clc;clear all;
format short g;

% theta=[30 40 50 60 70 80];
% theta=[0 0 0 0 0 0];
theta=[pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1)];
T=fkine_UR(theta)
Q=ikine_UR(T);
Q_deg=Q*180/pi %print()
for i=1:8
    eval(['Q',num2str(i),'=','fkine_UR(Q(i,1:6),6)-T']);
end



% T=[-0.5^0.5 0 0.5^0.5 90*0.5^0.5;
%    -0.5^0.5 0 -0.5^0.5 -90*0.5^0.5;
%       0  -1   0    0;
%       0   0   0    1 ]
% Q=ikine_UR(T)

%% 函数主体
function itheta=ikine_UR(T_goat)
    %输入目标坐标系T06得到机器人逆解
    %求解思路
    %--------------------------------------------%
%     DH_UR5=load('D:\SogouExp\Robota\UR\DH_UR5.txt');
DH_UR5=[0 0 90 0;
90 0 0 -90;
0 -420 0 0;
0 -400 90 -90;
90 0 90 0;
-90 0 90 0];
    %将不为0的参数赋值
    a2=DH_UR5(3,2);
    a3=DH_UR5(4,2);
    d1=DH_UR5(1,3);
    d4=DH_UR5(4,3);
    d5=DH_UR5(5,3);
%     a2=-420;    a3=-400;    d1=90;    d4=90;    d5=90;

    %%%%%%
    theta7=0;a6=0;afa6=0;d7=DH_UR5(6,3);
    T67=[cos(theta7),-sin(theta7),0,a6;sin(theta7)*cos(afa6),cos(theta7)*cos(afa6),-sin(afa6),-sin(afa6)*d7;sin(theta7)*sin(afa6),cos(theta7)*sin(afa6),cos(afa6),cos(afa6)*d7;0,0,0,1];
    T06=T_goat/T67;
    %将T06各元素提取出来赋值
    nx=T06(1,1);    ny=T06(2,1);    nz=T06(3,1);
    ox=T06(1,2);    oy=T06(2,2);    oz=T06(3,2);
    ax=T06(1,3);    ay=T06(2,3);    az=T06(3,3);
    px=T06(1,4);    py=T06(2,4);    pz=T06(3,4);
    %%%%%%初始化itheta矩阵
    %定义变量k,其代表逆解的行名称
    k=0;
    %求解theta1
    ForJudgment=px^2+py^2-d4^2; %判定1，绝对值；
    
    if ForJudgment<-1e-6
        disp('超出工作空间，无法求解');
        itheta='';
    else
        %求解theta1
        if ForJudgment>=-1e-6&&ForJudgment<0
            ForJudgment=0;
        end
        theta1_1=atan2(py,px)-atan2(-d4,sqrt(ForJudgment));
        theta1_2=atan2(py,px)-atan2(-d4,-sqrt(ForJudgment));
        %求解theta5
        S5_1=sqrt((-sin(theta1_1)*nx+cos(theta1_1)*ny)^2+(-sin(theta1_1)*ox+cos(theta1_1)*oy)^2);
        theta5_1=atan2(S5_1,sin(theta1_1)*ax-cos(theta1_1)*ay);
        S5_2=-sqrt((-sin(theta1_1)*nx+cos(theta1_1)*ny)^2+(-sin(theta1_1)*ox+cos(theta1_1)*oy)^2);
        theta5_2=atan2(S5_2,sin(theta1_1)*ax-cos(theta1_1)*ay);
        S5_3=sqrt((-sin(theta1_2)*nx+cos(theta1_2)*ny)^2+(-sin(theta1_2)*ox+cos(theta1_2)*oy)^2);
        theta5_3=atan2(S5_3,sin(theta1_2)*ax-cos(theta1_2)*ay);
        S5_4=-sqrt((-sin(theta1_2)*nx+cos(theta1_2)*ny)^2+(-sin(theta1_2)*ox+cos(theta1_2)*oy)^2);
        theta5_4=atan2(S5_4,sin(theta1_2)*ax-cos(theta1_2)*ay);
        %当S5不等于0时可求出如下theta6,需要判断语句,并求解theta234的和， %求解theta2，%求解theta23的和,%求解theta4=theta234-theta23,%求解theta3=theta23-theta2,%将解分成8组解，以弧度为单位
        if abs(S5_1)>1e-6
            theta6_1=atan2((-sin(theta1_1)*ox+cos(theta1_1)*oy)/S5_1,(sin(theta1_1)*nx-cos(theta1_1)*ny)/S5_1);
            S234(1)=-az/S5_1;C234(1)=-(cos(theta1_1)*ax+sin(theta1_1)*ay)/S5_1;theta234(1)=atan2(S234(1),C234(1));
            B1(1)=cos(theta1_1)*px+sin(theta1_1)*py-d5*S234(1);B2(1)=pz-d1+d5*C234(1);A(1)=-2*B2(1)*a2;B(1)=2*B1(1)*a2;C(1)=B1(1)^2+B2(1)^2+a2^2-a3^2;
            if A(1)^2+B(1)^2-C(1)^2>=0
                theta2(1)=atan2(B(1),A(1))-atan2(C(1),sqrt(A(1)^2+B(1)^2-C(1)^2));theta2(2)=atan2(B(1),A(1))-atan2(C(1),-sqrt(A(1)^2+B(1)^2-C(1)^2));
                theta23(1)=atan2((B2(1)-a2*sin(theta2(1)))/a3,(B1(1)-a2*cos(theta2(1)))/a3);theta23(2)=atan2((B2(1)-a2*sin(theta2(2)))/a3,(B1(1)-a2*cos(theta2(2)))/a3);
                theta4(1)=theta234(1)-theta23(1);theta4(2)=theta234(1)-theta23(2);
                theta3(1)=theta23(1)-theta2(1);
                theta3(2)=theta23(2)-theta2(2);
                itheta(k+1,:)=[theta1_1 theta2(1) theta3(1) theta4(1) theta5_1 theta6_1];
                itheta(k+2,:)=[theta1_1 theta2(2) theta3(2) theta4(2) theta5_1 theta6_1];
                k=k+2;
            end
        end
        if abs(S5_2)>1e-6
            theta6_2=atan2((-sin(theta1_1)*ox+cos(theta1_1)*oy)/S5_2,(sin(theta1_1)*nx-cos(theta1_1)*ny)/S5_2);
            S234(2)=-az/S5_2;C234(2)=-(cos(theta1_1)*ax+sin(theta1_1)*ay)/S5_2;theta234(2)=atan2(S234(2),C234(2));
            B1(2)=cos(theta1_1)*px+sin(theta1_1)*py-d5*S234(2);B2(2)=pz-d1+d5*C234(2);A(2)=-2*B2(2)*a2;B(2)=2*B1(2)*a2;C(2)=B1(2)^2+B2(2)^2+a2^2-a3^2;
            if A(2)^2+B(2)^2-C(2)^2>=0
                theta2(3)=atan2(B(2),A(2))-atan2(C(2),sqrt(A(2)^2+B(2)^2-C(2)^2));theta2(4)=atan2(B(2),A(2))-atan2(C(2),-sqrt(A(2)^2+B(2)^2-C(2)^2));
                theta23(3)=atan2((B2(2)-a2*sin(theta2(3)))/a3,(B1(2)-a2*cos(theta2(3)))/a3);theta23(4)=atan2((B2(2)-a2*sin(theta2(4)))/a3,(B1(2)-a2*cos(theta2(4)))/a3);
                theta4(3)=theta234(2)-theta23(3);theta4(4)=theta234(2)-theta23(4);
                theta3(3)=theta23(3)-theta2(3);
                theta3(4)=theta23(4)-theta2(4);
                itheta(k+1,:)=[theta1_1 theta2(3) theta3(3) theta4(3) theta5_2 theta6_2];
                itheta(k+2,:)=[theta1_1 theta2(4) theta3(4) theta4(4) theta5_2 theta6_2];
                k=k+2;
            end
        end
        if abs(S5_3)>1e-6
            theta6_3=atan2((-sin(theta1_2)*ox+cos(theta1_2)*oy)/S5_3,(sin(theta1_2)*nx-cos(theta1_2)*ny)/S5_3);
            S234(3)=-az/S5_3;C234(3)=-(cos(theta1_2)*ax+sin(theta1_2)*ay)/S5_3;theta234(3)=atan2(S234(3),C234(3));
            B1(3)=cos(theta1_2)*px+sin(theta1_2)*py-d5*S234(3);B2(3)=pz-d1+d5*C234(3);A(3)=-2*B2(3)*a2;B(3)=2*B1(3)*a2;C(3)=B1(3)^2+B2(3)^2+a2^2-a3^2;
            if A(3)^2+B(3)^2-C(3)^2>=0
                theta2(5)=atan2(B(3),A(3))-atan2(C(3),sqrt(A(3)^2+B(3)^2-C(3)^2));theta2(6)=atan2(B(3),A(3))-atan2(C(3),-sqrt(A(3)^2+B(3)^2-C(3)^2));
                theta23(5)=atan2((B2(3)-a2*sin(theta2(5)))/a3,(B1(3)-a2*cos(theta2(5)))/a3);theta23(6)=atan2((B2(3)-a2*sin(theta2(6)))/a3,(B1(3)-a2*cos(theta2(6)))/a3);
                theta4(5)=theta234(3)-theta23(5);theta4(6)=theta234(3)-theta23(6);
                theta3(5)=theta23(5)-theta2(5);
                theta3(6)=theta23(6)-theta2(6);
                itheta(k+1,:)=[theta1_2 theta2(5) theta3(5) theta4(5) theta5_3 theta6_3];
                itheta(k+2,:)=[theta1_2 theta2(6) theta3(6) theta4(6) theta5_3 theta6_3];
                k=k+2;
            end
        end
        if abs(S5_4)>1e-6
            theta6_4=atan2((-sin(theta1_2)*ox+cos(theta1_2)*oy)/S5_4,(sin(theta1_2)*nx-cos(theta1_2)*ny)/S5_4);
            S234(4)=-az/S5_4;C234(4)=-(cos(theta1_2)*ax+sin(theta1_2)*ay)/S5_4;theta234(4)=atan2(S234(4),C234(4));
            B1(4)=cos(theta1_2)*px+sin(theta1_2)*py-d5*S234(4);B2(4)=pz-d1+d5*C234(4);A(4)=-2*B2(4)*a2;B(4)=2*B1(4)*a2;C(4)=B1(4)^2+B2(4)^2+a2^2-a3^2;
            if A(4)^2+B(4)^2-C(4)^2>=0
                theta2(7)=atan2(B(4),A(4))-atan2(C(4),sqrt(A(4)^2+B(4)^2-C(4)^2));theta2(8)=atan2(B(4),A(4))-atan2(C(4),-sqrt(A(4)^2+B(4)^2-C(4)^2));
                theta23(7)=atan2((B2(4)-a2*sin(theta2(7)))/a3,(B1(4)-a2*cos(theta2(7)))/a3);theta23(8)=atan2((B2(4)-a2*sin(theta2(8)))/a3,(B1(4)-a2*cos(theta2(8)))/a3);
                theta4(7)=theta234(4)-theta23(7);theta4(8)=theta234(4)-theta23(8);
                theta3(7)=theta23(7)-theta2(7);
                theta3(8)=theta23(8)-theta2(8);
                itheta(k+1,:)=[theta1_2 theta2(7) theta3(7) theta4(7) theta5_4 theta6_4];
                itheta(k+2,:)=[theta1_2 theta2(8) theta3(8) theta4(8) theta5_4 theta6_4];
                k=k+2;
            end
        end
        %将逆解变成角度单位
        if k>0
            itheta=itheta*180/pi;
            for i=1:k
                for j=1:6
                    if itheta(i,j)<=-180
                    itheta(i,j)=itheta(i,j)+360;
                    elseif itheta(i,j)>180
                    itheta(i,j)=itheta(i,j)-360;
                    end
                end
            end
        else 
            disp('该位姿处于奇异位置有无穷解');
        end
    end
end

%% 函数说明  fkine_UR5
%******************************
%函数名称：fkine_UR5
%函数功能：UR5机器人正解
%函数输入：各个关节角度，单位是角度，为1×6向量
%函数输出：末端姿态，为4×4矩阵
%修改时间：201800403
%******************************

%% 根据建立的DH参数得到机器人正解

function T=fkine_UR(theta)
% DH_UR5=load('D:\SogouExp\Robota\UR\DH_UR5.txt');
% 0 0 89.2 0
% 90 0 0 -90
% 0 -425 0 0
% 0 -392 109.3 -90
% 90 0 94.75 0
% -90 0 82.5 0
% --------------
DH_UR5=[0 0 90 0;
90 0 0 -90;
0 -420 0 0;
0 -400 90 -90;
90 0 90 0;
-90 0 90 0];

IfAlphabet=0;           %IfAlpahabet为真(非0)表示结果用字母显示出来，此时无论输入任何角度将被屏蔽；为假(0)表示用实数显示出来
 
if IfAlphabet
        syms theta1 theta2 theta3 theta4 theta5 theta6;
        syms a2 a3;
        syms d1 d4 d5 d6;
        afa0=DH_UR5(1,1)*pi/180;
        afa1=DH_UR5(2,1)*pi/180;
        afa2=DH_UR5(3,1)*pi/180;
        afa3=DH_UR5(4,1)*pi/180;
        afa4=DH_UR5(5,1)*pi/180;
        afa5=DH_UR5(6,1)*pi/180;
        a0=DH_UR5(1,2);
        a1=DH_UR5(2,2);
        a4=DH_UR5(5,2);
        a5=DH_UR5(6,2);
        d2=DH_UR5(2,3);
        d3=DH_UR5(3,3);
else
        theta1=theta(1)*pi/180;
        theta2=theta(2)*pi/180;
        theta3=theta(3)*pi/180;
        theta4=theta(4)*pi/180;
        theta5=theta(5)*pi/180;
        theta6=theta(6)*pi/180;
        afa0=DH_UR5(1,1)*pi/180;
        afa1=DH_UR5(2,1)*pi/180;
        afa2=DH_UR5(3,1)*pi/180;
        afa3=DH_UR5(4,1)*pi/180;
        afa4=DH_UR5(5,1)*pi/180;
        afa5=DH_UR5(6,1)*pi/180;
        a0=DH_UR5(1,2);
        a1=DH_UR5(2,2);
        a2=DH_UR5(3,2);
        a3=DH_UR5(4,2);
        a4=DH_UR5(5,2);
        a5=DH_UR5(6,2);
        d1=DH_UR5(1,3);
        d2=DH_UR5(2,3);
        d3=DH_UR5(3,3);
        d4=DH_UR5(4,3);
        d5=DH_UR5(5,3);
        d6=DH_UR5(6,3);
end
T01=[cos(theta1),-sin(theta1),0,a0;sin(theta1)*cos(afa0),cos(theta1)*cos(afa0),-sin(afa0),-sin(afa0)*d1;sin(theta1)*sin(afa0),cos(theta1)*sin(afa0),cos(afa0),cos(afa0)*d1;0,0,0,1];
T12=[cos(theta2),-sin(theta2),0,a1;sin(theta2)*cos(afa1),cos(theta2)*cos(afa1),-sin(afa1),-sin(afa1)*d2;sin(theta2)*sin(afa1),cos(theta2)*sin(afa1),cos(afa1),cos(afa1)*d2;0,0,0,1];
T23=[cos(theta3),-sin(theta3),0,a2;sin(theta3)*cos(afa2),cos(theta3)*cos(afa2),-sin(afa2),-sin(afa2)*d3;sin(theta3)*sin(afa2),cos(theta3)*sin(afa2),cos(afa2),cos(afa2)*d3;0,0,0,1];
T34=[cos(theta4),-sin(theta4),0,a3;sin(theta4)*cos(afa3),cos(theta4)*cos(afa3),-sin(afa3),-sin(afa3)*d4;sin(theta4)*sin(afa3),cos(theta4)*sin(afa3),cos(afa3),cos(afa3)*d4;0,0,0,1];
T45=[cos(theta5),-sin(theta5),0,a4;sin(theta5)*cos(afa4),cos(theta5)*cos(afa4),-sin(afa4),-sin(afa4)*d5;sin(theta5)*sin(afa4),cos(theta5)*sin(afa4),cos(afa4),cos(afa4)*d5;0,0,0,1];
T56=[cos(theta6),-sin(theta6),0,a5;sin(theta6)*cos(afa5),cos(theta6)*cos(afa5),-sin(afa5),-sin(afa5)*d6;sin(theta6)*sin(afa5),cos(theta6)*sin(afa5),cos(afa5),cos(afa5)*d6;0,0,0,1];
T06=T01*T12*T23*T34*T45*T56;
T16=T12*T23*T34*T45*T56;
T=T06;

% if ~IfAlphabet
%     p=itransf(T)
% end

 
if IfAlphabet
    for i=1:4
        for j=1:4
            fprintf('T(%d,%d)=%s\n',i,j,T(i,j));
        end
    end
end
end