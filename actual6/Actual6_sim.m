%% initialize
clc; close all; clear all;
format shortg; format compact;

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
这个是实体六轴的最终简化版本；比先前版本有所改进；具体改进方法参考大论文 实体六轴部分；
与后来的修正版程序相比，很多计算方法（细节方面）都变了，这个脚本仅供参考吧；
在求解的过程中，选取不同的等式，选取不同的反三角函数，有不同的求解方案，这些方案各有特点，有的更简洁，有的更严谨，
当然，最终得到的都是8组解，只是解的次序略有不同;

代码主要分为两个部分
前半部分是符号推导，对符号变量进行等价替换，以确定参数取值范围；
后半部分是使用随机数进行逆解数值运算，对8组解进行组合，并进行正解验证；

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}
%% 等式组，符号推导
syms th1 th2 th3 th4 th5 th6 d1 d4 d5 d6 a2 a3 nx ox ax px ny oy ay py nz oz az pz;
Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1]; %先前版本命名为Tq，现版本命名为T_end
% T6i = DH_inverse(th6,d6,0,0)
% Tq*T6i %这个是测试O6到O5点的逆变换，验证第1个奇异点；

theta=[th1 th2 th3 th4 th5 th6];
% T=forward_kine_sim(theta)
nx=cos(th6)*(sin(th1)*sin(th5)+cos(th2+th3+th4)*cos(th1)*cos(th5))-sin(th2+th3+th4)*cos(th1)*sin(th6);
ox=-sin(th6)*(sin(th1)*sin(th5)+cos(th2+th3+th4)*cos(th1)*cos(th5))-sin(th2+th3+th4)*cos(th1)*cos(th6);
ax=cos(th5)*sin(th1)-cos(th2+th3+th4)*cos(th1)*sin(th5);
px=d6*(cos(th5)*sin(th1)-cos(th2+th3+th4)*cos(th1)*sin(th5))+d4*sin(th1)+a2*cos(th1)*cos(th2)+d5*sin(th2+th3+th4)*cos(th1)+a3*cos(th1)*cos(th2)*cos(th3)-a3*cos(th1)*sin(th2)*sin(th3);

ny=-cos(th6)*(cos(th1)*sin(th5)-cos(th2+th3+th4)*cos(th5)*sin(th1))-sin(th2+th3+th4)*sin(th1)*sin(th6);
oy=sin(th6)*(cos(th1)*sin(th5)-cos(th2+th3+th4)*cos(th5)*sin(th1))-sin(th2+th3+th4)*cos(th6)*sin(th1);
ay=-cos(th1)*cos(th5)-cos(th2+th3+th4)*sin(th1)*sin(th5);
py=a2*cos(th2)*sin(th1)-d4*cos(th1)-d6*(cos(th1)*cos(th5)+cos(th2+th3+th4)*sin(th1)*sin(th5))+d5*sin(th2+th3+th4)*sin(th1)+a3*cos(th2)*cos(th3)*sin(th1)-a3*sin(th1)*sin(th2)*sin(th3);

nz=cos(th2+th3+th4)*sin(th6)+sin(th2+th3+th4)*cos(th5)*cos(th6);
oz=cos(th2+th3+th4)*cos(th6)-sin(th2+th3+th4)*cos(th5)*sin(th6);
az=-sin(th2+th3+th4)*sin(th5);
pz=d1+d5*(sin(th2+th3)*sin(th4)-cos(th2+th3)*cos(th4))+a3*sin(th2+th3)+a2*sin(th2)-d6*sin(th5)*(cos(th2+th3)*sin(th4)+sin(th2+th3)*cos(th4));
Pz=d5*(sin(th2+th3)*sin(th4)-cos(th2+th3)*cos(th4))+a3*sin(th2+th3)+a2*sin(th2)-d6*sin(th5)*(cos(th2+th3)*sin(th4)+sin(th2+th3)*cos(th4));

simplify(px^2+py^2+Pz^2);
% a2^2 + a3^2 + d4^2 + d5^2 + d6^2 + 2*a3*d5*sin(th4) + 2*a2*a3*cos(th3) + 2*d4*d6*cos(th5) + 2*a2*d5*cos(th3)*sin(th4) + 2*a2*d5*cos(th4)*sin(th3) - 2*a3*d6*cos(th4)*sin(th5) - 2*a2*d6*cos(th3)*cos(th4)*sin(th5) + 2*a2*d6*sin(th3)*sin(th4)*sin(th5)

% d5^2 
% a2^2 + a3^2 + 2*a2*a3*cos(th3)
% d4^2 + d6^2 + 2*d4*d6*cos(th5)
%  
% +2*a3*d5*sin(th4) - 2*a3*d6*cos(th4)*sin(th5)
% +2*a2*d5*sin(th3+th4)-2*a2*d6*cos(th3+th4)*sin(th5)


% 高阶无穷小，当c3==1,th3==0
% +2*a3*d5*sin(th4) - 2*a3*d6*cos(th4)*sin(th5) 
% +2*a2*d5*sin(th4) - 2*a2*d6*cos(th4)*sin(th5)
% % 当s5=1
% +2*a3*d5*sin(th4) - 2*a3*d6*cos(th4)
% +2*a2*d5*sin(th4) - 2*a2*d6*cos(th4)
% 2*(a2+a3)*d5*sin(th4) - 2*(a2+a3)*d6*cos(th4)


% x=simplify(x)

% (d6*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + d4*cos(th1) - a2*cos(th2)*sin(th1) - d5*sin(th2 + th3 + th4)*sin(th1) - a3*cos(th2)*cos(th3)*sin(th1) + a3*sin(th1)*sin(th2)*sin(th3))^2 
% + (d6*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + d4*sin(th1) + a2*cos(th1)*cos(th2) + d5*sin(th2 + th3 + th4)*cos(th1) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3))^2

% 已确定最佳等式组获取方式：T2*T3*T4*T5=T1i*Tend*T6i
%
% cos(th2+th3+th4)*cos(th5) = -sin(th6)*(ox*cos(th1)+oy*sin(th1))+cos(th6)*(nx*cos(th1)+ny*sin(th1))  ①×
% sin(th2+th3+th4)*cos(th5) = nz*cos(th6)-oz*sin(th6)                                                 ②×
%                  sin(th5) = sin(th6)*(oy*cos(th1)-ox*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1))   ③
% 
% sin(th2+th3+th4) = -sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ox*cos(th1)+oy*sin(th1))  ④
% cos(th2+th3+th4) = oz*cos(th6)+nz*sin(th6)                                                 ⑤
%                0 = sin(th6)*(ny*cos(th1)-nx*sin(th1))+cos(th6)*(oy*cos(th1)-ox*sin(th1))   ⑥
%  
% cos(th2+th3+th4)*sin(th5) = -ax*cos(th1)-ay*sin(th1)  ⑦×
% sin(th2+th3+th4)*sin(th5) = -az                       ⑧×
%                  cos(th5) = ax*sin(th1)-ay*cos(th1)   ⑨
% 
% a2*cos(th2)+a3*cos(th2+th3)+d5*sin(th2+th3+th4) = -d6*(ax*cos(th1)+ay*sin(th1))+px*cos(th1)+py*sin(th1)   ⑩
% a2*sin(th2)+a3*sin(th2+th3)-d5*cos(th2+th3+th4) = -d6*az+pz-d1                                            ①①
%                                              d4 =  d6*(ay*cos(th1)-ax*sin(th1))-(py*cos(th1)-px*sin(th1)) ①②

%{
% 其他的反变换尝试
% d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
% d=[d1 0 0 d4 d5 d6];a=[0 a2 a3 0 0 0];
% alpha=[pi/2 0 0 pi/2 -pi/2 0]; %offset=[0 -pi/2 0 -pi/2 0 0];    
% T1i=T_stand_inv(theta(1),d(1),a(1),alpha(1)); %T1 T5 T6 offset=0;
% T6i=T_stand_inv(theta(6),d(6),a(6),alpha(6));
% %T5i=T_stand_inv(theta(5),d(5),a(5),alpha(5));
% Ti=T1i*Tq*T6i  %最终使用  T1i*Tq*T6i==T2*T3*T4*T5==T25
% %Ti=T1i*Tq*T6i*T5i  %T1i*Tq*T6i*T5i==T2*T3*T4==T24;测试用，证明效果一致
%}


m1=d6*ay-py; m1=simplify(m1); % -a2*sin(th1)*cos(th2)-a3*sin(th1)*cos(th2+th3) + d4*cos(th1) - d5*sin(th2 + th3 + th4)*sin(th1) 
n1=d6*ax-px; n1=simplify(n1); % -a2*cos(th1)*cos(th2)-a3*cos(th1)*cos(th2+th3) - d4*sin(th1) - d5*sin(th2 + th3 + th4)*cos(th1)
m6=oy*cos(th1)-ox*sin(th1);  m6=simplify(m6); % sin(th5)*sin(th6)
n6=ny*cos(th1)-nx*sin(th1);  n6=simplify(n6); % -sin(th5)*cos(th6)
    
% m5 = sin(th6)*(oy*cos(th1)-ox*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1));simplify(m5)
n5 = ax*sin(th1)-ay*cos(th1);n5=simplify(n5);
% m23= a2*cos(th2)+a3*cos(th2+th3) % th61 th62 -- th21 th22 th25 th26
% n23= a2*sin(th2)+a3*sin(th2+th3)
% th6=th6+pi;
m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6)*(nx*cos(th1)+ny*sin(th1))+cos(th6)*(ox*cos(th1)+oy*sin(th1)));
n23=pz-d1-d6*az+d5*(nz*sin(th6)+oz*cos(th6));
% simplify(m23) % a2*cos(th2) + a3*cos(th2 + th3) + 2*d5*sin(th2 + th3 + th4) % th63 th64 -- th23 th24 th27 th28
% simplify(n23) % a2*cos(th2) + a3*sin(th2 + th3) - 2*d5*cos(th2 + th3 + th4)

% m3=n23-a2*sin(th2);simplify(m3); %a3*sin(th2 + th3)
% n3=m23-a2*cos(th2);simplify(n3); %a3*cos(th2 + th3)
m234=-sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ox*cos(th1)+oy*sin(th1));
n234=nz*sin(th6)+oz*cos(th6);
% simplify(m234) % sin(th2 + th3 + th4)
% simplify(n234) % cos(th2 + th3 + th4)
% m n对应4种情况
% th11 th61 ①
% th12 th62 ②
% th11 th61+pi ③
% th12 th62+pi ④

%% main 实体六轴，数值计算
% q_deg=[0 -90 0 -90 0 0];
q_deg=[10 20 30 40 50 60]
% q_deg=[0 0 0 0 0 0]
q=q_deg/180*pi;
% q=[pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1)];
T=fkine_UR5(q,6)
% T=trotz(pi/3)*transl(0,0,500)

Q=ikine_UR5(T);
Q_deg=Q*180/pi %print()
for i=1:8
%     eval(['Q',num2str(i),'=','fkine_UR5(Q(i,1:6),6)-T']);
%     T8=fkine_UR5(Q(i,1:6),6)-T;
%     T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4))
    T8=fkine_UR5(Q(i,1:6),6)-T;
    T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4))
end

%% ikine_UR5(T)
function th=ikine_UR5(T)
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
        %奇异点 singularity 可以预自定义

% th1
    m1=d6*ax-px; % m1=N_zero(m1)
    n1=d6*ay-py; % n1=N_zero(n1)
    k1=d4/sqrt(m1^2+n1^2); %工作空间，盲区(dead zone)判定
    if (k1<=1) %工作空间判定
            th1(1)=acos(k1)-atan2(m1,n1);
            th1(2)=-acos(k1)-atan2(m1,n1);
    else
        th1(1:2)=NaN;
    end
    
% th6
    m6=oy*cos(th1)-ox*sin(th1);  m6=N_zero(m6)
    n6=ny*cos(th1)-nx*sin(th1);  n6=N_zero(n6)
    th6(1:2)=atan(-m6./n6); %matlab自动处理0/0为NaN
%     th6(1:2)=atan(-m6./n6);
%     th6(1:2)=atan2(m6,-n6);
    if  (m6(1)==0 && n6(1)==0) %覆盖替换无效数据
        th6(1)=0; %奇异判定1
    end
    if  (m6(2)==0 && n6(2)==0)
        th6(2)=0;
    end
    th6(3:4)=th6(1:2)+pi;
    
% th5 无奇异
%     k5=ax*sin(th1)-ay*cos(th1);
%     th5(1:2)=acos(k5); %尽量用arccos()，默认为正，加负号即可。
%     th5(3:4)=-th5(1:2);
    th1(3:4)=th1(1:2);
    m5=sin(th6).*(oy*cos(th1)-ox*sin(th1))-cos(th6).*(ny*cos(th1)-nx*sin(th1));
    n5=ax*sin(th1)-ay*cos(th1);
    th5=atan2(m5,n5);

% 早版本，先th5 后th6
% 排列组合顺序式唯一的
% sin(th5) = sin(th6)*(oy*cos(th1)-ox*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1))
%     k6=N_zero(sin(th5));
%     m6=oy*cos(th1)-ox*sin(th1);  m6=N_zero(m6);
%     n6=ny*cos(th1)-nx*sin(th1);  n6=N_zero(n6);
%     th6(1:2)=atan(-m6./n6) %matlab自动处理0/0为NaN
%     if k6(1)==0 || (m6(1)==0 && n6(1)==0) %双重判定
%         th6(1)=0;
%     elseif k6(2)==0 || (m6(2)==0 && n6(2)==0)
%         th6(2)=0;
%     end
%     th6(3:4)=th6(1:2)+pi;

% th2 th3 th4

%     m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)))
%     n23=pz-d1-d6*az+d5*(nz*sin(th6)+oz*cos(th6))
%     k2=(m23.^2+n23.^2+a2^2-a3^2)./sqrt((2*a2*m23).^2+(2*a2*n23).^2);
%     th2(1:4)=atan2(n23*a2,m23*a2)+acos(k2); %无奇异，除非a2==a3
%     th2(5:8)=atan2(n23*a2,m23*a2)-acos(k2);
%     
%     m23(5:8)=m23(1:4);
%     n23(5:8)=n23(1:4);
%     m3=(n23-a2*sin(th2))/a3;
%     n3=(m23-a2*cos(th2))/a3;
%     th23=atan2(m3,n3); %无奇异
%     th3=th23-th2;

    
    m=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)));
    n=pz-d1-d6*az+d5*(nz*sin(th6)+oz*cos(th6));
    mn=m.^2+n.^2; %4个不同且无关的值
    
    for i=1:4
        if (a2-a3)^2<=mn(i) && mn(i)<=(a2+a3)^2
            k2=(m(i)^2+n(i)^2+a2^2-a3^2)/sqrt((2*a2*m(i))^2+(2*a2*n(i))^2);
            th2(i)=atan2(n(i)*a2,m(i)*a2)-acos(k2);
            th2(i+4)=atan2(n(i)*a2,m(i)*a2)+acos(k2);
            k3=(m(i)^2+n(i)^2-a2^2-a3^2)/(2*a2*a3);
            th3(i)=acos(k3); %注意使用acos函数求th3时th2 th3唯一对应关系；
            th3(i+4)=-th3(i);
        else
            th2(i)=NaN;
            th2(i+4)=NaN;
            th3(i)=NaN;
            th3(i+4)=NaN;
        end
    end
    
% th4 无奇异
    m234=-sin(th6).*(nx*cos(th1)+ny*sin(th1))-cos(th6).*(ox*cos(th1)+oy*sin(th1));
    n234=nz*sin(th6)+oz*cos(th6);
    th234=atan2(m234,n234);
    th234(5:8)=th234(1:4);
%     th4=th234-th23;
    th4=th234-th3-th2;
    
% sort out;
    th1(5:8)=th1(1:4);
    th5(5:8)=th5(1:4);
    th6(5:8)=th6(1:4);
    for i=1:8
        th(i,1)=th1(i);
        th(i,2)=th2(i);
        th(i,3)=th3(i);
        th(i,4)=In_pi(th4(i));
        th(i,5)=th5(i);
        th(i,6)=In_pi(th6(i));
    end
end





%% Function
function T = fkine_UR5(theta,n) %forward_kine()
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    %d1=127.3;d4=164;d5=116;d6=92;a2=-612;a3=-573; %UR10
    d=[d1 0 0 d4 d5 d6];    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 0];
    for i=1:6
      Te(1:4,1:4,i)=DH_forward(theta(i),d(i),a(i),alpha(i));
    end
    T=eye(4); 
    for j=1:n
        T=T*Te(1:4,1:4,j);
    end 
end


function T = DH_forward(theta,d,a,alpha)
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end
%
function mn = N_zero(mn)
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<1E-10  
                mn(i,j)=0;
            end
        end
    end
end   
% 
function k = K_one(k)
%仅在判断acos()函数参数时使用，th1需要判断以确保在工作空间内；其实为了一个参数的判断不需要单独写一个函数了；
    S=size(k);
    for i=1:S(1)
        for j=1:S(2)
            if abs(k(i,j))>1
                k(i,j)=NaN;
            end
        end
    end
end
% 
function theta = In_pi(theta)
    while (abs(theta)>pi)
        if (theta>pi)
            theta=theta-2*pi;
        elseif (theta<-pi)
            theta=theta+2*pi;
        end
    end
end

