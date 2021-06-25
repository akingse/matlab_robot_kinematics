%% setting
clc;clear;close all;
format short g;format compact;
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
原来的虚拟六轴Virtual6推导化简程序；
初始为z(-y)z版，后修改为zyz版，进行了条件判断，使特殊位姿的求解更加完善；

完善版的程序，更侧重解决奇异点的问题；
易知，奇异点有内部奇异和边界奇异两种，显然，本文逆解解决的是内部奇异，当然，有些时候内部奇异也是边界奇异；
（ps，边界奇异一般需要使用雅可比矩阵求行列式的值为零）
% 逆解（内部）奇异点
    % 无穷量(Inf) %==1/0
    % 不定值量(NaN) %==0/0

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}


%% Virtual6 数值计算
theta_deg=[0 0 0 20 0 40 400*180/pi 50 60]
theta=theta_deg/180*pi;
% theta=[  0            0            0         2.41       0.8579      -1.1062       863.26      0.60959      -1.4757];%论文表格随机数据
% theta=[0 0 0 pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) 1000*rand(1) pi*(2*rand(1)-1) pi*(2*rand(1)-1)];
% theta_deg=theta*180/pi

T=forward_kine49(theta)
theta=ikine_Virtual(T);theta*180/pi;

theta_deg(1:8,4:6)=theta(:,4:6)*180/pi;
theta_deg(1:8,7)=theta(:,7); %单独处理d7
theta_deg(1:8,8:9)=theta(:,8:9)*180/pi;
theta_deg

for i=1:8 %验证
%     eval(['Q',num2str(i),'=','vpa(forward_kine49(theta(i,1:9))-T,6)']);
    T8=forward_kine49(theta(i,1:9))-T;
    T8p=abs(T8(1,4))+abs(T8(2,4))+abs(T8(3,4))
%     eval(['Q',num2str(i),'=','T8p']); %
end


%% function new algorithm
function th=ikine_Virtual(T)
    a8=50;a9=50;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
% th9
    Px=px-a9*nx;
    Py=py-a9*ny;
    Pz=pz-a9*nz;    
    Pm=Px*(az*ny-ay*nz)+Py*(ax*nz-az*nx)+Pz*(ay*nx-ax*ny);  N_zero(Pm);
    Pn=Px*(az*oy-ay*oz)+Py*(ax*oz-az*ox)+Pz*(ay*ox-ax*oy);  N_zero(Pn);

    if Pm==0 && Pn==0  %奇异点1（a8+d7*s8=0）
        th9(1)=0;
    else
        th9(1)=atan(Pm/Pn);
    end
    th9(2)=th9(1)+pi;
%     th9=In_pi(th9);%     th90=th9*180/pi

%% th8
    Rx=nx*cos(th9)-ox*sin(th9);
    Ry=ny*cos(th9)-oy*sin(th9);
    Rz=nz*cos(th9)-oz*sin(th9);

% m8=(ax*(Pz-Rz*a8)-az*(Px-Rx*a8)); %-d7*sin(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6))
% n8=(Rz*(Px-Rx*a8)-Rx*(Pz-Rz*a8)); %-d7*cos(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6))
% m8=(ay*(Pz-Rz*a8)-az*(Py-Ry*a8)); %-d7*sin(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6))
% n8=(Rz*(Py-Ry*a8)-Ry*(Pz-Rz*a8)); %-d7*cos(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6))
% m8=(ax*(Py-Ry*a8)-ay*(Px-Rx*a8)); % d7*sin(th8)*sin(th5)*sin(th6)
% n8=(Ry*(Px-Rx*a8)-Rx*(Py-Ry*a8)); % d7*cos(th8)*sin(th5)*sin(th6)
    m81=ax*(Pz-a8*Rz)-az*(Px-a8*Rx); m81=N_zero(m81);
    n81=Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz); n81=N_zero(n81);
    m82=ay*(Pz-a8*Rz)-az*(Py-a8*Ry); m82=N_zero(m82);
    n82=Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz); n82=N_zero(n82);
    m83=ax*(Py-Ry*a8)-ay*(Px-Rx*a8); m83=N_zero(m83);
    n83=Ry.*(Px-Rx*a8)-Rx.*(Py-Ry*a8); n83=N_zero(n83);
% 数据验证
% th4=20*pi/180; th5=30*pi/180; th6=50*pi/180; th8=50*pi/180; d7=400; %测试验证正确；
% th8=-90*pi/180; d7=2*a8;
% 
% tm81=-d7*sin(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6)) 
% tm82=-(2*a8+d7*sin(th8))*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6))
% 数据验证
% m71(1)==px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9));       %-d7*cos(th4)*sin(th5)
% n71(1)==ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)); %-cos(th4)*sin(th5)
% m71(2)==(d7+2*a8*sin(th8))*cos(th4)*sin(th5) - 2*a8*cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))
% n71(2)==        cos(2*th8)*cos(th4)*sin(th5) +    sin(2*th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))
% theta_deg=[0 0 0 90 30 50 400*180/pi 50 60]
% th4=90*pi/180; th5=30*pi/180; th6=50*pi/180;
% th8=0*pi/180; d7=400;
% 
% tm711=-d7*cos(th4)*sin(th5)
% tn711=-cos(th4)*sin(th5)
% tm712=(d7+2*a8*sin(th8))*cos(th4)*sin(th5) - 2*a8*cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))
% tn712=        cos(2*th8)*cos(th4)*sin(th5) +    sin(2*th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))

    if m81(1)==0&&m81(2)==0 %当m81(1:2)=0，则触发(cos(th4)*cos(th6)-cos(th5)*sin(th4)*sin(th6))=0，必有n81(1:2)=0
% if m81(1)==0&&m81(2)==0 && n81(1)==0&&n81(2)==0 
    %m81==0 ;0数组可以忽略0的个数
        if m82(1)==0&&m82(2)==0
            th8(1:2)=atan(m83./n83);
        else
            th8(1:2)=atan(m82./n82);
        end
    else
        th8(1:2)=atan(m81./n81);
    end
    th8(3:4)=th8(1:2)+pi; %theta8有4个值；
    th9(3:4)=th9(1:2);
%     th8=In_pi(th8)
%     th80=th8*180/pi
%     th82=In_pi(th82);th820=th82*180/pi

%% d7
%     m71=px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9)); m71=N_zero(m71)%    simplify(m71) %-d7*cos(th4)*sin(th5) %Px-a8*Rx;
%     n71=ax*cos(th8) + sin(th8).*(nx*cos(th9)-ox*sin(th9)); n71=N_zero(n71)
%     m72=py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9)); m72=N_zero(m72)%    simplify(m72) %-d7*sin(th4)*sin(th5) %Py-a8*Ry;
%     n72=ay*cos(th8) + sin(th8).*(ny*cos(th9)-oy*sin(th9)); n72=N_zero(n72)
%     m73=pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9)); m73=N_zero(m73)  %Pz-a8*Rz;% simplify(m73) %d7*cos(th5) %Pz-a8*Rz;
%     n73=az*cos(th8) + sin(th8).*(nz*cos(th9)-oz*sin(th9)); n73=N_zero(n73)


    m71=Px-a8*(nx*cos(th9)-ox*sin(th9)); m71=N_zero(m71);
    n71=ax*cos(th8)+sin(th8).*(nx*cos(th9)-ox*sin(th9)); n71=N_zero(n71);
    m72=Py-a8*(ny*cos(th9)-oy*sin(th9)); m72=N_zero(m72);
    n72=ay*cos(th8)+sin(th8).*(ny*cos(th9)-oy*sin(th9)); n72=N_zero(n72);
    m73=Pz-a8*(nz*cos(th9)-oz*sin(th9)); m73=N_zero(m73);
    n73=az*cos(th8)+sin(th8).*(nz*cos(th9)-oz*sin(th9)); n73=N_zero(n73);
    if m71(1)==0
        if m72(1)==0
            d7(1:4)=m73./n73;
        else
            d7(1:4)=m72./n72;
        end
    else
        d7(1:4)=m71./n71
    end
    
%{
%     if m71(1)==0
%         if m72(1)==0
%             d7(1)=m73(1)/n73(1);
%             d7(3)=m73(3)/n73(3);
%         else
%             d7(1)=m72(1)/n72(1);
%             d7(3)=m72(3)/n72(3);
%         end
%     else
%         d7(1)=m71(1)/n71(1);
%         d7(3)=m71(3)/n71(3);
%     end
% % 注意if m71(2)==0 %还需要判断另外一组的解的分母为零的情况，以选择正确的（不为零）分母获得d72；
%     if m71(2)==0
%         if m72(2)==0
%             d7(2)=m72(2)/n72(2);
%             d7(4)=m72(4)/n72(4);
%         else
%             d7(2)=m73(2)/n73(2);
%             d7(4)=m73(4)/n73(4);
%         end
%     else
%         d7(2)=m73(2)/n73(2);
%         d7(4)=m73(4)/n73(4);
%     end
%     

%}

    
%% th4 th5 th6
% sin(th8)*(nz*cos(th9) - oz*sin(th9)) + az*cos(th8)==cos(th5) ==> th5
    n5=az*cos(th8)+sin(th8).*(nz*cos(th9)-oz*sin(th9))
    th5(1:4)=acos(n5);
    th5(5:8)=-th5(1:4)

% 同样的th5只有两个独立值，对应th9(1)和th9(2)，其他都是派生来的；
%  th4
    th9(5:8)=th9(1:4);
    th8(5:8)=th8(1:4);
    d7(5:8)=d7(1:4);
% sin(th8)*(ny*cos(th9) - oy*sin(th9)) + ay*cos(th8)==sin(th4)*sin(th5) % ==>sin4
% sin(th8)*(nx*cos(th9) - ox*sin(th9)) + ax*cos(th8)==cos(th4)*sin(th5) % ==>cos4

    m4=(ay*cos(th8)+sin(th8).*(ny*cos(th9)-oy*sin(th9)))./sin(th5);
    n4=(ax*cos(th8)+sin(th8).*(nx*cos(th9)-ox*sin(th9)))./sin(th5);
    th4=atan2(m4,n4);
    
%                           oz*cos(th9) + nz*sin(th9)==sin(th5)*sin(th6) % ==>sin6
% -cos(th8)*(nz*cos(th9) - oz*sin(th9)) + az*sin(th8)==cos(th6)*sin(th5) % ==>cos6
    m6=(oz*cos(th9)+nz*sin(th9))./sin(th5);
    n6=(az*sin(th8)-cos(th8).*(nz*cos(th9)-oz*sin(th9)))./sin(th5);
    th6=atan2(m6,n6);
% 使用atan2函数，其参数m、n关于th8 th5呈正负相关，atan2中心对称，图像可知相差pi；

    
if N_zero(sin(th5(1)))==0 %覆盖更新原数据
% sin(th5)==0
% -ox*cos(th9) - nx*sin(th9)== cos(th6)*sin(th4) + cos(th4)*sin(th6)==sin(th4+th6)
%  oy*cos(th9) + ny*sin(th9)==-sin(th4)*sin(th6) + cos(th4)*cos(th6)==cos(th4+th6)
    m46=-ox*cos(th9)-nx*sin(th9);
    n46=oy*cos(th9)+ny*sin(th9);
    th46=atan2(m46,n46);
    th4(1)=0; %奇异点
    th4(3)=th4(1)+pi;
% 此时奇异点应该注意的是，th4(1)th4(3)的对应关系，th6(1)th6(3)的对应关系，（这个bug卡住了很久）
% th6()应满足th6(1)+th6(3)=pi，而不是th4(3)+th6(3)=th46(3)
    th6(1)=th46(1)-th4(1);
    th6(3)=pi-th6(1);% th6(3)!=th46(3)-th4(3);
    th4(5)=th4(1)+pi;
    th4(7)=th4(1);
    th6(5)=th46(1)-th4(5);
    th6(7)=pi-th6(5);
end



    for i=1:8
        th(i,9)=In_pi(th9(i));
        th(i,8)=In_pi(th8(i));
        th(i,7)=d7(i);%+1000; %*pi/180;  %被动转角度
        th(i,5)=th5(i);
        th(i,4)=th4(i);
        th(i,6)=th6(i);
    end
end

%% 数据处理，保留3位，四舍五入；
function theta=the_rounding(the)
    a = the;
    n = 4; %保留位数
    b = a*10^n;% 先化成整数，
    a = round(b); %s四舍五入
    need_num = a/10^n; %再转成小数
    need_str = num2str(need_num)
end

%% standard 标准化数据预处理
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

%% kinetic_function

function T = forward_kine49(theta) % virtual六轴专用函数，正运动位姿获取
    %syms  a8 a9 d7; %注意区分zyz和z(-y)z
    a8=50;a9=50; %d7=0;
    d=[0 0 0 0 0 0 0 0 0];      
    a=[0 0 0 0 0 0 0 a8 a9]; 
    alpha=[0 0 0 -pi/2 pi/2 0 pi/2 -pi/2 0]; %zyz
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(0,theta(7),a(7),alpha(7)); 
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T4*T5*T6*T7*T8*T9;
end
function T = DH_forward(theta,d,a,alpha) % 正运动函数
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end

%{
% function T = forward_kine456(theta) % virtual六轴专用函数，正运动位姿获取；
% 后使用d7 th8 th9，就不需要完全求解Virtual6，能有效减少计算过程；
%     d=[0 0 0 0 0 0 0 0 0];     %syms  a8 a9 d7;
% %     a8=50;a9=50; %d7=0;
%     a=[0 0 0 0 0 0 0 50 50]; 
%     alpha=[0 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
%     T4=DH_forward(theta(4),d(4),a(4),alpha(4));
%     T5=DH_forward(theta(5),d(5),a(5),alpha(5));
%     T6=DH_forward(theta(6),d(6),a(6),alpha(6));
%     T7=DH_forward(0,theta(7),a(7),alpha(7)); 
%     T=T4*T5*T6*T7;
% end
%}
