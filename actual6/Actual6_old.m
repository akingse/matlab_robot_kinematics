%% initialize
clc; close all; clear all;
format shortg; format compact;
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
�ɰ��Ƶ����̣�outdated��abandoned��
��Ӧ���Ǻ����Ƚ���ʽ��һ���ˣ���·�Ѿ��߹��ˣ��ܶ����Ѿ������ˣ�
�Ѿ������γ��˹̶��ı�̷�񣬱����������
��滹���õĶ�ά���飬��ʵʹ��һά����͹��ˣ�ע���ά�����ʹ�ã�
�ļ���3���汾��function1��function2����Ҫ�����ǣ�function1�Ľ��õ���һά���飻
���⣬��Ҫ����Щ�������һ���������ڣ��ļ�������ʾ��������20190925���ж���ѧ�ڿ�ʼ��ʱ��
����2ά����汾����Ȼ���㹻���Ƶ����̣����汾���ϣ���ʹ���°�Actual6_sim
����Ҫ��лRobotics Toolbox for MATLAB����лPeter Corke���ڼ����Ŷӣ��ṩ�ļ��ײ�ʧǿ��Ļ����˹����䣻
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

%% main
% theta_deg=[0 -90 0 -90 0 0] %���������λ�ò���
% theta_deg=[0 0 45 0 0 30]
theta_deg=[30 40 50 60 70 80]
% theta_deg=[0 0 0 0 0 90]
% theta_deg=[0 0 0 0 0 0]
% theta_deg=[0 0 45 0 0 0]
% theta_deg=[0 -90 0 -90 0 0]
% theta_deg=[0 30 -30 -90 0 0]
% theta_deg=[0 0 0 180 90 0]
% theta=[pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1) pi*(2*rand(1)-1)]
theta=[      -0.17518     -0.89301       1.9609       1.4129       2.8116     -0.48937] %�����е��������

% theta=theta_deg/180*pi
T=forward_kine(theta) 
Q=inverse_kine(T);
Q_deg=inverse_kine(T)*180/pi

for i=1:8 %��֤
%     eval(['Q',num2str(i),'=','N_infinite0(forward_kine(Q(i,1:6))-T)']);
%     eval(['Q',num2str(i),'=','forward_kine(Q(i,1:6))']);
%     eval(['Q',num2str(i),'=','N_0(forward_kine(Q(i,1:6))-T)']);%����
end


%% inverse_kine�Ƶ�����
function th=inverse_kine(T)
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
   
% th1
%{
%����thʱ�������������ݿ��Ժ������У�Ҳ����12�������У���UR5 GP7 ���汾��Ϊ�������У�����һά�������и�Ϊ��ࣻ
% ��һ��th��벿��ֱ�Ӹ��ƣ����� th1(3:4)=th1(1:2);
% if px^2+py^2+(pz-d1)^2>=(sqrt((a2+a3)^2+d4^2)+sqrt(d5^2+d6^2))^2
%     break;% �����ж�����������Χ��
% end
%}
% d4 = d6*(ay*cos(th1)-ax*sin(th1))-(py*cos(th1)-px*sin(th1)) �٢�
% d4 = (d6*ay-py)*cos(th1)-(d6*ax-px)*sin(th1)); 
% d4=r1*cos(phi1)*cos(th1)-r1*sin(phi1)*sin(th1)); %�������sin(phi-th1)��ʵ�����ø���
% d4 = r1*cos(phi1+th1),phi1=atan2((d6*ax-px)/(d6*ay-py))
%phi1+th1=��acos(d4/r1),th1=��acos(d4/r1)-atan2((d6*ax-px),(d6*ay-py))
m1=d6*ay-py;
n1=d6*ax-px;
if (d4^2<=m1^2+n1^2) % d4^2<=m1^2+n1^2  %�ṹ���µ��˶�ä����
% ��Χ���� d4^2<=(d6*ay-py)^2+(d6*ax-px)^2=> ����������̬��ͨ��px,py��Сֵ��px^2+py^2>=(d4+d6)^2
% ��m1==n1==0,O������z1�����ϣ�z6��z1ͬ�ᣬ��ʱth1��th6��ͬ����rotz6�����ɶȶ�ʧһ�����ǽṹ���ƣ���״̬�����ڣ�
%{
% th1(1,2)=theta(1)+2*atan2(k1,sqrt(1-k1^2))-pi;k1=d4/0;
% d4 = r1*sin(phi)*cos(th1)-r1*cos(phi)*sin(th1)); 
% sin(phi-th1)=d4/r1;  cos(phi-th1)=��sqrt(1-(d4/r1)^2);
    k1=d4/sqrt(m1^2+n1^2); % 0.086<k1<=1;
    th1(1,1)=atan2(m1,n1)-atan2(k1,sqrt(1-k1^2));
    th1(1,2)=atan2(m1,n1)-atan2(k1,-sqrt(1-k1^2));
%}
    k1=d4/sqrt(m1^2+n1^2); % k1=d4/r1
    th1(1,1)=acos(k1)-atan2(n1,m1);
    th1(1,2)=-acos(k1)-atan2(n1,m1);
else
    th1(1,1:2)=NaN;
end
th10=th1*180/pi;  % th10=vpa(th1*180/pi,8);

%% th5
% cos(th5) = ax*sin(th1)-ay*cos(th1) ��
% sin(th5) = ��sqrt(1-(ax*sin(th1)-ay*cos(th1))^2);
k5=ax*sin(th1)-ay*cos(th1); % ����������˻���k5<=1;����Ҫ�ж���
%matlab���������������֤��th5(1,1:2)��Ӧth1(1,1:2);th5(2,1:2)��Ӧth1(1,1:2);
% th5(1,1:2)=atan2(sqrt(1-k5.^2),k5);
% th5(2,1:2)=atan2(-sqrt(1-k5.^2),k5); %-th5(1,1:2);
th5(1,1:2)=acos(k5);
th5(2,1:2)=-th5(1,1:2);
% th5(1,1:2)=acos(ax*sin(th1)-ay*cos(th1));
% th5(2,1:2)=-th5(1:2); 
th50=th5*180/pi;
%{
% �߹�����·��
% sin(th5) = sin(th6)*(oy*cos(th1)-ox*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1))
% k5=sin(th6).*(oy*cos(th1)-ox*sin(th1))-cos(th6).*(ny*cos(th1)-nx*sin(th1));
% th51(1:2,1:2)=atan2(k5,sqrt(1-k5.^2));
% th51(3:4,1:2)=atan2(k5,-sqrt(1-k5.^2));
% ����������th6������th1 th6����th5
% =>th51*180/pi ��ȫ������ cos(th5)=ax*sin(th1)-ay*cos(th1);
% ��Ϊ������ǰ�ӵĸ����������ģ�Ϊ��ǿ�дճ��۵����н⣬�������ǵ�ʽ�۵����н������ʽ�᣻
% ��֮����ʽ������н������ʽ�ۣ�
%}
%% th6
% 0 = sin(th6)*(ny*cos(th1)-nx*sin(th1))+cos(th6)*(oy*cos(th1)-ox*sin(th1)) ��
% sin(th6)*n6+cos(th6)*m6=0;% sin(th6)/cos(th6)=-(m6/n6);%���������ţ�
% ��˻�Ϊ0����ά����th6��mn��ֱ��th6����������Ľ⣻
m6=oy*cos(th1)-ox*sin(th1); % |m6|<=1;
n6=ny*cos(th1)-nx*sin(th1); % |n6|<=1;
% tan(th1)==ny/nx==oy/ox; %sin()������ܵ�������С��Ϊ�㣻
m6=N_0(m6);
n6=N_0(n6);
% if (m6==0)&&(n6==0)  %�����;
% nx=cos(th6)*(sin(th1)*sin(th5)+cos(th2+th3+th4)*cos(th1)*cos(th5))-sin(th2+th3+th4)*cos(th1)*sin(th6)
% ny=-cos(th6)*(cos(th1)*sin(th5)-cos(th2+th3+th4)*cos(th5)*sin(th1))-sin(th2+th3+th4)*sin(th1)*sin(th6)
% ox=-sin(th6)*(sin(th1)*sin(th5)+cos(th2+th3+th4)*cos(th1)*cos(th5))-sin(th2+th3+th4)*cos(th1)*cos(th6)
% oy=sin(th6)*(cos(th1)*sin(th5)-cos(th2+th3+th4)*cos(th5)*sin(th1))-sin(th2+th3+th4)*cos(th6)*sin(th1)
% ��nx,ny,ox,oy���ʽ����n6,m6������ã�
% m6=sin(th5)*sin(th6);
% n6=-sin(th5)*cos(th6);
% if sin(th5)==0; % (th5==0||th5==��pi)����ʱ234��6����ƽ�У�����th6���ɶȶ�ʧ��
% ֻ��th1(1,1)��ʹn6,m6==0���˴�ֻ���ж�th1(1,1)�µ�th6(1:2,1)��
% ԭ���ǣ�ʵ��֤������th1(1,1)��ֵ����nx ny��������Ϊ�㣻
if m6(1,1)==0 && n6(1,1)==0
    th6(1:2,1)=0;%theta(6);
    th6(1,2)=atan2(m6(1,2),-n6(1,2));
    th6(2,2)=atan2(-m6(1,2),n6(1,2));
% elseif m6(1,2)==0 && n6(1,2)==0
%     th6(1:2,2)=theta(6);
%     th6(1,1)=atan2(-m6(1,1),n6(1,1));
%     th6(2,1)=atan2(m6(1,1),-n6(1,1));
else
    th6(1,1:2)=atan2(m6,-n6);
    th6(2,1:2)=atan2(-m6,n6); %th6(1,1:2)-pi;
end
th60=th6*180/pi;

%% th3
% a2*cos(th2)+a3*cos(th2+th3)+d5*sin(th2+th3+th4)= px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1)) ��
% a2*sin(th2)+a3*sin(th2+th3)-d5*cos(th2+th3+th4)= pz-d1-d6*az �٢�
% a3*cos(th2+th3)+a2*cos(th2)= px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6)*(nx*cos(th1)+ny*sin(th1))+cos(th6)*(ox*cos(th1)+oy*sin(th1)))
% a3*sin(th2+th3)+a2*sin(th2)= pz-d1-d6*az+d5*(oz*cos(th6)+nz*sin(th6))
% m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))-d5*sin(th234)
% n23=-d1+pz-d6*az+d5*cos(th234);
m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)));
n23=pz-d1-d6*az+d5*(nz*sin(th6)+oz*cos(th6)); 
% mn�������Ϊ�㣬ˮƽ�򽻲棻
m23=N_0(m23);
n23=N_0(n23);
% r23=sqrt(m23.^2+n23.^2);
% m23^2+n23^2=a2^2+2*cos(th3)*a2*a3+a3^2;��th2=0��piʱ��
% ���ֵa2^2+2*a2*a3+a3^2=(a2+a3)^2����Сֵa2^2-2*a2*a3+a3^2=(a2-a3)^2��
% ���ڽṹ���ƣ�m23^2+n23^2>0�����Բ����� m23==0&&n23==0;�˴������죻
% (a2+a3)^2<=m23^2+n23^2<=(a2+a3)^2
k3=(m23.^2+n23.^2-a2^2-a3^2)/(2*a2*a3);
k3=K_1(k3);
th3(1:2,1:2)=acos(k3);
th3(3:4,1:2)=-th3(1:2,1:2);
% th3(1:2,1:2)=atan2(sqrt(1-k3.^2),k3);
% th3(3:4,1:2)=atan2(-sqrt(1-k3.^2),k3); %-th3(1:2,1:2)
th30=th3*180/pi;
%% th2
% k2=(m3.^2+n3.^2+a2^2-a3^2); %old
% r2=sqrt((2*a2*m3).^2+(2*a2*n3).^2);
% th3m(1:2,1)=th3(1:2,1); th3m(1:2,2)=th3(3:4,1);
% m2=a3*cos(th3m)+a2;n2=a2*sin(th3m);
% r2=sqrt(m2.^2+n2.^2);
% th2(1:2,1:2)=atan2(m3./r2,sqrt(1-(m3./r2).^2))-atan2(n2,m2);
% th2(3:4,1:2)=atan2(m3./r2,-sqrt(1-(m3./r2).^2))-atan2(n2,m2);
% ����û�г�2*a2������Ϊa2Ϊ�����ᵼ��atan2��������Ƕ����䷢���仯��Ӱ��th2����������ϣ�
% �����Ǻ��� sin(a+b)==sina*cosb+cosa*sinb
k2=(m23.^2+n23.^2+a2^2-a3^2)./sqrt((2*a2*m23).^2+(2*a2*n23).^2);
k2=K_1(k2);
th2(1:2,1:2)=atan2(k2,sqrt(1-k2.^2))-atan2(2*a2*m23,2*a2*n23);
th2(3:4,1:2)=atan2(k2,-sqrt(1-k2.^2))-atan2(2*a2*m23,2*a2*n23);
% �����Ǻ��� cos(a-b)==cosa*cosb+sina*sinb
% 2*m*a2*cos(th2)+2*n*a2*sin(th2)==m^2+n^2+a2^2+a3^2;
% r2*cos(phi)=2*m*a2;
% r2*sin(phi)=2*n*a2;
% r2=sqrt((2*m*a2)^2+(2*n*a2)^2);
% cos(phi)*cos(th2)+sin(phi)*sin(th2)==(m^2+n^2+a2^2+a3^2)/r2;
% cos(phi-th2)==(m^2+n^2+a2^2+a3^2)/r2;
% th2(1:4)=atan2(2*n23*a2,2*m23*a2)-acos(k2);
% th2(5:8)=atan2(2*n23*a2,2*m23*a2)+acos(k2);
%     th2(1:4)=atan2(k2,sqrt(1-k2.^2))-atan2(2*a2*m23,2*a2*n23);
%     th2(5:8)=atan2(k2,-sqrt(1-k2.^2))-atan2(2*a2*m23,2*a2*n23);
%     th2(1:4)=atan2(2*n23*a2,2*m23*a2)-acos(k2);
%     th2(5:8)=atan2(2*n23*a2,2*m23*a2)+acos(k2);
th20=th2*180/pi;
%% th4
% sin(th2+th3+th4)=-sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ox*cos(th1)+oy*sin(th1))  ��
% cos(th2+th3+th4)=oz*cos(th6)+nz*sin(th6)  ��
m234=-sin(th6).*(nx*cos(th1)+ny*sin(th1))-cos(th6).*(ox*cos(th1)+oy*sin(th1));
n234=nz*sin(th6)+oz*cos(th6);
m234=N_0(m234);
n234=N_0(n234);
% ֤���ó�����ͬʱ����nx==ny&&ox==oy&&nz==nx*sin(th1)+ny*cos(th1)&&oz==ox*sin(th1)+oy*cos(th1);
% ����ͬʱ����m234=0&&n234==0��th234=th2+th3+th4����������㣻
th234(1:2,1:2)=atan2(m234,n234);
% e1=-d6*sin(th5).*cos(th234)+d5*sin(th234)
% e2=-d6*sin(th5).*sin(th234)-d5*cos(th234)
% sqrt(e1.^2+e2.^2);
th234(3:4,1:2)=th234(1:2,1:2);
th4=th234-th3-th2;
th40=th4*180/pi;

%% sort out;
    th5(3:4,1:2)=th5(1:2,1:2);
    th6(3:4,1:2)=th6(1:2,1:2);
    for i=1:2 
        for j=1:4
            th(4*(i-1)+j,1)=In_pi(th1(1,i)); %th1;
            th(4*(i-1)+j,2)=In_pi(th2(j,i)); %th2;
            th(4*(i-1)+j,3)=N_0(th3(j,i)); %th3;
            th(4*(i-1)+j,4)=In_pi(th4(j,i)); %th4;
            th(4*(i-1)+j,5)=th5(j,i); %th5;
            th(4*(i-1)+j,6)=th6(j,i); %th6;
        end
    end
% theta_deg=th*180/pi;
end

%% function2
% �ؽڽǶ�ά����汾������汾
function th=ikine_UR5_2(T)
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    % th1
    m1=d6*ay-py;
    n1=d6*ax-px;
    if (d4^2<=m1^2+n1^2)
        k1=d4/sqrt(m1^2+n1^2);
        th1(1,1)=atan2(m1,n1)-atan2(k1,sqrt(1-k1^2));
        th1(1,2)=atan2(m1,n1)-atan2(k1,-sqrt(1-k1^2));
    else
        th1(1,1:2)=NaN;
    end
    % th5
    k5=ax*sin(th1)-ay*cos(th1);
    th5(1,1:2)=acos(k5);
    th5(2,1:2)=-th5(1,1:2);
    % th6
    n6=ny*cos(th1)-nx*sin(th1);
    m6=oy*cos(th1)-ox*sin(th1);
    n6=N_zero(n6);
    m6=N_zero(m6);
    if m6(1,1)==0 && n6(1,1)==0 %Singularity��
        th6(1:2,1)=0; %or th6=read(theta_6);
        th6(1,2)=atan2(m6(1,2),-n6(1,2));
        th6(2,2)=th6(1,2)-pi;
    else
        th6(1,1:2)=atan2(m6,-n6);
        th6(2,1:2)=th6(1,1:2)-pi;
    end
%     th1(2,1:2)=th1(1,1:2);
    m23=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)));
    n23=pz-d1-d6*az+d5*(nz*sin(th6)+oz*cos(th6)); 
    m23=N_zero(m23);
    n23=N_zero(n23);
    % th3
    k3=(m23.^2+n23.^2-a2^2-a3^2)/(2*a2*a3);
    k3=K_one(k3);
    th3(1:2,1:2)=atan2(sqrt(1-k3.^2),k3);
    th3(3:4,1:2)=-th3(1:2,1:2);
    % th2
    k2=(m23.^2+n23.^2+a2^2-a3^2)./sqrt((2*a2*m23).^2+(2*a2*n23).^2);
    k2=K_one(k2);
    th2(1:2,1:2)=atan2(k2,sqrt(1-k2.^2))-atan2(2*a2*m23,2*a2*n23);
    th2(3:4,1:2)=atan2(k2,-sqrt(1-k2.^2))-atan2(2*a2*m23,2*a2*n23);
    % th4
    m234=-sin(th6).*(nx*cos(th1)+ny*sin(th1))-cos(th6).*(ox*cos(th1)+oy*sin(th1));
    n234=nz*sin(th6)+oz*cos(th6);
    m234=N_zero(m234);
    n234=N_zero(n234);
    th234(1:2,1:2)=atan2(m234,n234);
    th234(3:4,1:2)=th234(1:2,1:2);
    th4=th234-th3-th2;
    % sort out;
    th5(3:4,1:2)=th5(1:2,1:2);
    th6(3:4,1:2)=th6(1:2,1:2);
    for i=1:2 
        for j=1:4
            th(4*(i-1)+j,1)=In_pi(th1(1,i)); %th1;
            th(4*(i-1)+j,2)=In_pi(th2(j,i)); %th2;
            th(4*(i-1)+j,3)=N_zero(th3(j,i)); %th3;
            th(4*(i-1)+j,4)=In_pi(th4(j,i)); %th4;
            th(4*(i-1)+j,5)=N_zero(th5(j,i)); %th5;
            th(4*(i-1)+j,6)=In_pi(th6(j,i)); %th6;
        end
    end
end
%% inverse_kine_simplify
% ��һ�λ�����ϰ汾����������
function theta=inverse_kine_sim(T)
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;
    nx=T(1,1);ny=T(2,1);nz=T(3,1);
    ox=T(1,2);oy=T(2,2);oz=T(3,2);
    ax=T(1,3);ay=T(2,3);az=T(3,3);
    px=T(1,4);py=T(2,4);pz=T(3,4);
    
    %���ؽڽ�1
    m1=d(6)*ay-py;  n1=ax*d(6)-px; 
    th1(1,1)=atan2(m1,n1)-atan2(d(4),sqrt(m1^2+n1^2-(d(4))^2));
    th1(1,2)=atan2(m1,n1)-atan2(d(4),-sqrt(m1^2+n1^2-(d(4))^2));
%     th10=th1*180/pi
    
    %���ؽڽ�5
    th5(1,1:2)=acos(ax*sin(th1)-ay*cos(th1));
    th5(2,1:2)=-th5(1,1:2);
%     th50=th5*180/pi
    
    %���ؽڽ�6
    m6=nx*sin(th1)-ny*cos(th1); 
    n6=ox*sin(th1)-oy*cos(th1);
    th6=atan2(m6,n6)-atan2(sin(th5),0)
    
    %���ؽڽ�3
    m3=d(5)*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)))-d(6)*(ax*cos(th1)+ay*sin(th1))+px*cos(th1)+py*sin(th1);
    n3=pz-d(1)-az*d(6)+d(5)*(oz*cos(th6)+nz*sin(th6));
    k3=(m3.^2+n3.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3));
%     th3(1:2,1:2)=atan2(sqrt(1-k3.^2),k3);
%     th3(3:4,1:2)=atan2(-sqrt(1-k3.^2),k3);
    th3(1:2,1:2)=acos((m3.^2+n3.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));
    th3(3:4,1:2)=-th3(1:2,1:2);
%     th3(3:4,1:2)=-acos((m3.^2+n3.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));
    %���ؽڽ�2
    m2(1:2,1:2)=m3;    m2(3:4,1:2)=m3;
    n2(1:2,1:2)=n3;    n2(3:4,1:2)=n3;
    
    s2=((a(3)*cos(th3)+a(2)).*n2-a(3)*sin(th3).*m2)./((a(2))^2+(a(3))^2+2*a(2)*a(3)*cos(th3));
    c2=(m2+a(3)*sin(th3).*s2)./(a(3)*cos(th3)+a(2));
    th2=atan2(s2,c2);
    
    %����ؽڽ�1 5 6 3 2
    theta(1:4,1)=th1(1,1);theta(5:8,1)=th1(1,2);
    theta(1:8,2)=[th2(1,1),th2(3,1),th2(2,1),th2(4,1),th2(1,2),th2(3,2),th2(2,2),th2(4,2)]';
    theta(1:8,3)=[th3(1,1),th3(3,1),th3(2,1),th3(4,1),th3(1,2),th3(3,2),th3(2,2),th3(4,2)]';
    theta(1:2,5)=th5(1,1);theta(3:4,5)=th5(2,1);
    theta(5:6,5)=th5(1,2);theta(7:8,5)=th5(2,2);
    theta(1:2,6)=th6(1,1);theta(3:4,6)=th6(2,1);
    theta(5:6,6)=th6(1,2);theta(7:8,6)=th6(2,2);
    
    %���ؽڽ�4
    theta(:,4)=atan2(-sin(theta(:,6)).*(nx*cos(theta(:,1))+ny*sin(theta(:,1)))-cos(theta(:,6)).* (ox*cos(theta(:,1))+oy*sin(theta(:,1))),oz*cos(theta(:,6))+nz*sin(theta(:,6)))-theta(:,2)-theta(:,3);  
end
%{
% function theta=inverse_kinematics(T)
% �����м��һ������İ汾��Ŀ��ȽϹ��ϣ�
%     %�任����T��֪
%     %SDH:��׼DH����������⣨�����⣩����DH���������£���Ҫ���theta��Ϣ
%     a=[0,-0.42500,-0.39225,0,0,0];
%     d=[0.089159,0,0,0.10915,0.09465,0.08230];
%     alpha=[pi/2,0,0,pi/2,-pi/2,0];% alphaû���õ�,�ʴ�������ֻ�ʺ�alpha=[pi/2,0,0,pi/2,-pi/2,0]�������
%     
%     nx=T(1,1);ny=T(2,1);nz=T(3,1);
%     ox=T(1,2);oy=T(2,2);oz=T(3,2);
%     ax=T(1,3);ay=T(2,3);az=T(3,3);
%     px=T(1,4);py=T(2,4);pz=T(3,4);
%     
%     %���ؽڽ�1
%     m=d(6)*ay-py;  n=ax*d(6)-px; 
%     theta1(1,1)=atan2(m,n)-atan2(d(4),sqrt(m^2+n^2-(d(4))^2));
%     theta1(1,2)=atan2(m,n)-atan2(d(4),-sqrt(m^2+n^2-(d(4))^2));
%   
%     %���ؽڽ�5
%     theta5(1,1:2)=acos(ax*sin(theta1)-ay*cos(theta1));
%     theta5(2,1:2)=-acos(ax*sin(theta1)-ay*cos(theta1));      
%     
%     %���ؽڽ�6
%     mm=nx*sin(theta1)-ny*cos(theta1); nn=ox*sin(theta1)-oy*cos(theta1);
%     theta6=atan2(mm,nn)-atan2(sin(theta5),0);
%     
%     %���ؽڽ�3
%     mmm=d(5)*(sin(theta6).*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6).*(ox*cos(theta1)+oy*sin(theta1))) ...
%         -d(6)*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1);
%     nnn=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta6)+nz*sin(theta6));
%     theta3(1:2,:)=acos((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));
%     theta3(3:4,:)=-acos((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));
%     
%     %���ؽڽ�2
%     mmm_s2(1:2,:)=mmm;
%     mmm_s2(3:4,:)=mmm;
%     nnn_s2(1:2,:)=nnn;
%     nnn_s2(3:4,:)=nnn;
%     s2=((a(3)*cos(theta3)+a(2)).*nnn_s2-a(3)*sin(theta3).*mmm_s2)./ ...
%         ((a(2))^2+(a(3))^2+2*a(2)*a(3)*cos(theta3));
%     c2=(mmm_s2+a(3)*sin(theta3).*s2)./(a(3)*cos(theta3)+a(2));
%     theta2=atan2(s2,c2);   
%     
%     %����ؽڽ�1 5 6 3 2
%     theta(1:4,1)=theta1(1,1);theta(5:8,1)=theta1(1,2);
%     theta(:,2)=[theta2(1,1),theta2(3,1),theta2(2,1),theta2(4,1),theta2(1,2),theta2(3,2),theta2(2,2),theta2(4,2)]';
%     theta(:,3)=[theta3(1,1),theta3(3,1),theta3(2,1),theta3(4,1),theta3(1,2),theta3(3,2),theta3(2,2),theta3(4,2)]';
%     theta(1:2,5)=theta5(1,1);theta(3:4,5)=theta5(2,1);
%     theta(5:6,5)=theta5(1,2);theta(7:8,5)=theta5(2,2);
%     theta(1:2,6)=theta6(1,1);theta(3:4,6)=theta6(2,1);
%     theta(5:6,6)=theta6(1,2);theta(7:8,6)=theta6(2,2); 
%     
%     %���ؽڽ�4
%     theta(:,4)=atan2(-sin(theta(:,6)).*(nx*cos(theta(:,1))+ny*sin(theta(:,1)))-cos(theta(:,6)).* ...
%         (ox*cos(theta(:,1))+oy*sin(theta(:,1))),oz*cos(theta(:,6))+nz*sin(theta(:,6)))-theta(:,2)-theta(:,3);  
%     
% end
%}

%% kinematics
function T = forward_kine(theta)% ���˶�
    d1=90;d4=90;d5=90;d6=90;a2=-420;a3=-400;%syms d1 d4 d5 d6 a2 a3; 
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
%     T=simplify(T1*T2*T3*T4*T5*T6);
end

function T = forward_kine_sim(theta)% ���˶�
    syms d1 d4 d5 d6 a2 a3; 
    d=[d1 0 0 d4 d5 d6];
    a=[0 a2 a3 0 0 0];
    alpha=[pi/2 0 0 pi/2 -pi/2 0];
    
    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T=simplify(T1*T2*T3*T4*T5*T6);
end

function T = DH_forward(theta,d,a,alpha) %DH_forward()
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end

function T = DH_inverse(theta,d,a,alpha) %DH_inverse()
    T=invtrot(trotx(alpha))*invtrans(transl(a,0,0))*invtrot(trotz(theta))*invtrans(transl(0,0,d));
end


%% function
function theta = In_pi(theta) %united pi,ͳһ�޶�theta��[-pi,pi]
    while (abs(theta)>pi)
        if (theta>pi)
            theta=theta-2*pi;
        elseif (theta<-pi)
            theta=theta+2*pi;
        end
    end
end

function [Ti]=invtrot(T)
    for i=1:2
        for j=i+1:3
            temp=T(i,j);
            T(i,j)=T(j,i);
            T(j,i)=temp;
        end
    end
    Ti=T;
end
function [Ti]=invtrans(T)
    for i=1:3
        T(i,4)=-T(i,4);
    end
    Ti=T;
end

function mn = N_0(mn) %N_infinite0,recorrect bug,limit 1E-10=0��
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<1E-10  
                mn(i,j)=0;
            end
        end
    end
end   

function k = K_1(k) %N_greater1,if the k(i,j)>1,k(i,j)=NaN;
    S=size(k);
    for i=1:S(1)
        for j=1:S(2)
            if abs(k(i,j))>1
                k(i,j)=NaN;
            end
        end
    end
end



%% offset version
% -cos(th2+th3+th4+th5)/2-cos(th2+th3+th4-th5)/2=cos(th6)*(nx*cos(th1)+ny*sin(th1))-sin(th6)*(ox*cos(th1)+oy*sin(th1)  ��
% -sin(th2+th3+th4-th5)/2-sin(th2+th3+th4+th5)/2= nz*cos(th6)-oz*sin(th6)  ��
% sin(th5)= sin(th6)*(oy*cos(th1)-ox*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1))  ��
% 
% sin(th2+th3+th4)= sin(th6)*(nx*cos(th1)+ny*sin(th1))+cos(th6)*(ox*cos(th1)+oy*sin(th1))  ��
% -cos(th2+th3+th4)= oz*cos(th6)+nz*sin(th6)  ��
% 0= -sin(th6)*(ny*cos(th1)-nx*sin(th1))-cos(th6)*(oy*cos(th1)-ox*sin(th1))  ��
%  
% sin(th2+th3+th4+th5)/2-sin(th2+th3+th4-th5)/2= ax*cos(th1)+ay*sin(th1)  ��
% cos(th2+th3+th4-th5)/2-cos(th2+th3+th4+th5)/2 = az  ��
% cos(th5)= ax*sin(th1)-ay*cos(th1)  ��
% 
% a3*sin(th2+th3)+a2*sin(th2)-d5*sin(th2+th3+th4)= px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1)) ��
% d5*cos(th2+th3+th4)-a2*cos(th2)-a3*cos(th2+th3)= pz-d1-az*d6  �٢�
% d4= d6*(ay*cos(th1)-ax*sin(th1))-py*cos(th1)+px*sin(th1)  �٢�
% -------------------------------------------------------------------------
% % th1
% % d4 = d6*(ay*cos(th1)-ax*sin(th1))-py*cos(th1)+px*sin(th1) �٢�
% % d4 = (d6*ay-py)*cos(th1)-(d6*ax-px)*sin(th1)); 
% % if (d6*ay-py==0)and(d6*ax-px==0) %�����1
% % px=0,py=0,az=1��O������z1�����ϣ�z6��z1ͬ�ᣬ��ʱth1��th6��ͬ����rotz6��
% % ������������ɶȶ�ʧһ�����޶�th1������ֻ��th6��
% % d4 = r1*sin(phi)*cos(th1)-r1*cos(phi)*sin(th1)); 
% % sin(phi-th1)=d4/r1;  cos(phi-th1)=��sqrt(1-(d4/r1)^2);
% r1=sqrt((d6*ay-py)^2+(d6*ax-px)^2);
% th1(1,1)=atan2(d6*ay-py,d6*ax-px)-atan2(d4/r1,sqrt(1-(d4/r1)^2));
% th1(1,2)=atan2(d6*ay-py,d6*ax-px)-atan2(d4/r1,-sqrt(1-(d4/r1)^2));
% % th10=vpa(th1*180/pi,8);
% th10=th1*180/pi;
% 
% % th5
% % cos(th5) = ax*sin(th1)-ay*cos(th1) ��
% % sin(th5) = ��sqrt(1-(ax*sin(th1)-ay*cos(th1))^2);
% % th5(1,1:2)=atan2(sqrt(1-(ax*sin(th1)-ay*cos(th1))^2),ax*sin(th1)-ay*cos(th1));
% % th5(2,1:2)=atan2(-sqrt(1-(ax*sin(th1)-ay*cos(th1))^2),ax*sin(th1)-ay*cos(th1));
% % if (ax*sin(th1)-ay*cos(th1)<=1) �����2
% th5(1,1:2)=acos(ax*sin(th1)-ay*cos(th1));
% th5(2,1:2)=-th5(1:2); 
% 
% % th6
% % 0 = -sin(th6)*(ny*cos(th1)-nx*sin(th1))-cos(th6)*(oy*cos(th1)-ox*sin(th1)) ��
% % sin(th6)*m6+cos(th6)*n6=0;% sin(th6+phi6)=0;%���������ţ�
% m6=nx*sin(th1)-ny*cos(th1); n6=ox*sin(th1)-oy*cos(th1);
% % if (m6==0)and(n6==0)  %�����3,��nz=1,oz=1,�����β����ڣ�
% % th6(1:2)=pi-atan2(n6,m6);
% % th6(3:4)=0-atan2(n6,m6);
% th6(1,1:2)=atan2(-n6,m6);
% th6(2,1:2)=atan2(n6,-m6);
% 
% % th2 th3 th4
% % a3*sin(th2+th3)+a2*sin(th2)-d5*sin(th2+th3+th4) = px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1)) ��
% % d5*cos(th2+th3+th4)-a2*cos(th2)-a3*cos(th2+th3) = pz-d1-az*d6 �٢�
% % a3*sin(th2+th3)+a2*sin(th2) = px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6)*(nx*cos(th1)+ny*sin(th1))+cos(th6)*(ox*cos(th1)+oy*sin(th1)))
% % a3*cos(th2+th3)+a2*cos(th2) = d1+az*d6-pz-d5*(oz*cos(th6)+nz*sin(th6))
% m3=px*cos(th1)+py*sin(th1)-d6*(ax*cos(th1)+ay*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)));
% n3=d1+az*d6-pz-d5*(oz*cos(th6)+nz*sin(th6));
% 
% k3=(m3.^2+n3.^2-a2^2-a3^2)/(2*a2*a3);
% % th3(1:2,1:2)=atan2(sqrt(1-k3.^2),k3);
% % th3(3:4,1:2)=atan2(-sqrt(1-k3.^2),k3);
% th3(1:2,1:2)=acos(k3);
% th3(3:4,1:2)=-th3(1:2,1:2);
% % ------------------------------------------------------------------------
% 
% k2=(m3.^2+n3.^2+a2^2-a3^2)/2;
% % a2*m2*sin(th2)+a2*n2*cos(th2)=k2;
% r2=sqrt((a2*m3).^2+(a2*n3).^2);
% % th3m(1:2,1)=th3(1:2,1); th3m(1:2,2)=th3(3:4,1);
% % m2=a3*cos(th3m)+a2;n2=a2*sin(th3m);
% % r2=sqrt(m2.^2+n2.^2);
% % th2(1:2,1:2)=atan2(m3./r2,sqrt(1-(m3./r2).^2))-atan2(n2,m2);
% % th2(3:4,1:2)=atan2(m3./r2,-sqrt(1-(m3./r2).^2))-atan2(n2,m2);
% % �㷨�����ݣ������Ǻ�������˾���
% % th2(1:2,1:2)=-atan2(k2./r2,sqrt(1-(k2./r2).^2))+atan2(a2*m3,a2*n3);
% % th2(3:4,1:2)=-atan2(k2./r2,-sqrt(1-(k2./r2).^2))+atan2(a2*m3,a2*n3);
% th2(1:2,1:2)=atan2(k2./r2,sqrt(1-(k2./r2).^2))-atan2(a2*n3,a2*m3)-pi/2;
% th2(3:4,1:2)=atan2(k2./r2,-sqrt(1-(k2./r2).^2))-atan2(a2*n3,a2*m3)-pi/2;
% % (a3*cos(th3)+a2)*sin(th2)+a3*sin(th3)*cos(th2)=m3;
% % (a3*cos(th3)+a2)*cos(th2)-a3*sin(th3)*sin(th2)=n3;
% % AB=[a3*cos(th3)+a2 a3*sin(th3);a3*cos(th3)+a2 -a3*sin(th3)];
% % [m2; n2]=inv(AB)*[m3; n3];
% 
% % m2(1:2,1:2)=m3;    m2(3:4,1:2)=m3;% ������չ
% % n2(1:2,1:2)=n3;    n2(3:4,1:2)=n3;
% % km2=(m2.*(a3*cos(th3)+a2)-n2.*(a3*sin(th3)));%/((a3*cos(th3)+a2).^2+(a3*sin(th3)).^2);
% % kn2=(n2.*(a3*cos(th3)+a2)+m2.*(a3*sin(th3)));%/((a3*cos(th3)+a2).^2+(a3*sin(th3)).^2);
% % th2=atan2(km2,kn2)-pi/2;
% 
% % sin(th2+th3+th4) = sin(th6)*(nx*cos(th1)+ny*sin(th1))+cos(th6)*(ox*cos(th1)+oy*sin(th1)) ��
% % -cos(th2+th3+th4) = oz*cos(th6)+nz*sin(th6) ��
% thsum(1:2,1:2)=acos(oz*cos(th6)+nz*sin(th6));
% % thsum(3:4,1:2)=-acos(oz*cos(th6)+nz*sin(th6));
% th234(1,1:2)=thsum(1,1:2);th234(2,1:2)=-thsum(2,1:2); %����
% th234(3:4,1:2)=th234(1:2,1:2);
% 
% % thsum=atan2(sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ox*cos(th1)+oy*sin(th1)),-oz*cos(th6)-nz*sin(th6));
% % th234(1,1:2)=thsum(2,1:2);th234(2,1:2)=thsum(1,1:2); %����
% % th234(3:4,1:2)=th234(1:2,1:2);
% th4=th234-th3-th2;

%% old version
% original �ɰ汾
% % L1 = Link('d', 0.089159, 'a', 0,        'alpha', pi/2 ,'standard' );
% % L2 = Link('d', 0,        'a', -0.42500, 'alpha',   0  ,'standard' );
% % L3 = Link('d', 0,        'a', -0.39225, 'alpha',   0  ,'standard' );
% % L4 = Link('d', 0.10915,  'a', 0,        'alpha', pi/2 ,'standard' );
% % L5 = Link('d', 0.09465,  'a', 0,        'alpha', -pi/2 ,'standard');
% % L6 = Link('d', 0.08230,  'a', 0,        'alpha',   0   ,'standard');
% % robot_UR5=SerialLink([L1,L2,L3,L4,L5,L6],'name','UR5'); 
% % robot_UR5.display();  
% % robot_UR5.teach(); 
% % A=fkine(robot_UR5,theta);
% % B=ikine(robot_UR5,A);
% 
% function T = forward_kinematics(theta)% ���˶�ѧ
%     %��֪�ؽڽ���任����
%     a=[0,-0.42500,-0.39225,0,0,0];
%     d=[0.089159,0,0,0.10915,0.09465,0.08230];
%     alpha=[pi/2,0,0,pi/2,-pi/2,0];
%  
%     T01=T_para(theta(1),d(1),a(1),alpha(1));
%     T12=T_para(theta(2),d(2),a(2),alpha(2));
%     T23=T_para(theta(3),d(3),a(3),alpha(3));
%     T34=T_para(theta(4),d(4),a(4),alpha(4));
%     T45=T_para(theta(5),d(5),a(5),alpha(5));
%     T56=T_para(theta(6),d(6),a(6),alpha(6));
%     
%     T=T01*T12*T23*T34*T45*T56;
% end
% 
% function T = T_para(theta,d,a,alpha)
%     T=[ccc(theta),-sss(theta)*ccc(alpha),sss(theta)*sss(alpha),a*ccc(theta);
%         sss(theta),ccc(theta)*ccc(alpha),-ccc(theta)*sss(alpha),a*sss(theta);
%         0,sss(alpha),ccc(alpha),d;
%         0,0,0,1];
% end

% function T = forward_kine(para)% ���˶�
%     d=[0 para(2) 0 0 para(5) 20];
%     a=[0 0 20 0 0 0];
%     alpha=[-pi/2 0 pi/2 pi/2 0 0];
%     offset=[pi/2 0 0 0 0 0];
%  
%     T1=T_standard(para(1)+offset(1),d(1),a(1),alpha(1));
%     T2=T_standard(0,d(2),a(2),alpha(2));
%     T3=T_standard(para(3),d(3),a(3),alpha(3));
%     T4=T_standard(para(4),d(4),a(4),alpha(4));
%     T5=T_standard(0,d(5),a(5),alpha(5));
%     T6=T_standard(para(6),d(6),a(6),alpha(6));
%     T=troty(pi/2)*T1*T2*T3*T4*T5*T6;
% end



 