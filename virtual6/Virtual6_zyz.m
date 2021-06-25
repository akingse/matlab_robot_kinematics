%% setting
clc;clear;close all;
format short g;format compact;

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
�˳������Toolbox��ģ��չʾ��
%% zyz
% ֮ǰ��DH�������ô��󣬵��²�����������zyz��ʽ��ŷ�������У�����z(-y)z����������һ�棻
% zyzӦ�����㣺T4*T5*T6==Rotz(th4)*Roty(th5)*Rotz(th6)
% �������Ჿ�ֵ�Rotx(alpha)��th4,th5,th6��ԭ���� [pi/2 -pi/2 0]���ָ�Ϊ [pi/2 -pi/2 0]���������䣻
% ��ʵ֤��������th4��th6����������ܵ�Ӱ�죬��DOF9��Ӱ�죨��Ҫ��ʹ��d7 th8 th9����ȡʵ������Ĳ�����
% ��Ȼ�������������ز����ᱻӰ�죬����noap���ʽ�������Ҫ����noap�ı������ſɱȾ��������

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

%% Toolbox model
theta=[0 0 0 0 0 0]; %��Z 
% d=[0 0 -1000 0 0 0]; %��Z 
d=[0 0 0 0 0 0]; %��Z 
a=[0 0 0 0 50 50]; %��X  % a=[0 420 400 0 0 0]; 
% alpha=[pi/2 -pi/2 0 pi/2 -pi/2 0]; %��X
% ԭz(-y)z DH��
% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|          0|          0|     1.5708|          0|
% |  2|         q2|          0|          0|    -1.5708|          0|
% |  3|         q3|          0|          0|          0|          0|
% |  4|          0|         q4|          0|     1.5708|          0|
% |  5|         q5|          0|         50|    -1.5708|          0|
% |  6|         q6|          0|         50|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+

% ��zyz DH��
alpha=[-pi/2 pi/2 0 pi/2 -pi/2 0];
% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|          0|          0|    -1.5708|          0|
% |  2|         q2|          0|          0|     1.5708|          0|
% |  3|         q3|          0|          0|          0|          0|
% |  4|          0|         q4|          0|     1.5708|          0|
% |  5|         q5|          0|         50|    -1.5708|          0|
% |  6|         q6|          0|         50|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+
L(1) = Link([theta(1) d(1) a(1) alpha(1) 0],0);
L(2) = Link([theta(2) d(2) a(2) alpha(2) 0],0);
L(3) = Link([theta(3) d(3) a(3) alpha(3) 0],0);
L(4) = Link([theta(4) d(4) a(4) alpha(4) 1],0);  L(4).qlim = [0 2000];
L(5) = Link([theta(5) d(5) a(5) alpha(5) 0],0); 
L(6) = Link([theta(6) d(6) a(6) alpha(6) 0],0);
robot_V6=SerialLink( L,'name','V6'); 
robot_V6.display(); % ʹ��robot_V6.display() ���DH��
robot_V6.teach(); 

% robot_V6.teach(theta(3,4:7)); 
% robot_V6.teach(theta(8,4:9));  %�������ͼ��
% robot_V6.teach(theta(2,4:9)); 
        
%% ��֤zyz
syms th1 th2 th3 th4 th5 th6 d7 th8 th9;
syms d a theta alpha th;
% d=0; a=0; alpha=pi/2;
% �����˶�ѧ����
% DH_forward(th,d,a,alpha)% �ȶ�
% SDH_forward(th,d,a,alpha)%���ȶ�
% DH_inverse(th,d,a,alpha)%�ȶ�
% SDH_inverse(th,d,a,alpha)%���ȶ�
% forward_kine49(theta)

theta=[th1 th2 th3 th4 th5 th6 d7 th8 th9]; %�ؽ��������
% �鿴noap
forward_kine_zyz(theta)
% �򿪷ֺŲ鿴���
T=DH_forward(th4,0,0,-pi/2)*DH_forward(th5,0,0,pi/2)*DH_forward(th6,0,0,0);
zyz=trotz(th4)*troty(th5)*trotz(th6);
% answer=
% [-sin(th4)*sin(th6) + cos(th4)*cos(th5)*cos(th6), - sin(th4)*cos(th6) - cos(th4)*cos(th5)*sin(th6), cos(th4)*sin(th5), 0]
% [ cos(th4)*sin(th6) + sin(th4)*cos(th5)*cos(th6),   cos(th4)*cos(th6) - sin(th4)*cos(th5)*sin(th6), sin(th4)*sin(th5), 0]
% [                             -sin(th5)*cos(th6),                                sin(th5)*sin(th6),          cos(th5), 0]
% [                                              0,                                                0,                 0, 1]
T-zyz;



%% T6*T7*T8*T9=T5i*T4i*Tp
% ʹ��RRPRRR��DH�任��ʹ��d6�ƶ��ᣬ������d7���൱�ڽ����˱任˳����ʵ֤�����������һ�£�������˵ľ�����õ��ĵ�ʽ�飻
syms th7 d6;
theta=[th1 th2 th3 th4 th5 d6 th7 th8 th9]; %�ؽ��������
% �鿴noap
forward_kine_RRP(theta);
T=equality_RRP(theta);

% Ŀ��Ƚ��ѽ�
% nx*cos(th4)*cos(th5)-nz*sin(th5)+ny*cos(th5)*sin(th4)=cos(th7)*cos(th8)*cos(th9)-sin(th7)*sin(th9)
%                               ny*cos(th4)-nx*sin(th4)=cos(th7)*sin(th9)+cos(th8)*cos(th9)*sin(th7)
% nz*cos(th5)+nx*cos(th4)*sin(th5)+ny*sin(th4)*sin(th5)=cos(th9)*sin(th8)
% 
% ox*cos(th4)*cos(th5)-oz*sin(th5)+oy*cos(th5)*sin(th4)=-cos(th9)*sin(th7)-cos(th7)*cos(th8)*sin(th9)
%                               oy*cos(th4)-ox*sin(th4)=cos(th7)*cos(th9)-cos(th8)*sin(th7)*sin(th9)
% oz*cos(th5)+ox*cos(th4)*sin(th5)+oy*sin(th4)*sin(th5)=-sin(th8)*sin(th9)
% 
% ax*cos(th4)*cos(th5)-az*sin(th5)+ay*cos(th5)*sin(th4)=-cos(th7)*sin(th8)
%                               ay*cos(th4)-ax*sin(th4)=-sin(th7)*sin(th8)
% az*cos(th5)+ax*cos(th4)*sin(th5)+ay*sin(th4)*sin(th5)=cos(th8)
% 
% px*cos(th4)*cos(th5)-pz*sin(th5)+py*cos(th5)*sin(th4)=a8*cos(th7)*cos(th8)-a9*sin(th7)*sin(th9)+a9*cos(th7)*cos(th8)*cos(th9)
%                               py*cos(th4)-px*sin(th4)=a8*cos(th8)*sin(th7)+a9*cos(th7)*sin(th9)+a9*cos(th8)*cos(th9)*sin(th7)
% pz*cos(th5)+px*cos(th4)*sin(th5)+py*sin(th4)*sin(th5)=d6+a8*sin(th8)+a9*cos(th9)*sin(th8)


%% RRPRRR ��ʽ��,d6��ʽ
% equality(theta)
% Tp*T9i*T8i
% [ cos(th8)*(nx*cos(th9) - ox*sin(th9)) - ax*sin(th8), sin(th8)*(nx*cos(th9) - ox*sin(th9)) + ax*cos(th8), - ox*cos(th9) - nx*sin(th9), px - a9*nx - a8*(nx*cos(th9) - ox*sin(th9))]
% [ cos(th8)*(ny*cos(th9) - oy*sin(th9)) - ay*sin(th8), sin(th8)*(ny*cos(th9) - oy*sin(th9)) + ay*cos(th8), - oy*cos(th9) - ny*sin(th9), py - a9*ny - a8*(ny*cos(th9) - oy*sin(th9))]
% [ cos(th8)*(nz*cos(th9) - oz*sin(th9)) - az*sin(th8), sin(th8)*(nz*cos(th9) - oz*sin(th9)) + az*cos(th8), - oz*cos(th9) - nz*sin(th9), pz - a9*nz - a8*(nz*cos(th9) - oz*sin(th9))]
% T=T4*T5*T6*T7;
% [ cos(th4)*cos(th5)*cos(th7) - sin(th4)*sin(th7), cos(th4)*sin(th5), cos(th7)*sin(th4) + cos(th4)*cos(th5)*sin(th7), d6*cos(th4)*sin(th5)]
% [ cos(th4)*sin(th7) + cos(th5)*cos(th7)*sin(th4), sin(th4)*sin(th5), cos(th5)*sin(th4)*sin(th7) - cos(th4)*cos(th7), d6*sin(th4)*sin(th5)]
% [                             -cos(th7)*sin(th5),          cos(th5),                             -sin(th5)*sin(th7),          d6*cos(th5)]
% -------------------------------------------------------------------------
% �� Tp*T9i*T8i==T4*T5*T6*T7;
% cos(th8)*(nx*cos(th9) - ox*sin(th9)) - ax*sin(th8)==cos(th4)*cos(th5)*cos(th6) - sin(th4)*sin(th6)
% cos(th8)*(ny*cos(th9) - oy*sin(th9)) - ay*sin(th8)==sin(th4)*cos(th5)*cos(th6) + cos(th4)*sin(th6)
% cos(th8)*(nz*cos(th9) - oz*sin(th9)) - az*sin(th8)==-cos(th6)*sin(th5)
% 
% sin(th8)*(nx*cos(th9) - ox*sin(th9)) + ax*cos(th8)==cos(th4)*sin(th5)
% sin(th8)*(ny*cos(th9) - oy*sin(th9)) + ay*cos(th8)==sin(th4)*sin(th5)
% sin(th8)*(nz*cos(th9) - oz*sin(th9)) + az*cos(th8)==cos(th5)
% 
% - ox*cos(th9) - nx*sin(th9)==cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6)
% - oy*cos(th9) - ny*sin(th9)==cos(th5)*sin(th4)*sin(th6) - cos(th4)*cos(th6)
% - oz*cos(th9) - nz*sin(th9)==-sin(th5)*sin(th6)
% 
% px - a9*nx - a8*(nx*cos(th9) - ox*sin(th9))==d7*cos(th4)*sin(th5)
% py - a9*ny - a8*(ny*cos(th9) - oy*sin(th9))==d7*sin(th4)*sin(th5)
% pz - a9*nz - a8*(nz*cos(th9) - oz*sin(th9))==d7*cos(th5)

% zyz��T4*T5*T6*T7
% [ cos(th4)*cos(th5)*cos(th6) - sin(th4)*sin(th6), cos(th4)*sin(th5), cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6), d7*cos(th4)*sin(th5)]
% [ cos(th4)*sin(th6) + cos(th5)*cos(th6)*sin(th4), sin(th4)*sin(th5), cos(th5)*sin(th4)*sin(th6) - cos(th4)*cos(th6), d7*sin(th4)*sin(th5)]
% [                             -cos(th6)*sin(th5),          cos(th5),                             -sin(th5)*sin(th6),          d7*cos(th5)]
% [                                              0,                 0,                                              0,                    1]
% zyx��T4*T5*T6*T7
% [ cos(th4)*cos(th5), sin(th4)*sin(th6) + cos(th4)*cos(th6)*sin(th5),   cos(th6)*sin(th4) - cos(th4)*sin(th5)*sin(th6),  d7*(sin(th4)*sin(th6) + cos(th4)*cos(th6)*sin(th5))]
% [ cos(th5)*sin(th4), cos(th6)*sin(th4)*sin(th5) - cos(th4)*sin(th6), - cos(th4)*cos(th6) - sin(th4)*sin(th5)*sin(th6), -d7*(cos(th4)*sin(th6) - cos(th6)*sin(th4)*sin(th5))]
% [         -sin(th5),                              cos(th5)*cos(th6),                               -cos(th5)*sin(th6),                                 d7*cos(th5)*cos(th6)]
% [                 0,                                              0,                                                0,                                                    1]

%% RRPRRR �����ϵ��һ����d6
% ��ʵ֤����RRPRRR��RRRPRR������һ�£���������d7��
function T = forward_kine_RRP(theta) % virtual�������ᣬRRPRRR,d6�汾
    syms  a8 a9;
    d=[0 0 0 0 0 0 0 0 0];      
    a=[0 0 0 0 0 0 0 a8 a9]; 
    alpha=[0 0 0 -pi/2 pi/2 0 pi/2 -pi/2 0];
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(0,theta(6),a(6),alpha(6));
    T7=DH_forward(theta(7),d(7),a(7),alpha(7)); 
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T4*T5*T6*T7*T8*T9;
end
function T = equality_RRP(theta) %������ˣ�������ȡ��ʽ��
% ��ʵ֤����T4*T5*T6*T7==Tp*T9i*T8i����RRRP��ȫһ�£�
    syms  a8 a9;
    d=[0 0 0 0 0 0 0 0 0];      
    a=[0 0 0 0 0 0 0 a8 a9]; 
    alpha=[0 0 0 -pi/2 pi/2 0 pi/2 -pi/2 0];
    
    T6=DH_forward(0,theta(6),a(6),alpha(6)); 
    T7=DH_forward(theta(7),d(7),a(7),alpha(7)); 
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T6*T7*T8*T9;
    
    global py; % 'py' is an existing function name, class name, method name
    syms nx ox ax px ny oy ay py nz oz az pz; 
    Tp=[nx ox ax px; ny oy ay py; nz oz az pz;0 0 0 1];
    T4i=DH_inverse(theta(4),d(4),a(4),alpha(4));
    T5i=DH_inverse(theta(5),d(5),a(5),alpha(5));
    Ti=T5i*T4i*Tp;

end

%% function area

function T = forward_kine_zyz(theta) % virtual�������ᣬzyz
    syms  a8 a9 d7;%     a8=50;a9=50; %d7=0;
    d=[0 0 0 0 0 0 0 0 0];      
    a=[0 0 0 0 0 0 0 a8 a9]; 
    alpha=[0 0 0 -pi/2 pi/2 0 pi/2 -pi/2 0];
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(0,theta(7),a(7),alpha(7)); 
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
    T=T4*T5*T6*T7*T8*T9;
end

function T = equality(theta) %������ˣ�������ȡ��ʽ��
    syms  a8 a9 d7;
    d=[0 0 0 0 0 0 0 0 0];      
    a=[0 0 0 0 0 0 0 a8 a9]; 
    alpha=[0 0 0 -pi/2 pi/2 0 pi/2 -pi/2 0];
%     T4=DH_forward(theta(4),d(4),a(4),alpha(4));
%     T5=DH_forward(theta(5),d(5),a(5),alpha(5));
%     T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(0,theta(7),a(7),alpha(7)); 
%     T=T4*T5*T6*T7;
    T=trotz(theta(4))*troty(theta(5))*trotz(theta(6))*T7;%zyz�ȼ�
%     T=trotz(theta(4))*troty(theta(5))*trotx(theta(6))*T7;%zyx
    global py; % 'py' is an existing function name, class name, method name
    syms nx ox ax px ny oy ay py nz oz az pz; 
    Tp=[nx ox ax px; ny oy ay py; nz oz az pz;0 0 0 1];
    T9i=DH_inverse(theta(9),d(9),a(9),alpha(9));
    T8i=DH_inverse(theta(8),d(8),a(8),alpha(8));
    Tp*T9i*T8i

end

function T = DH_forward(theta,d,a,alpha) % ���˶�����
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha); %��׼
%     T=trotx(alpha)*transl(a,0,0)*trotz(theta)*transl(0,0,d); %�Ľ�
% ���ڱ�׼SDH�͸Ľ�MDH
% �ھ����ҳ��ϵ���Ҫ�����ǣ�x z ��ı任˳��
% ���У�ͬһ���trans��trot��˳��ɽ���������trans�����Խ�����
end

function T = DH_inverse(theta,d,a,alpha) % ���˶�����
    T=invtrot(trotx(alpha))*invtrans(transl(a,0,0))*invtrot(trotz(theta))*invtrans(transl(0,0,d));
end

function Ti=invtrot(T) %��ת����������
    for i=1:2
        for j=i+1:3
            temp=T(i,j);
            T(i,j)=T(j,i);
            T(j,i)=temp;
        end
    end
    Ti=T;
end
function Ti=invtrans(T) %ƽ�ƾ���������
    for i=1:3
        T(i,4)=-T(i,4);
    end
    Ti=T;
end
