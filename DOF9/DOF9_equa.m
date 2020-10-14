%% initialize
clc; close all; clear all;
format shortg; format compact;
%2019�� �ı����룬������ʷ�浵
%% model
theta=[0 0 0 0 0 0 0 0 0]; %��Z 
d=[90 0 0 90 90 90 0 0 0]; %��Z 
a=[0 -420 -400 0 0 0 0 50 50]; %��X  % a=[0 420 400 0 0 0]; 
alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0]; %��X
offset=[0 0 0 0 0 0 0 0 0]; %��ʼƫ�ƽ�
offset=[0 -pi/2 0 -pi/2 0 0 0 pi/2 0]; 
sigma=0; %��ת0�ƶ�1
mdh=0; %��׼0�Ľ�1
% Li = Link([theta(i) d(i) a(i) alpha(i) sigma offset(i)],mdh);
L(1) = Link([theta(1) d(1) a(1) alpha(1) sigma offset(1)],mdh);
L(2) = Link([theta(2) d(2) a(2) alpha(2) sigma offset(2)],mdh);
L(3) = Link([theta(3) d(3) a(3) alpha(3) sigma offset(3)],mdh);
L(4) = Link([theta(4) d(4) a(4) alpha(4) sigma offset(4)],mdh);
L(5) = Link([theta(5) d(5) a(5) alpha(5) sigma offset(5)],mdh);
L(6) = Link([theta(6) d(6) a(6) alpha(6) sigma offset(6)],mdh);
L(7) = Link([theta(7) d(7) a(7) alpha(7) 1 offset(7)], mdh);  L(7).qlim = [0 1000];
L(8) = Link([theta(8) d(8) a(8) alpha(8) sigma offset(8)],mdh); 
L(9) = Link([theta(9) d(9) a(9) alpha(9) sigma offset(9)],mdh);

% L7 = Link([0 0 0 -pi/2 1 0],mdh);  L7.qlim = [0 1000];
% L8 = Link([0 0 50 pi/2 0 0],mdh);
% L9 = Link([0 0 50 0 0 0],mdh);
% robot_UR5=SerialLink([L1,L2,L3,L4,L5,L6,L7,L8,L9],'name','DOF9'); 
robot_UR5=SerialLink([L(1) L(2) L(3) L(4) L(5) L(6) L(7) L(8) L(9)],'name','DOF9');%   
robot_UR5.display();
figure('NumberTitle', 'off', 'Name', 'DOF9');
robot_UR5.teach(); 


% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|         90|          0|     1.5708|          0|
% |  2|         q2|          0|       -420|          0|          0|
% |  3|         q3|          0|       -400|          0|          0|
% |  4|         q4|         90|          0|     1.5708|          0|
% |  5|         q5|         90|          0|    -1.5708|          0|
% |  6|         q6|         90|          0|          0|          0|
% |  7|          0|         q7|          0|     1.5708|          0| % translz
% |  8|         q8|          0|         50|    -1.5708|          0|
% |  9|         q9|          0|         50|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+
% syms rx ry rz xi yi zi; %��תʸ����Զ�ĵ�λ��


%% main
syms th1 th2 th3 th4 th5 th6 th8 th9; syms d7;
syms d1 d4 d5 d6 a2 a3 a8 a9;
theta=[th1 th2 th3 th4 th5 th6 d7 th8 th9]; %�ؽ��������
% global py;syms py;
% theta=[0 0 0 0 0 0 d7 0 0]; %theta[7]==d7
% theta=[0 0 0 0 0 0 0 -pi/2 0]
% T=forward_kine9(theta);
% simplify(T)
% Tmain=equation(theta)

%% equation ��������
% RCMpoint-O9point ������������⣻Զ�ĵ�RCM (Remote Center Motion)
% ��ʽ��������ȡ������12���ʽ���ɳ���ʹ�ò�ͬ�ľ�����ˣ��ҳ����׽�ĵ�ʽ��
function T = equation(theta) % Tq*T9i*T8i==T4i*T5*T6*T7
    syms a8 a9; syms d1 d4 d5 d6 a2 a3;    syms d7;
    d=[d1 0 0 d4 d5 d6 d7 0 0];    a=[0 a2 a3 0 0 0 0 a8 a9];
    alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];
    % T0=DH_forward(0,0,0,pi/2); %O0 to O6����̬ת�� 
    %��O6�㣬����λ������ͬ�ģ�ŷ����ZYZ��ϣ�
    T4=DH_forward(theta(4),0,0,alpha(4)); % T4=DH_forward(theta(4),0,0,pi/2);
    T5=DH_forward(theta(5),0,0,alpha(5)); % T5=DH_forward(theta(5),0,0,-pi/2);
    T6=DH_forward(theta(6),0,0,alpha(6)); % T6=DH_forward(theta(6),0,0,0);
    T7=DH_forward(0,theta(7),a(7),alpha(7)); % T7=DH_forward(0,theta(7),0,pi/2);
    T8=DH_forward(theta(8),d(8),a(8),alpha(8)); % T8=DH_forward(theta(8),0,a8,-pi/2);
    T9=DH_forward(theta(9),d(9),a(9),alpha(9)); % T9=DH_forward(theta(9),0,a9,0);
    T=T4*T5*T6*T7*T8*T9; %simplify(T)
    global py;
    syms nx ox ax px ny oy py ay nz oz az pz;  %th1 th2 th3 th4 th5 th6 d1 d4 d5 d6 a2 a3 
    Tq=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1]; %����py���ܷ��ں����ڲ�����global��
    % T4i=DH_inverse(theta(4),0,0,alpha(4));
%     T7i=DH_inverse(0,theta(7),0,pi/2);
    T8i=DH_inverse(theta(8),d(8),a(8),alpha(8));%     T8i=DH_inverse(theta(8),0,a8,-pi/2);
    T9i=DH_inverse(theta(9),d(9),a(9),alpha(9));%     T9i=DH_inverse(theta(9),0,a9,0);
%     Tq98=Tq*T9i*T8i;   % simplify(Tq98)
end

%%  Equation group:   T4*T5*T6*T7==Tq*T9i*T8i
% inverse1 T5678==T4i*Tq*T9i �˷������ý�
% T4567=T4*T5*T6*T7;  %     simplify(T4567)
% [ cos(th4)*cos(th5)*cos(th6) - sin(th4)*sin(th6), -cos(th4)*sin(th5), cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6), -d7*cos(th4)*sin(th5)]
% [ cos(th4)*sin(th6) + cos(th5)*cos(th6)*sin(th4), -sin(th4)*sin(th5), cos(th5)*sin(th4)*sin(th6) - cos(th4)*cos(th6), -d7*sin(th4)*sin(th5)]
% [                              cos(th6)*sin(th5),           cos(th5),                              sin(th5)*sin(th6),           d7*cos(th5)]
%     Tq98=Tq*T9i*T8i;  %     simplify(Tq98)
% [ cos(th8)*(nx*cos(th9) - ox*sin(th9)) - ax*sin(th8), sin(th8)*(nx*cos(th9) - ox*sin(th9)) + ax*cos(th8), - ox*cos(th9) - nx*sin(th9), px - a9*nx - a8*(nx*cos(th9) - ox*sin(th9))]
% [ cos(th8)*(ny*cos(th9) - oy*sin(th9)) - ay*sin(th8), sin(th8)*(ny*cos(th9) - oy*sin(th9)) + ay*cos(th8), - oy*cos(th9) - ny*sin(th9), py - a9*ny - a8*(ny*cos(th9) - oy*sin(th9))]
% [ cos(th8)*(nz*cos(th9) - oz*sin(th9)) - az*sin(th8), sin(th8)*(nz*cos(th9) - oz*sin(th9)) + az*cos(th8), - oz*cos(th9) - nz*sin(th9), pz - a9*nz - a8*(nz*cos(th9) - oz*sin(th9))]
% 
% sin(th5)*sin(th8) - cos(th5)*cos(th6)*cos(th8) == sin(th9)*(ox*cos(th4)+oy*sin(th4))-cos(th9)*(nx*cos(th4)+ny*sin(th4))   ��
% cos(th5)*sin(th8) + sin(th5)*cos(th6)*cos(th8) == nz*cos(th9)-oz*sin(th9)                                                 ��
%                            - sin(th6)*cos(th8) == sin(th9)*(oy*cos(th4)-ox*sin(th4))-cos(th9)*(ny*cos(th4)-nx*sin(th4))   ��
% 
% -cos(th5)*sin(th6) == sin(th9)*(nx*cos(th4)+ny*sin(th4))+cos(th9)*(ox*cos(th4)+oy*sin(th4))   ��
% -sin(th5)*sin(th6) == oz*cos(th9)+nz*sin(th9)                                                 ��
%           cos(th6) == sin(th9)*(ny*cos(th4)-nx*sin(th4))+cos(th9)*(oy*cos(th4)-ox*sin(th4))   ��
% 
% -sin(th5)*cos(th8) - cos(th5)*cos(th6)*sin(th8) == ax*cos(th4)+ay*sin(th4)  ��
%  cos(th5)*cos(th8) - sin(th5)*cos(th6)*sin(th8) == az                       ��
%                               sin(th6)*sin(th8) == ax*sin(th4)-ay*cos(th4)  ��
%
% -d7*sin(th5)-a8*(sin(th5)*sin(th8)-cos(th5)*cos(th6)*cos(th8)) == px*cos(th4)+py*sin(th4)-a9*(nx*cos(th4)+ny*sin(th4))  ��
%  d7*cos(th5)+a8*(cos(th5)*sin(th8)+sin(th5)*cos(th6)*cos(th8)) == pz-a9*nz                                              �٢�
%                                          -a8*sin(th6)*cos(th8) == px*sin(th4)-py*cos(th4)+a9*(ny*cos(th4)-nx*sin(th4))  �٢�
% 
% a9=0 version
% -d7*sin(th5) - a8*(sin(th5)*sin(th8)-cos(th5)*cos(th6)*cos(th8)) == px*cos(th4)+py*sin(th4)  ��
%  d7*cos(th5) + a8*(cos(th5)*sin(th8)+sin(th5)*cos(th6)*cos(th8)) == pz                       �٢�
%                                            -a8*sin(th6)*cos(th8) == px*sin(th4)-py*cos(th4)  �٢�
%

%{
%     T5678=T5*T6*T7*T8;  T=T5*T6*T7*T8; %T58
%     simplify(T5678)
%     [ cos(th5)*cos(th6)*cos(th8) - sin(th5)*sin(th8), -cos(th5)*sin(th6), - cos(th8)*sin(th5) - cos(th5)*cos(th6)*sin(th8), a8*cos(th5)*cos(th6)*cos(th8) - a8*sin(th5)*sin(th8) - d7*sin(th5)]
%     [ cos(th5)*sin(th8) + cos(th6)*cos(th8)*sin(th5), -sin(th5)*sin(th6),   cos(th5)*cos(th8) - cos(th6)*sin(th5)*sin(th8), d7*cos(th5) + a8*cos(th5)*sin(th8) + a8*cos(th6)*cos(th8)*sin(th5)]
%     [                             -cos(th8)*sin(th6),          -cos(th6),                                sin(th6)*sin(th8),                                              -a8*cos(th8)*sin(th6)]

%         T4q9=T4i*Tq*T9i;
%         simplify(T4q9)
%     [ cos(th9)*(nx*cos(th4) + ny*sin(th4)) - sin(th9)*(ox*cos(th4) + oy*sin(th4)),   sin(th9)*(nx*cos(th4) + ny*sin(th4)) + cos(th9)*(ox*cos(th4) + oy*sin(th4)), ax*cos(th4) + ay*sin(th4), px*cos(th4) - a9*(nx*cos(th4) + ny*sin(th4)) + py*sin(th4)]
%     [                                                   nz*cos(th9) - oz*sin(th9),                                                     oz*cos(th9) + nz*sin(th9),                        az,                                                 pz - a9*nz]
%     [ sin(th9)*(oy*cos(th4) - ox*sin(th4)) - cos(th9)*(ny*cos(th4) - nx*sin(th4)), - sin(th9)*(ny*cos(th4) - nx*sin(th4)) - cos(th9)*(oy*cos(th4) - ox*sin(th4)), ax*sin(th4) - ay*cos(th4), a9*(ny*cos(th4) - nx*sin(th4)) - py*cos(th4) + px*sin(th4)]

    T=T4*T5*T6;
    [ cos(th4)*cos(th5)*cos(th6) - sin(th4)*sin(th6), - cos(th6)*sin(th4) - cos(th4)*cos(th5)*sin(th6), -cos(th4)*sin(th5), 0]
    [ cos(th4)*sin(th6) + cos(th5)*cos(th6)*sin(th4),   cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6), -sin(th4)*sin(th5), 0]
    [                              cos(th6)*sin(th5),                               -sin(th5)*sin(th6),           cos(th5), 0]
    T=T7*T8*T9;
    [ cos(th8)*cos(th9), -cos(th8)*sin(th9), -sin(th8),      a8*cos(th8) + a9*cos(th8)*cos(th9)]
    [          sin(th9),           cos(th9),         0,                             a9*sin(th9)]
    [ cos(th9)*sin(th8), -sin(th8)*sin(th9),  cos(th8), d7 + a8*sin(th8) + a9*cos(th9)*sin(th8)]

    T=T4*T5*T6*T7*T8*T9;
%}

%% T5678=T5*T6*T7*T8;  T=T5*T6*T7*T8; %T58 ���շ���
% inverse1 T5678==T4i*Tq*T9i
% ��������⡣���հ��ʽ
% RCM to Oxyz7��T4567=Tq98��% �������
%     simplify(T5678)
%     [ cos(th5)*cos(th6)*cos(th8) - sin(th5)*sin(th8), -cos(th5)*sin(th6), - cos(th8)*sin(th5) - cos(th5)*cos(th6)*sin(th8), a8*cos(th5)*cos(th6)*cos(th8) - a8*sin(th5)*sin(th8) - d7*sin(th5)]
%     [ cos(th5)*sin(th8) + cos(th6)*cos(th8)*sin(th5), -sin(th5)*sin(th6),   cos(th5)*cos(th8) - cos(th6)*sin(th5)*sin(th8), d7*cos(th5) + a8*cos(th5)*sin(th8) + a8*cos(th6)*cos(th8)*sin(th5)]
%     [                             -cos(th8)*sin(th6),          -cos(th6),                                sin(th6)*sin(th8),                                              -a8*cos(th8)*sin(th6)]

%         T4q9=T4i*Tq*T9i;
%         simplify(T4q9)
%     [ cos(th9)*(nx*cos(th4) + ny*sin(th4)) - sin(th9)*(ox*cos(th4) + oy*sin(th4)),   sin(th9)*(nx*cos(th4) + ny*sin(th4)) + cos(th9)*(ox*cos(th4) + oy*sin(th4)), ax*cos(th4) + ay*sin(th4), px*cos(th4) - a9*(nx*cos(th4) + ny*sin(th4)) + py*sin(th4)]
%     [                                                   nz*cos(th9) - oz*sin(th9),                                                     oz*cos(th9) + nz*sin(th9),                        az,                                                 pz - a9*nz]
%     [ sin(th9)*(oy*cos(th4) - ox*sin(th4)) - cos(th9)*(ny*cos(th4) - nx*sin(th4)), - sin(th9)*(ny*cos(th4) - nx*sin(th4)) - cos(th9)*(oy*cos(th4) - ox*sin(th4)), ax*sin(th4) - ay*cos(th4), a9*(ny*cos(th4) - nx*sin(th4)) - py*cos(th4) + px*sin(th4)]
% RCM to O7��T4567=Tq98��% �������

% -sin(th4)*sin(th6) + cos(th4)*cos(th5)*cos(th6) == -ax*sin(th8) + cos(th8)*(nx*cos(th9)-ox*sin(th9))  ��
%  cos(th4)*sin(th6) + sin(th4)*cos(th5)*cos(th6) == -ay*sin(th8) + cos(th8)*(ny*cos(th9)-oy*sin(th9))  ��
%                               sin(th5)*cos(th6) == -az*sin(th8) + cos(th8)*(nz*cos(th9)-oz*sin(th9))  ��
% 
% -cos(th4)*sin(th5) == ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))  ��
% -sin(th4)*sin(th5) == ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))  ��
%           cos(th5) == az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))  ��
% 
%  sin(th4)*cos(th6) + cos(th4)*cos(th5)*sin(th6) == -ox*cos(th9)-nx*sin(th9)  ��
% -cos(th4)*cos(th6) + sin(th4)*cos(th5)*sin(th6) == -oy*cos(th9)-ny*sin(th9)  ��
%                               sin(th5)*sin(th6) == -oz*cos(th9)-nz*sin(th9)  ��
% 
% -d7*cos(th4)*sin(th5) == px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9))  ��
% -d7*sin(th4)*sin(th5) == py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9))  �٢�
%           d7*cos(th5) == pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9))  �٢�

%     T=T4*T5*T6;
%     [ cos(th4)*cos(th5)*cos(th6) - sin(th4)*sin(th6), - cos(th6)*sin(th4) - cos(th4)*cos(th5)*sin(th6), -cos(th4)*sin(th5), 0]
%     [ cos(th4)*sin(th6) + cos(th5)*cos(th6)*sin(th4),   cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6), -sin(th4)*sin(th5), 0]
%     [                              cos(th6)*sin(th5),                               -sin(th5)*sin(th6),           cos(th5), 0]
%     T=T7*T8*T9;
%     [ cos(th8)*cos(th9), -cos(th8)*sin(th9), -sin(th8),      a8*cos(th8) + a9*cos(th8)*cos(th9)]
%     [          sin(th9),           cos(th9),         0,                             a9*sin(th9)]
%     [ cos(th9)*sin(th8), -sin(th8)*sin(th9),  cos(th8), d7 + a8*sin(th8) + a9*cos(th9)*sin(th8)]
% 
%     T=T4*T5*T6*T7*T8*T9;
%


%% forward9
function T = forward_kine9(theta)% ���˶�
    syms d1 d4 d5 d6 a2 a3 a8 a9; syms d7;
%     d1=90;d4=90;d5=90;d6=90;%d7=0;
%     a2=-420;a3=-400;a8=50;a9=50;
    d=[d1 0 0 d4 d5 d6 d7 0 0];
    a=[0 a2 a3 0 0 0 0 a8 a9];
    alpha=[pi/2 0 0 pi/2 -pi/2 0 pi/2 -pi/2 0];

    T1=DH_forward(theta(1),d(1),a(1),alpha(1));
    T2=DH_forward(theta(2),d(2),a(2),alpha(2));
    T3=DH_forward(theta(3),d(3),a(3),alpha(3));
    T4=DH_forward(theta(4),d(4),a(4),alpha(4));
    T5=DH_forward(theta(5),d(5),a(5),alpha(5));
    T6=DH_forward(theta(6),d(6),a(6),alpha(6));
    T7=DH_forward(0,theta(7),a(7),alpha(7)); %λ�þ���˳��
    T8=DH_forward(theta(8),d(8),a(8),alpha(8));
    T9=DH_forward(theta(9),d(9),a(9),alpha(9));
%     T=T1*T2*T3*T4*T5*T6*T7*T8*T9;
    T=T1*T2*T3*T4*T5*T6;
end

%% function
function T = DH_forward(theta,d,a,alpha) % ���˶�����
% T=transZ(0,0,d)*trotZ(theta)*transX(a,0,0)*trotX(alpha);
    T=transl(0,0,d)*trotz(theta)*transl(a,0,0)*trotx(alpha);
end
function T = DH_inverse(theta,d,a,alpha) % ���˶�����
    T=invtrot(trotx(alpha))*invtrans(transl(a,0,0))*invtrot(trotz(theta))*invtrans(transl(0,0,d));
end
function [Ti]=invtrot(T) %��ת���������
    for i=1:2
        for j=i+1:3
            temp=T(i,j);
            T(i,j)=T(j,i);
            T(j,i)=temp;
        end
    end
    Ti=T;
end
function [Ti]=invtrans(T) %ƽ�ƾ��������
    for i=1:3
        T(i,4)=-T(i,4);
    end
    Ti=T;
end





