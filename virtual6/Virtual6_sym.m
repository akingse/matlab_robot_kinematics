%% setting
clc;clear;close all;
format short g;format compact;
% �����﷨
% % MATLAB�е�strfind��strrep���������Ƿֱ��������ַ����в����Ӵ����滻�Ӵ���
% ��1��simplify �����Ա��ʽ���л���
% ��2��radsimp�����Ժ���ʽ�ı��ʽ���л���
% ��3��combine ���������ʽ������͡��˻������������ʽ���ֵ�����кϲ���
% ��4��collet�ϲ�ͬ����
% ��5��factor����ʵ����ʽ�ֽ�
% ��6) convert������ɱ��ʽ��ʽ��ת��
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
symbol�����Ƶ�
���Ӧ����Virtual6�����հ滯������ˣ��ϸ����������п��ܷ��������������matlab��ǿ��������
�÷����Ƶ���ȼ۱��ʽ�����ж�����ͷ�ĸΪ��������������Щ���ѡ���Ӧ�Ĳ�����Ϊ�����Ǻ��������������

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}



%% �������ᣬ�����Ƶ�
syms th1 th2 th3 th4 th5 th6 d7 th8 th9;
syms nx ox ax px ny oy ay py nz oz az pz;
syms d1 d4 d5 d6 a1 a2 a3 a8 a9 d7;
theta=[th1 th2 th3 th4 th5 th6 d7 th8 th9]; %�ؽ��������
% ���ݱ��ʽ���Ƶ�����У������Ǻ����Ĳ����ı��ʽ���ж��������ԣ�
% noap���ʽ��z(-y)z��
% nx=-cos(th9)*(cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))+cos(th4)*sin(th5)*sin(th8))-sin(th9)*(cos(th6)*sin(th4)+cos(th4)*cos(th5)*sin(th6));
% ox=sin(th9)*(cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))+cos(th4)*sin(th5)*sin(th8))-cos(th9)*(cos(th6)*sin(th4)+cos(th4)*cos(th5)*sin(th6));
% ax=sin(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))-cos(th4)*cos(th8)*sin(th5);
% px=-a8*cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))-a9*sin(th9)*(cos(th6)*sin(th4)+cos(th4)*cos(th5)*sin(th6))-d7*cos(th4)*sin(th5)-a9*cos(th9)*(cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))+cos(th4)*sin(th5)*sin(th8))-a8*cos(th4)*sin(th5)*sin(th8);
% 
% ny=cos(th9)*(cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))-sin(th4)*sin(th5)*sin(th8))+sin(th9)*(cos(th4)*cos(th6)-cos(th5)*sin(th4)*sin(th6));
% oy=cos(th9)*(cos(th4)*cos(th6)-cos(th5)*sin(th4)*sin(th6))-sin(th9)*(cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))-sin(th4)*sin(th5)*sin(th8));
% ay=-sin(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))-cos(th8)*sin(th4)*sin(th5);
% py=a8*cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))+a9*sin(th9)*(cos(th4)*cos(th6)-cos(th5)*sin(th4)*sin(th6))-d7*sin(th4)*sin(th5)+a9*cos(th9)*(cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))-sin(th4)*sin(th5)*sin(th8))-a8*sin(th4)*sin(th5)*sin(th8);
% 
% nz=cos(th9)*(cos(th5)*sin(th8)+cos(th6)*cos(th8)*sin(th5))-sin(th5)*sin(th6)*sin(th9);
% oz=-sin(th9)*(cos(th5)*sin(th8)+cos(th6)*cos(th8)*sin(th5))-cos(th9)*sin(th5)*sin(th6);
% az=cos(th5)*cos(th8)-cos(th6)*sin(th5)*sin(th8);
% pz=d7*cos(th5)+a9*cos(th9)*(cos(th5)*sin(th8)+cos(th6)*cos(th8)*sin(th5))+a8*cos(th5)*sin(th8)+a8*cos(th6)*cos(th8)*sin(th5)-a9*sin(th5)*sin(th6)*sin(th9);

% noap���ʽ��zyz��
nx=-cos(th9)*(cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))-cos(th4)*sin(th5)*sin(th8))-sin(th9)*(cos(th6)*sin(th4)+cos(th4)*cos(th5)*sin(th6));
ox=sin(th9)*(cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))-cos(th4)*sin(th5)*sin(th8))-cos(th9)*(cos(th6)*sin(th4)+cos(th4)*cos(th5)*sin(th6));
ax=sin(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))+cos(th4)*cos(th8)*sin(th5);
px=d7*cos(th4)*sin(th5)-a9*sin(th9)*(cos(th6)*sin(th4)+cos(th4)*cos(th5)*sin(th6))-a8*cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))-a9*cos(th9)*(cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))-cos(th4)*sin(th5)*sin(th8))+a8*cos(th4)*sin(th5)*sin(th8);

ny=cos(th9)*(cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))+sin(th4)*sin(th5)*sin(th8))+sin(th9)*(cos(th4)*cos(th6)-cos(th5)*sin(th4)*sin(th6));
oy=cos(th9)*(cos(th4)*cos(th6)-cos(th5)*sin(th4)*sin(th6))-sin(th9)*(cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))+sin(th4)*sin(th5)*sin(th8));
ay=cos(th8)*sin(th4)*sin(th5)-sin(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4));
py=a8*cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))+a9*sin(th9)*(cos(th4)*cos(th6)-cos(th5)*sin(th4)*sin(th6))+d7*sin(th4)*sin(th5)+a9*cos(th9)*(cos(th8)*(cos(th4)*sin(th6)+cos(th5)*cos(th6)*sin(th4))+sin(th4)*sin(th5)*sin(th8))+a8*sin(th4)*sin(th5)*sin(th8);

nz=cos(th9)*(cos(th5)*sin(th8)-cos(th6)*cos(th8)*sin(th5))+sin(th5)*sin(th6)*sin(th9);
oz=cos(th9)*sin(th5)*sin(th6)-sin(th9)*(cos(th5)*sin(th8)-cos(th6)*cos(th8)*sin(th5));
az=cos(th5)*cos(th8)+cos(th6)*sin(th5)*sin(th8);
pz=d7*cos(th5)+a9*cos(th9)*(cos(th5)*sin(th8)-cos(th6)*cos(th8)*sin(th5))+a8*cos(th5)*sin(th8)-a8*cos(th6)*cos(th8)*sin(th5)+a9*sin(th5)*sin(th6)*sin(th9);


% P1=px^2+py^2+pz^2;
% simplify(P1) %a8^2 + a9^2 + d7^2 + 2*a8*d7*sin(th8) - a8*a9*(1/cos(th9))- a8*a9*cos(2*th9)*(1/cos(th9))- a9*d7*sin(th8)*(1/cos(th9))- a9*d7*cos(2*th9)*sin(th8)*(1/cos(th9))
% p=2*a9*(px*nx+py*ny+pz*nz);
% p=simplify(p) %�޷�����
fprintf("---\n")

% -------------------------------------------------------------------------
%{
% ��ⷽ���飻������z(-y)z���۵���
% -sin(th4)*sin(th6) + cos(th4)*cos(th5)*cos(th6) == -ax*sin(th8) + cos(th8)*(nx*cos(th9)-ox*sin(th9))  �� ��
%  cos(th4)*sin(th6) + sin(th4)*cos(th5)*cos(th6) == -ay*sin(th8) + cos(th8)*(ny*cos(th9)-oy*sin(th9))  �� ��
%                               sin(th5)*cos(th6) == -az*sin(th8) + cos(th8)*(nz*cos(th9)-oz*sin(th9))  ��

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
% ���˼·z(-y)z
% -d7*cos(th4)*sin(th5) == px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9))  %% ������ʽ��������th4 th5
%    -cos(th4)*sin(th5) == ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))
% -d7*sin(th4)*sin(th5) == py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9))  
%    -sin(th4)*sin(th5) == ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))
%           d7*cos(th5) == pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9))  
%              cos(th5) == az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))

% cos(th5) == az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))  
%  sin(th4)*cos(th6) + cos(th4)*sin(th6) == -ox*cos(th9)-nx*sin(th9) %%  cos(th5) == 1��th4 th6����
% -cos(th4)*cos(th6) + sin(th4)*sin(th6) == -oy*cos(th9)-ny*sin(th9)

% -cos(th4)*sin(th5) == ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))% ==>cos(th4)
% -sin(th4)*sin(th5) == ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9)) % ==>sin(th4)

% sin(th5)*cos(th6) == -az*sin(th8) + cos(th8)*(nz*cos(th9)-oz*sin(th9))  % ==>cos(th6)
% sin(th5)*sin(th6) == -oz*cos(th9)-nz*sin(th9)                           % ==>sin(th6)
%}
% -------------------------------------------------------------------------
% ���ڵ�zyz
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

% -------------------------------------------------------------------------




%% ��th9 �м仯��
% һ��һ����ӡ����������ǿ��
% d7*(ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))) == px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9))  ��
% d7*(ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))) == py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9))  ��
% d7*(az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))) == pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9))  ��
% d7 == (Pz-a8*(nz*cos(th9)-oz*sin(th9))) / (az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9)))  �ܴ��� �� ��
% ����汾���� Pz-a8RzΪ��ĸ��������Py-a8Ry��ʼ����

Px=px-a9*nx;
Py=py-a9*ny;
Pz=pz-a9*nz;
Rx=nx*cos(th9)-ox*sin(th9); simplify(Rx) ;
Ry=ny*cos(th9)-oy*sin(th9); simplify(Ry) ;
Rz=nz*cos(th9)-oz*sin(th9); simplify(Rz) ;
% x=Px*Ry*az - Px*Rz*ay - Py*Rx*az + Py*Rz*ax + Pz*Rx*ay - Pz*Ry*ax
% simplify(x) 
%     0=ax*(Pz-a8*Rz)-az*(Px-a8*Rx) %th9
%     0=ax*(Pz+a8*Rz)-az*(Px+a8*Rx) %th9+pi�������
%     0=Rz*(Px-a8*Rx)-Rx*(Pz-a8*Rz)
%    0=-Rz*(Px+a8*Rx)+Rx*(Pz+a8*Rz)

%     0=ax*(Pz-a8*Rz)-az*(Px-a8*Rx) ==> 0=Rx*(Pz-a8*Rz)-Rz*(Px-a8*Rx)
%     0=ax*(Pz+a8*Rz)-az*(Px+a8*Rx) ==> 0=Rx*(Pz+a8*Rz)-Rz*(Px+a8*Rx)
% ax= sin(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))+cos(th4)*sin(th5)*cos(th8);
% Rx=-cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))+cos(th4)*sin(th5)*sin(th8);
% az=cos(th5)*cos(th8)+cos(th6)*sin(th5)*sin(th8);
% Rz=cos(th5)*sin(th8)-cos(th6)*sin(th5)*cos(th8);
% (Px-a8*Rx)==-d7*cos(th4)*sin(th5)
% (Py-a8*Ry)==-d7*sin(th4)*sin(th5)
% (Pz-a8*Rz)==d7*cos(th5)

% syms Rx Ry Rz Px Py Pz ;%ʹ��matlab����Ĺؼ������� Rx Ry Rz
syms cos8 sin8; %������ű����ظ�����.����ʹ�� sym/subsindex 
% (Pz-a8*(nz*cos(th9)-oz*sin(th9)))*(ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)))==(Px-a8*(nx*cos(th9)-ox*sin(th9)))*(az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9))) %z
% (Pz-a8*(nz*cos(th9)-oz*sin(th9)))*(ay*cos(th8)+sin(th8)*(ny*cos(th9)-oy*sin(th9)))==(Py-a8*(ny*cos(th9)-oy*sin(th9)))*(az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9)))
% (Py-a8*(ny*cos(th9)-oy*sin(th9)))*(ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)))==(Px-a8*(nx*cos(th9)-ox*sin(th9)))*(ay*cos(th8)+sin(th8)*(ny*cos(th9)-oy*sin(th9))) %y
% (Py-a8*(ny*cos(th9)-oy*sin(th9)))*(az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9)))==(Pz-a8*(nz*cos(th9)-oz*sin(th9)))*(ay*cos(th8)+sin(th8)*(ny*cos(th9)-oy*sin(th9)))
% (Px-a8*(nx*cos(th9)-ox*sin(th9)))*(ay*cos(th8)+sin(th8)*(ny*cos(th9)-oy*sin(th9)))==(Py-a8*(ny*cos(th9)-oy*sin(th9)))*(ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)))  %x
% (Px-a8*(nx*cos(th9)-ox*sin(th9)))*(az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9)))==(Pz-a8*(nz*cos(th9)-oz*sin(th9)))*(ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)))

% (Pz-a8*Rz)*(ax*cos(th8) + sin(th8)*Rx) == (Px-a8*Rx)*(az*cos(th8) + sin(th8)*Rz) %z
% (Pz-a8*Rz)*(ay*cos(th8) + sin(th8)*Ry) == (Py-a8*Ry)*(az*cos(th8) + sin(th8)*Rz)
% (Py-a8*Ry)*(ax*cos(th8) + sin(th8)*Rx) == (Px-a8*Rx)*(ay*cos(th8) + sin(th8)*Ry) %y
% (Py-a8*Ry)*(az*cos(th8) + sin(th8)*Rz) == (Pz-a8*Rz)*(ay*cos(th8) + sin(th8)*Ry)
% (Px-a8*Rx)*(ay*cos(th8) + sin(th8)*Ry) == (Py-a8*Ry)*(ax*cos(th8) + sin(th8)*Rx) %z
% (Px-a8*Rx)*(az*cos(th8) + sin(th8)*Rz) == (Pz-a8*Rz)*(ax*cos(th8) + sin(th8)*Rx)

% �ϲ�ͬ����
% ʹ��collect�������ϲ�cos(th8) sin(th8)�����㲻����㣬�����yyds������Զ�������ż������
% �����ʱ��ע�͵� noap,��syms Rx Ry Rz Px Py Pz ;ע������ĸ��ţ�
Tz=(Pz-a8*Rz)*(ax*cos8 + sin8*Rx) - (Px-a8*Rx)*(az*cos8 + sin8*Rz); %z
Tz=(Pz-a8*Rz)*(ay*cos8 + sin8*Ry) - (Py-a8*Ry)*(az*cos8 + sin8*Rz);
Ty=(Py-a8*Ry)*(ax*cos8 + sin8*Rx) - (Px-a8*Rx)*(ay*cos8 + sin8*Ry); %y
Ty=(Py-a8*Ry)*(az*cos8 + sin8*Rz) - (Pz-a8*Rz)*(ay*cos8 + sin8*Ry);
Tx=(Px-a8*Rx)*(ay*cos8 + sin8*Ry) - (Py-a8*Ry)*(ax*cos8 + sin8*Rx); %x
Tx=(Px-a8*Rx)*(az*cos8 + sin8*Rz) - (Pz-a8*Rz)*(ax*cos8 + sin8*Rx);

collect(Tx,cos8); %��Ȼ���ϲ�ͬ���ຯ����ÿ��ֻ�ܺϲ�һ��������
Tz=-(Rx*sin8*(Pz - Rz*a8) - Rz*sin8*(Px - Rx*a8));
Tz=-(Ry*sin8*(Pz - Rz*a8) - Rz*sin8*(Py - Ry*a8));
Ty=-(Rx*sin8*(Py - Ry*a8) - Ry*sin8*(Px - Rx*a8));
Ty=-(Rz*sin8*(Py - Ry*a8) - Ry*sin8*(Pz - Rz*a8));
Tx=-(Ry*sin8*(Px - Rx*a8) - Rx*sin8*(Py - Ry*a8));
Tx=-(Rz*sin8*(Px - Rx*a8) - Rx*sin8*(Pz - Rz*a8));
% collect(Tx,sin8)

% (ax*(Pz-Rz*a8)-az*(Px-Rx*a8))*cos8 == (Rz*(Px-Rx*a8)-Rx*(Pz-Rz*a8))*sin8 %z
% (ay*(Pz-Rz*a8)-az*(Py-Ry*a8))*cos8 == (Rz*(Py-Ry*a8)-Ry*(Pz-Rz*a8))*sin8
% (ax*(Py-Ry*a8)-ay*(Px-Rx*a8))*cos8 == (Ry*(Px-Rx*a8)-Rx*(Py-Ry*a8))*sin8 %y
% (az*(Py-Ry*a8)-ay*(Pz-Rz*a8))*cos8 == (Ry*(Pz-Rz*a8)-Rz*(Py-Ry*a8))*sin8
% (ay*(Px-Rx*a8)-ax*(Py-Ry*a8))*cos8 == (Rx*(Py-Ry*a8)-Ry*(Px-Rx*a8))*sin8 %x
% (az*(Px-Rx*a8)-ax*(Pz-Rz*a8))*cos8 == (Rx*(Pz-Rz*a8)-Rz*(Px-Rx*a8))*sin8

% ������ˣ������
Tz=(ax*(Pz-Rz*a8)-az*(Px-Rx*a8))*(Rz*(Py-Ry*a8)-Ry*(Pz-Rz*a8))-(ay*(Pz-Rz*a8)-az*(Py-Ry*a8))*(Rz*(Px-Rx*a8)-Rx*(Pz-Rz*a8));
Ty=(ax*(Py-Ry*a8)-ay*(Px-Rx*a8))*(Ry*(Pz-Rz*a8)-Rz*(Py-Ry*a8))-(az*(Py-Ry*a8)-ay*(Pz-Rz*a8))*(Ry*(Px-Rx*a8)-Rx*(Py-Ry*a8));
Tx=(ay*(Px-Rx*a8)-ax*(Py-Ry*a8))*(Rx*(Pz-Rz*a8)-Rz*(Px-Rx*a8))-(az*(Px-Rx*a8)-ax*(Pz-Rz*a8))*(Rx*(Py-Ry*a8)-Ry*(Px-Rx*a8));
simplify(Tz) 
% (Pz - Rz*a8)*(Px*Ry*az - Px*Rz*ay - Py*Rx*az + Py*Rz*ax + Pz*Rx*ay - Pz*Ry*ax)==0
% (Py - Ry*a8)*(Px*Ry*az - Px*Rz*ay - Py*Rx*az + Py*Rz*ax + Pz*Rx*ay - Pz*Ry*ax)==0
% (Px - Rx*a8)*(Px*Ry*az - Px*Rz*ay - Py*Rx*az + Py*Rz*ax + Pz*Rx*ay - Pz*Ry*ax)==0

% �ؼ����裬��Ҫ�˿����˿��͵õ�4�η���
% 0==Rx*(Pz*ay-Py*az) + Ry*(Px*az-Pz*ax) + Rz*(Py*ax-Px*ay);
% 0==Pz - Rz*a8==pz-a9*nz - a8*(nz*cos(th9)-oz*sin(th9))
% T9=Px*Ry*az - Px*Rz*ay - Py*Rx*az + Py*Rz*ax + Pz*Rx*ay - Pz*Ry*ax;
% (Px*ay*oz-Px*az*oy-Py*ax*oz+Py*az*ox+Pz*ax*oy-Pz*ay*ox)*sin+(Px*az*ny-Px*ay*nz+Py*ax*nz-Py*az*nx-Pz*ax*ny+Pz*ay*nx)*cos
% (Px*(ay*oz-az*oy)+Py*(az*ox-ax*oz)+Pz*(ax*oy-ay*ox))*sin+(Px*(az*ny-ay*nz)+Py*(ax*nz-az*nx)+Pz*(ay*nx-ax*ny))*cos
Pm=Px*(az*ny-ay*nz)+Py*(ax*nz-az*nx)+Pz*(ay*nx-ax*ny);
Pn=Px*(az*oy-ay*oz)+Py*(ax*oz-az*ox)+Pz*(ay*ox-ax*oy);
% 0==Pm*cos(th9) - Pn*sin(th9) %���ˣ��ɵ�th9�ı��ʽ���������� %�����Գƣ�nice
% Pm=(Pz*ay-Py*az)*nx+(Px*az-Pz*ax)*ny+(Py*ax-Px*ay)*nz; Pm=simplify(Pm); %sin(th9)*(a8 + d7*sin(th8)) ����zyzӰ��
% Pn=(Pz*ay-Py*az)*ox+(Px*az-Pz*ax)*oy+(Py*ax-Px*ay)*oz; Pn=simplify(Pn); %cos(th9)*(a8 + d7*sin(th8))

% һԪ�Ĵ�ר��
% a=a8*nz*Pn; % a8*sin(th9)*(a8 + d7*sin(th8))*(cos(th5)*cos(th9)*sin(th8) - sin(th5)*sin(th6)*sin(th9) + cos(th6)*cos(th8)*cos(th9)*sin(th5))
% b=a8*oz*Po; %-a8*cos(th9)*(a8 + d7*sin(th8))*(cos(th9)*sin(th5)*sin(th6) + cos(th5)*sin(th8)*sin(th9) + cos(th6)*cos(th8)*sin(th5)*sin(th9))
% c=-a8*(oz*Pn+nz*Po); %a8*(a8 + d7*sin(th8))*(cos(th5)*sin(th8) - 2*cos(th5)*cos(th9)^2*sin(th8) + cos(th6)*cos(th8)*sin(th5) - 2*cos(th6)*cos(th8)*cos(th9)^2*sin(th5) + 2*cos(th9)*sin(th5)*sin(th6)*sin(th9))
% d=-Pz*Pn; %-sin(th9)*(a8 + d7*sin(th8))*(d7*cos(th5) + a8*cos(th5)*sin(th8) + a8*cos(th6)*cos(th8)*sin(th5))
% e=Pz*Po; %  cos(th9)*(a8 + d7*sin(th8))*(d7*cos(th5) + a8*cos(th5)*sin(th8) + a8*cos(th6)*cos(th8)*sin(th5))
% s=d*cos(th9)+e*sin(th9);%0
% delta=simplify(c*c-4*a*b) %a8^2*(a8 + d7*sin(th8))^2*(cos(th5)*sin(th8) + cos(th6)*cos(th8)*sin(th5))^2
% T=simplify(t)
% U=simplify(d)

%% th8
% % m81=ax*(Pz-a8*Rz)-az*(Px-a8*Rx);%-d7*sin(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6)) ԭz(-y)z�棬��zyz���һ�£�
% % n81=Rz*(Px-a8*Rx)-Rx*(Pz-a8*Rz);%-d7*cos(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6))
% % m82=ay*(Pz-a8*Rz)-az*(Py-a8*Ry);%-d7*sin(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6))
% % n82=Rz*(Py-a8*Ry)-Ry*(Pz-a8*Rz);%-d7*cos(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6))

% m8=(ax*(Pz-Rz*a8)-az*(Px-Rx*a8)); %Pxz% -d7*sin(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6)) %1
% n8=(Rz*(Px-Rx*a8)-Rx*(Pz-Rz*a8)); %Rxz% -d7*cos(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6)) %2
% m8=(ay*(Pz-Rz*a8)-az*(Py-Ry*a8)); %Pyz% -d7*sin(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6)) %3
% n8=(Rz*(Py-Ry*a8)-Ry*(Pz-Rz*a8)); %Ryz% -d7*cos(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6)) %4
% ---
% m8=(ax*(Py-Ry*a8)-ay*(Px-Rx*a8)); %Pxy% d7*sin(th8)*sin(th5)*sin(th6) %5
% n8=(Ry*(Px-Rx*a8)-Rx*(Py-Ry*a8)); %Rxy% d7*cos(th8)*sin(th5)*sin(th6) %6
% m8=(az*(Py-Ry*a8)-ay*(Pz-Rz*a8)); %Pzy% d7*sin(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6)) %7
% n8=(Ry*(Pz-Rz*a8)-Rz*(Py-Ry*a8)); %Rzy% d7*cos(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6)) %8
% ---
% m8=(ay*(Px-Rx*a8)-ax*(Py-Ry*a8)); %Pyx% -d7*sin(th8)*sin(th5)*sin(th6) %9
% n8=(Rx*(Py-Ry*a8)-Ry*(Px-Rx*a8)); %Ryx% -d7*cos(th8)*sin(th5)*sin(th6) %10
% m8=(az*(Px-Rx*a8)-ax*(Pz-Rz*a8)); %Pzx% d7*sin(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6)) %11
% n8=(Rx*(Pz-Rz*a8)-Rz*(Px-Rx*a8)); %Rzx% d7*cos(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6)) %12

%����th9(1)�� th9(2)����Ӧ�Ĺ�ϵ��
% th9=th9+pi;
Rx=nx*cos(th9)-ox*sin(th9); 
Ry=ny*cos(th9)-oy*sin(th9); 
Rz=nz*cos(th9)-oz*sin(th9); 
% m82=(ax*(Pz-Rz*a8)-az*(Px-Rx*a8)); %Pxz% -(2*a8 + d7*sin(th8))*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6))
% n82=(Rz*(Px-Rx*a8)-Rx*(Pz-Rz*a8)); %Rxz%           d7*cos(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6))
% m82=(ay*(Pz-Rz*a8)-az*(Py-Ry*a8)); %Pyz% -(2*a8 + d7*sin(th8))*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6))
% n82=(Rz*(Py-Ry*a8)-Ry*(Pz-Rz*a8)); %Ryz%           d7*cos(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6))
% ---
% m82=(ax*(Py-Ry*a8)-ay*(Px-Rx*a8)); %Pxy% (2*a8 + d7*sin(th8))*sin(th5)*sin(th6)
% n82=(Ry*(Px-Rx*a8)-Rx*(Py-Ry*a8)); %Rxy%         -d7*cos(th8)*sin(th5)*sin(th6)
% m82=(az*(Py-Ry*a8)-ay*(Pz-Rz*a8)); %Pzy% (2*a8 + d7*sin(th8))*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6))
% n82=(Ry*(Pz-Rz*a8)-Rz*(Py-Ry*a8)); %Rzy%         -d7*cos(th8)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6))
% ---
% m82=(ay*(Px-Rx*a8)-ax*(Py-Ry*a8)); %Pyx% -(2*a8 + d7*sin(th8))*sin(th5)*sin(th6)
% n82=(Rx*(Py-Ry*a8)-Ry*(Px-Rx*a8)); %Ryx%           d7*cos(th8)*sin(th5)*sin(th6) 
% m82=(az*(Px-Rx*a8)-ax*(Pz-Rz*a8)); %Pzx% (2*a8 + d7*sin(th8))*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6))
% n82=(Rx*(Pz-Rz*a8)-Rz*(Px-Rx*a8)); %Rzx%         -d7*cos(th8)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6)) 
% simplify(m82)
% simplify(n82)
% d7*sin(th8) != (2*a8+d7*sin(th8)) �� (cos(th6)*sin(th4)+cos(th4)*cos(th5)*sin(th6))=0�� sin(th5)*sin(th6)=0




%% d7=m7/n7
% �����Ǻ���������simplify������Ȼ�������ˣ��㿴ԭ12��ʽ�飬������ d7*cos(th4)*sin(th5)
% (Px-a8*Rx)==-d7*cos(th4)*sin(th5)
% (Py-a8*Ry)==-d7*sin(th4)*sin(th5)
% (Pz-a8*Rz)== d7*cos(th5)
th9=th9+pi;
m71=px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9));          m=simplify(m71) %-d7*cos(th4)*sin(th5)
% d7*cos(th4)*sin(th5) + 2*a8*cos(th4)*sin(th5)*sin(th8) - 2*a8*sin(th4)*sin(th6)*cos(th8) + 2*a8*cos(th4)*cos(th5)*cos(th6)*cos(th8)
n71=ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)); n=simplify(n71) %-cos(th4)*sin(th5)
% cos(2*th8)*cos(th4)*sin(th5) + sin(2*th8)*(sin(th4)*sin(th6) - cos(th4)*cos(th5)*cos(th6))
% m72=py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9));          m=simplify(m72) %-d7*sin(th4)*sin(th5)
% % d7*sin(th4)*sin(th5) + 2*a8*sin(th4)*sin(th5)*sin(th8) + 2*a8*cos(th4)*sin(th6)*cos(th8) + 2*a8*sin(th4)*cos(th5)*cos(th6)*cos(th8)
% n72=ay*cos(th8)+sin(th8)*(ny*cos(th9)-oy*sin(th9)); n=simplify(n72) %-sin(th4)*sin(th5)
% cos(2*th8)*sin(th4)*sin(th5) - sin(2*th8)*(cos(th4)*sin(th6) + *sin(th4)*cos(th5)*cos(th6))
% m73=pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9));          m=simplify(m73) %d7*cos(th5)
% d7*cos(th5) + 2*a8*cos(th5)*sin(th8) - 2*a8*sin(th5)*cos(th6)*cos(th8)
% n73=az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9)); n=simplify(n73) %cos(th5)
% cos(2*th8)cos(th5) + sin(2*th8)*sin(th5)*cos(th6)

% ����th9 th8����Ȼ�����ֽ��������ϣ���Ӧ��m71(1) n71(1) m71(2) n71(2) m71(3) n71(3) m71(4) n71(4);
% ��th9 th8��ϵ��֪m71 n71��ϵ
% th9 ; th8 ��
% th9+pi ; th8 ��
% th9 ; th8+pi ��
% th9+pi ; th8+pi ��

% ����n7�Ƿ�Ϊ�㣬��n71(1)=0��n71(2)=0�Ĺ�ϵ��
% m7 n7��������֪th91 th81 �ɵ� n71=-cos(th4)*sin(th5)���趨th4 th5�ɴ���
% ȷ��th81 th82��ϵ���ɵ� 

% th9 ��
% m71(1)==px-a9*nx-a8*(nx*cos(th9)-ox*sin(th9));          %m=simplify(m71) %-d7*cos(th4)*sin(th5)
% n71(1)==ax*cos(th8)+sin(th8)*(nx*cos(th9)-ox*sin(th9)); %n=simplify(n71) %-cos(th4)*sin(th5)
% th9=th9+pi; ��
% m71(2)==(d7+2*a8*sin(th8))*cos(th4)*sin(th5) - 2*a8*cos(th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))==0
% n71(2)==        cos(2*th8)*cos(th4)*sin(th5) +    sin(2*th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6))==0
% ������֣���th8�Ĺ�ϵ % k1=(d7+2*a8*sin(th8))*sin(2*th8) + 2*a8*cos(th8)*cos(2*th8);k=simplify(k1); %2*(d7*sin(th8)+a8)*cos(th8)

% m72(1)==py-a9*ny-a8*(ny*cos(th9)-oy*sin(th9));          m=simplify(m72) %-d7*sin(th4)*sin(th5)
% n72(1)==ay*cos(th8)+sin(th8)*(ny*cos(th9)-oy*sin(th9)); n=simplify(n72) %-sin(th4)*sin(th5)
% m72(2)==(d7+2*a8*sin(th8))*sin(th4)*sin(th5) + 2*a8*cos(th8)*(cos(th4)*sin(th6)+sin(th4)*cos(th5)*cos(th6))==0
% n72(2)==        cos(2*th8)*sin(th4)*sin(th5) -    sin(2*th8)*(cos(th4)*sin(th6)+sin(th4)*cos(th5)*cos(th6))==0
% ������� % k2=-(d7+2*a8*sin(th8))*sin(2*th8) - 2*a8*cos(th8)*cos(2*th8);k=simplify(k2); %- d7*sin(2*th8) - 2*a8*cos(th8)

% m73=pz-a9*nz-a8*(nz*cos(th9)-oz*sin(th9));          m=simplify(m73) %d7*cos(th5)
% n73=az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9)); n=simplify(n73) %cos(th5)
% m73(2)==(d7+2*a8*sin(th8))*cos(th5) - 2*a8*cos(th8)*sin(th5)*cos(th6)==0
% n73(2)==        cos(2*th8)*cos(th5) +    sin(2*th8)*sin(th5)*cos(th6)==0

% ������� % k3=(d7+2*a8*sin(th8))*sin(2*th8) + 2*a8*cos(th8)*cos(2*th8);k=simplify(k3);

% ����ֻ��ͬʱ����th9(1:2)
% a=cos(2*th8)*cos(th4)*sin(th5) + sin(2*th8)*(sin(th4)*sin(th6)-cos(th4)*cos(th5)*cos(th6));
% b=cos(2*th8)*sin(th4)*sin(th5) - sin(2*th8)*(cos(th4)*sin(th6)+sin(th4)*cos(th5)*cos(th6));
% c=cos(2*th8)*cos(th5) + sin(2*th8)*sin(th5)*cos(th6);
% d=a^2+b^2+c^2 %��1

%% th5
% syms az nz oz; %cos(th5)��ֵ
% fprintf("cos(th51) ");c5=az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9))
% th9=th9+pi;
% fprintf("cos(th52) ");c5=az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9))
% th9=th9-pi;
% th8=th8+pi;
% fprintf("cos(th53) ");c5=az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9))
% th9=th9+pi;
% fprintf("cos(th54) ");c5=az*cos(th8)+sin(th8)*(nz*cos(th9)-oz*sin(th9))
% c=simplify(c5) %

% th8=th8+pi;
n51=az*cos(th8) + sin(th8)*(nz*cos(th9) - oz*sin(th9))%==1 %th81 th91
n52=az*cos(th8) - sin(th8)*(nz*cos(th9) - oz*sin(th9))%==1 %th82 th91+pi
% ע�� th81��th82������ֱ��������֪����
simplify(n51) %cos(2*th8)*cos(th5) + sin(2*th8)*sin(th5)*cos(th6)==1
% ��ʱ��c6=1,
simplify(n52) %cos(th5)==1


% m4=ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9));
% n4=ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9));
% m6=-oz*cos(th9)-nz*sin(th9);
% n6=-az*sin(th8) + cos(th8)*(nz*cos(th9)-oz*sin(th9));

m46=simplify(-ox*cos(th9)-nx*sin(th9)); %-(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6)) �ֻ�ȥ��
n46=simplify(oy*cos(th9)+ny*sin(th9)); %-(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6))

m4=simplify(ay*cos(th8)+sin(th8)*Ry); %sin(th4)*sin(th5)
n4=simplify(ax*cos(th8)+sin(th8)*Rx); %cos(th4)*sin(th5)

%% �����ռ�

Px=d7*(ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))) +a8*(nx*cos(th9)-ox*sin(th9));
Py=d7*(ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))) +a8*(ny*cos(th9)-oy*sin(th9));
Pz=d7*(az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))) +a8*(nz*cos(th9)-oz*sin(th9));
P2=Px^2+Py^2+Pz^2;
% P2=simplify(P2)% a8^2 + 2*sin(th8)*a8*d7 + d7^2


px=d7*(ax*cos(th8) + sin(th8)*(nx*cos(th9)-ox*sin(th9))) +a9*nx+a8*(nx*cos(th9)-ox*sin(th9));
py=d7*(ay*cos(th8) + sin(th8)*(ny*cos(th9)-oy*sin(th9))) +a9*ny+a8*(ny*cos(th9)-oy*sin(th9));
pz=d7*(az*cos(th8) + sin(th8)*(nz*cos(th9)-oz*sin(th9))) +a9*nz+a8*(nz*cos(th9)-oz*sin(th9));
P3=px^2+py^2+pz^2; %��ô���Ǹ��ʴ�����
% simplify(P3) %a8^2 + a9^2 + d7^2 + 2*a8*d7*sin(th8) + a8*a9*(1/cos(th9)) + a8*a9*cos(2*th9)*(1/cos(th9)) + a9*d7*sin(th8)*(1/cos(th9)) + a9*d7*cos(2*th9)*sin(th8)*(1/cos(th9))
% ��һ������

%
% m=(2*a9*(a8 - a9*(1/cos(th9)^2)^(1/2) + d7*sin(th8)))/(1/cos(th9)^2)^(1/2)
% n=(2*a9*(a8 - a9*(1/cos(th9)) + d7*sin(th8)))*cos(th9)
% m=a8^2 + a9^2 + d7^2 + 2*a8*d7*sin(th8) + a8*a9*(1/cos(th9)^2)^(1/2) + a9*d7*sin(th8)*(1/cos(th9)^2)^(1/2) + a8*a9*cos(2*th9)*(1/cos(th9)^2)^(1/2) + a9*d7*cos(2*th9)*sin(th8)*(1/cos(th9)^2)^(1/2)
% n=a8^2 + a9^2 + d7^2 + 2*a8*d7*sin(th8) + a8*a9*(1/cos(th9)) + a8*a9*cos(2*th9)*(1/cos(th9)) + a9*d7*sin(th8)*(1/cos(th9)) + a9*d7*cos(2*th9)*sin(th8)*(1/cos(th9))




