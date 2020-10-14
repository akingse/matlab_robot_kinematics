%% initialize
clc; close all; clear all;
format shortg; format compact;

%% subplot

% �ָ�figure������������ϵ
% �﷨
% 
% h = subplot(m,n,p) or subplot(mnp)
%        subplot(m,n,p,'replace')
%        subplot(m,n,P)
%        subplot(h)
%        subplot('Position',[left bottom width height])
%        subplot(..., prop1, value1, prop2, value2, ...)
%        h = subplot(...) 


%% ����plot�÷���
% plot ���������������Ӷ�������ͼ�� ��Ԫ ��ɫ ��Ԫ ͼ����̬
% loglog x��y�ᶼȡ������Ƚ���ͼ�� y ��ɫ . ��
% semilogx x�����ڶ�����ȣ�y�����Ա�Ȼ���ͼ�� k ��ɫ o Բ
% semilogy y�����ڶ�����ȣ�x�����Ա�Ȼ���ͼ�� w ��ɫ x x
% title ��ͼ�μӱ��� b ��ɫ + +
% xlabel ��x��ӱ�� g ��ɫ * *
% ylabel ��y��ӱ�� r ��ɫ - ʵ��
% text ��ͼ��ָ����λ���ϼ��ı��ַ��� c ����ɫ : ����
% gtext ������λ���ϼ��ı��ַ��� m ����ɫ -. ������
% grid �������� -- ����
% hold on �����������ѻ��õ�ͼ��������µ�ͼ��


%{
1�� x=0:0.001:10; % 0��10��1000����(ÿ��0.001��һ����)��x����
y=sin(x); % ��Ӧ��y����
plot(x,y); % ��ͼ
ע��matlab��ͼʵ���Ͼ���������ߣ���������ȡ�ò��ܣ��������ͳ�������ͼ��������֮
2�� Y=sin(10*x);
plot(x,y,'r:',x,Y,'b') % ͬʱ����������
3�� ��Ҫ�ı���ɫ��������Ժ����������ִ����ɣ�
x=0:0.01:10;
plot(x,sin(x),'r')
4�� ��Ҫͬʱ�ı���ɫ��ͼ����̬��Line style����Ҳ��������Ժ����������ִ����ɣ�
plot(x,sin(x),'r*')
5�� ��axis([xmin,xmax,ymin,ymax])����������ͼ��ķ�Χ
axis([0,6,-1.5,1])
6��MATLABҲ�ɶ�ͼ�μ��ϸ���ע���봦�������ϱ�
xlabel('x��'); % x��ע��
ylabel('y��'); % y��ע��
title('���Һ���'); % ͼ�α���
legend('y = cos(x)'); % ͼ��ע��
gtext('y = cos(x)'); % ͼ��ע�� ,����궨λע��λ��
grid on; % ��ʾ����
7������Բ
a = [0:pi/50:2*pi]'; %�Ƕ�
X = cos(a)*3; %��������
Y = sin(a)*2;
plot(X,Y);
xlabel('x'), ylabel('y');
title('��Բ')
8�� ���ƺ��� ��0 �� x �� 1ʱ�����ߡ�
x=0:0.1:1
y=x.*exp(-x) %Ϊʲô�õ����㣿�����û�����
plot(x,y),xlabel('x'),ylabel('y'),title('y=x*exp(-x)')
9������˥�������� �����İ����� �� ��t ��ȡֵ��Χ��[0, 4��] ��
t=0:pi/50:4*pi;
y0=exp(-t/3);
y=exp(-t/3).*sin(3*t);
plot(t,y,'-r',t,y0,':b',t,-y0,':b') % -r��ʾ��ɫʵ�ߣ�:b��ʾ��ɫ���ߣ����ϱ�
grid
10�� ��ͬһ�������Ͻ�����������ϵ, ��subplot(m,n,p)�����һ������ֳ�m��n��ͼ������, p����ǰ������ţ���ÿ�������зֱ�һ��ͼ,��
x=linspace(0,2*pi,30); y=sin(x); z=cos(x);
u=2*sin(x).*cos(x); v=sin(x)./cos(x);
subplot(2,2,1),plot(x,y),axis([0 2*pi -1 1]),title('sin(x)')
subplot(2,2,2),plot(x,z),axis([0 2*pi -1 1]),title('cos(x)')
subplot(2,2,3),plot(x,u),axis([0 2*pi -1 1]),title('2sin(x)cos(x)')
subplot(2,2,4),plot(x,v),axis([0 2*pi -20 20]),title('sin(x)/cos(x)')
%}
figure(1);
x=0:0.001:10; % 0��10��1000����(ÿ��0.001��һ����)��x����
y=sin(x); % ��Ӧ��y����
plot(x,y); % ��ͼ

figure(2);
Y=sin(2*x);
plot(x,y,'r:',x,Y,'b') % ͬʱ����������

figure(3);
x=0:0.01:10;
plot(x,sin(x),'r')
plot(x,sin(x)+1,'r.')

% 5�� ��axis([xmin,xmax,ymin,ymax])����������ͼ��ķ�Χ
% axis([0,6,-1.5,1])
% 6��MATLABҲ�ɶ�ͼ�μ��ϸ���ע���봦�������ϱ�
xlabel('x��'); % x��ע��
ylabel('y��'); % y��ע��
title('���Һ���'); % ͼ�α���
legend('y = cos(x)'); % ͼ��ע��
% gtext('y = cos(x)'); % ͼ��ע�� ,����궨λע��λ��
grid on; % ��ʾ����
% 7������Բ

figure(4);
a = [0:pi/50:2*pi]'; %�Ƕ�
X = cos(a)*3; %��������
Y = sin(a)*2;
plot(X,Y);
xlabel('x'), ylabel('y');
title('��Բ')

figure(5);
x=0:0.1:1;
y=x.*exp(-x); %Ϊʲô�õ����㣿�����û�����
plot(x,y),xlabel('x'),ylabel('y'),title('y=x*exp(-x)')
% 9������˥�������� �����İ����� �� ��t ��ȡֵ��Χ��[0, 4��] ��

figure(6);
t=0:pi/50:4*pi;
y0=exp(-t/3);
y=exp(-t/3).*sin(3*t);
plot(t,y,'-r',t,y0,':b',t,-y0,':b') % -r��ʾ��ɫʵ�ߣ�:b��ʾ��ɫ���ߣ����ϱ�
grid
% 10�� ��ͬһ�������Ͻ�����������ϵ, ��subplot(m,n,p)�����һ������ֳ�m��n��ͼ������, p����ǰ������ţ���ÿ�������зֱ�һ��ͼ,��
figure(7);
x=linspace(0,2*pi,30); y=sin(x); z=cos(x);
u=2*sin(x).*cos(x); v=sin(x)./cos(x);
subplot(2,2,1),plot(x,y),axis([0 2*pi -1 1]),title('sin(x)')
subplot(2,2,2),plot(x,z),axis([0 2*pi -1 1]),title('cos(x)')
subplot(2,2,3),plot(x,u),axis([0 2*pi -1 1]),title('2sin(x)cos(x)')
subplot(2,2,4),plot(x,v),axis([0 2*pi -20 20]),title('sin(x)/cos(x)')



%% 
figure(8);
x=linspace(0,2*pi,30);
y=cos(x);
% e=vpa(std(y)*ones(size(x)),2)
e=std(y)*ones(size(x));
errorbar(x,y,e);
% x=roots(p)
% p1=poly(x)
% syms x y z p;
% x=-20:1:20;y=-20:1:20;
% y=x.^2+2*x+1;
% z=x.^2+y.^2+x.*y;
% plot(x,z)

%% Բ
figure(9);
t=-pi:pi/100:pi;
x=2*sin(t);y=2*cos(t);
plot(x,y);
ezplot('x^2+y^2-4')

figure(10);
x=-10:0.1:10;
y=-10:0.1:10;
h=polyval(y,x);
plot(x,h);
xlabel('��עx');ylabel('��עy');


%% 
% [x,y,z]=peaks;
% subplot(121)
% [c,h]=contour(x,y,z);
% set(h,'linewidth',4,'linestyle',':')
% subplot(122)
% surf(x,y,z);

% D=4;
% n=2^D-1;
% [x,y,z]=sphere(n);
% c=hadamard(2^D);
% surf(x,y,z,c);
% colormap([0 1 1;1 1 0]);
% axis equal

% x=0:0.1:2*pi;
% y=sin(x);
% plot(x,y);
% A='hello matlab';
% disp(A)
% x=input('x=');
% y=input('y=');
% z=x;x=y;y=z;
% disp(x);
% disp(y);
% a=4;b=7;
% for i=1:1:4;%
%     b=b+1
%     if i>2 break;
%     end
%     a=a+2
% end
%% һ����ͼ

figure(11);
x=linspace(0,2*pi,30);%x=0:pi/15:2*pi;
y=sin(x); z=cos(x);
u=2*sin(x).*cos(x); v=sin(x)./cos(x);
subplot(2,2,1),plot(x,y),axis([0 2*pi -1 1]),title('sin(x)')
subplot(2,2,2),plot(x,z),axis([0 2*pi -1 1]),title('cos(x)')
subplot(2,2,3),plot(x,u),axis([0 2*pi -1 1]),title('2sin(x)cos(x)')
subplot(2,2,4),plot(x,v),axis([0 2*pi -20 20]),title('sin(x)/cos(x)')

%% �ֶκ���
figure(12);
x=0:0.01:10;
y=sqrt(x).*(x>=0&x<4)+2.*(x>=4&x<6)+(5-x/2).*(x>=6&x<8)+1.*(x>=8);
plot(x,y);
axis([0 10 0 2.2]) %�޶�xyͼ��Χ
grid on %������

%% ��άͼ��
% 11����ά�����ߣ�
figure(13);
t=0:pi/50:10*pi;
plot3(sin(t),cos(t),t) %��������
grid %�������

% 12�� 
figure(14);
t=linspace(0,20*pi, 501);
plot3(t.*sin(t), t.*cos(t), t); %ע����
% Ҳ����ͬʱ�����������ߣ���ʽ���ά������ƣ��Ȳ�������

% 13����mesh�������
% �����ɺ��� �γɵ�������״ͼ:
figure(15);
a=linspace(-2, 2, 25); % ��x���ϴӣ�-2��2��ȡ25��
b=linspace(-2, 2, 25); % ��y����ȡ25��
[x,y]=meshgrid(a, b); % x��y����21x21�ľ���
z=x.*exp(-x.^2-y.^2); % ���㺯��ֵ��zҲ��21x21�ľ���
mesh(x, y, z); % ����������״ͼ

% 14�� surf��mesh���÷����ƣ�
figure(16);
a=linspace(-2, 2, 25); % ��x����ȡ25��
b=linspace(-2, 2, 25); % ��y����ȡ25��
[x,y]=meshgrid(a, b); % x��y����21x21�ľ���
z=x.*exp(-x.^2-y.^2); % ���㺯��ֵ��zҲ��21x21�ľ���
surf(x, y, z); % ������������ͼ

%% Figure
% f = figure('Units','normalized','OuterPosition',[0 0.5 1 0.5]);
% 
% ʹ�÷�����subplot��m,n,p������subplot��m n p����
% subplot�ǽ����ͼ����һ��ƽ���ϵĹ��ߡ����У�m��ʾ��ͼ�ų�m�У�n��ʾͼ�ų�n�У�p��ʾͼ���ڵ�λ��
% ax1 = subplot(1,3,1);
% ax2 = subplot(1,3,2);
% ax3 = subplot(1,3,3);
%%Plot some surfaces on 1st subplot
% [X,Y,Z] = peaks(100); %[X,Y,Z] = peaks;
% z=peaks;Ĭ�Ϸ���һ��49*49�ľ���
% z=peaks(n);����һ��n*n�ľ���
% s = surf(ax1,X,Y,Z); hold(ax1,'on');
% p = surf(ax1,X,Y,zeros(size(X))); hold(ax1,'off');
% plotObjs = [s,p]
% %Copy plot objects to other 2 subplots
% copyobj(plotObjs,ax2);
% copyobj(plotObjs,ax3);
%%Set different viewing angle for each subplot
% view(ax1,0,90); title(ax1,'top');
% view(ax2,90,0); title(ax2,'left');
% view(ax3,0,0); title(ax3,'front');

% ttl={'����ͼ','����ͼ','����ͼ','��άͼ'};
% angle={[0,0],[-90,0],[0 90],[-37.5,30]};
% for i=1:4
% subplot(2,2,i);
% ezmesh('1/((1-X)^2+Y^2)^0.5+1/((1+X)^2+Y^2)^0.5',[-2 2]);
% view(angle{i});title(ttl{i});
% end

%% ����������ϣ�
% y1=polyfit(x,y,N) %���ﺯ��polyfit��һ���������ݵ���������ݵ��Ա������ڶ������������������������������϶���ʽ�Ľ���
% y1=polyfit(x,y,3)

% x=[0    0.3000    0.6000    0.9000    1.2000    1.5000    1.8000    2.1000    2.4000    2.7000    3.0000]
% y=[2.0000    2.3780    3.9440    7.3460   13.2320   22.2500   35.0480   52.2740   74.5760  102.6020  137.0000]
% % y1=polyfit(x,y,3)
% 
% for i=1:5
%     y2=polyfit(x,y,i);
%     Y=polyval(y2,x);%������Ϻ�����x����ֵ��
%     if sum((Y-y).^2)<0.1
%         c=i  
%         break;
%     end
% end

%%  ���²��޼�
% a = -0.11;
% b = 0.65;
a = -0.19;
b = 0.6557;
N = 80;
r = 2;
i = 0;
% for x0 = -2:0.01:2
%     for y0 = -2:0.01:2
%         x = x0;
%         y = y0;
%         if x0^2+y0^2 < r^2
%             i = i+1;
%            for n = 1:N
%                x1 = x*x-y*y+a;
%                y1 = 2*x*y+b;
%                x = x1;
%                y = y1;               
%            end
%            if (x*x+y*y) < r^2
%               plot(x0,y0,'b.');
%            end
%            hold on;
%         end
%     end
% end




