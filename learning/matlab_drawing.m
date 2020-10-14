%% initialize
clc; close all; clear all;
format shortg; format compact;

%% subplot

% 分割figure，创建子坐标系
% 语法
% 
% h = subplot(m,n,p) or subplot(mnp)
%        subplot(m,n,p,'replace')
%        subplot(m,n,P)
%        subplot(h)
%        subplot('Position',[left bottom width height])
%        subplot(..., prop1, value1, prop2, value2, ...)
%        h = subplot(...) 


%% 其他plot用法：
% plot 建立向量或矩阵各队队向量的图形 字元 颜色 字元 图线型态
% loglog x、y轴都取对数标度建立图形 y 黄色 . 点
% semilogx x轴用于对数标度，y轴线性标度绘制图形 k 黑色 o 圆
% semilogy y轴用于对数标度，x轴线性标度绘制图形 w 白色 x x
% title 给图形加标题 b 蓝色 + +
% xlabel 给x轴加标记 g 绿色 * *
% ylabel 给y轴加标记 r 红色 - 实线
% text 在图形指定的位置上加文本字符串 c 亮青色 : 点线
% gtext 在鼠标的位置上加文本字符串 m 锰紫色 -. 点虚线
% grid 打开网格线 -- 虚线
% hold on 命令用于在已画好的图形上添加新的图形


%{
1） x=0:0.001:10; % 0到10的1000个点(每隔0.001画一个点)的x座标
y=sin(x); % 对应的y座标
plot(x,y); % 绘图
注：matlab画图实际上就是描点连线，因此如果点取得不密，画出来就成了折线图，请试验之
2） Y=sin(10*x);
plot(x,y,'r:',x,Y,'b') % 同时画两个函数
3） 若要改变颜色，在座标对后面加上相关字串即可：
x=0:0.01:10;
plot(x,sin(x),'r')
4） 若要同时改变颜色及图线型态（Line style），也是在坐标对后面加上相关字串即可：
plot(x,sin(x),'r*')
5） 用axis([xmin,xmax,ymin,ymax])函数来调整图轴的范围
axis([0,6,-1.5,1])
6）MATLAB也可对图形加上各种注解与处理：（见上表）
xlabel('x轴'); % x轴注解
ylabel('y轴'); % y轴注解
title('余弦函数'); % 图形标题
legend('y = cos(x)'); % 图形注解
gtext('y = cos(x)'); % 图形注解 ,用鼠标定位注解位置
grid on; % 显示格线
7）画椭圆
a = [0:pi/50:2*pi]'; %角度
X = cos(a)*3; %参数方程
Y = sin(a)*2;
plot(X,Y);
xlabel('x'), ylabel('y');
title('椭圆')
8） 绘制函数 在0 ≤ x ≤ 1时的曲线。
x=0:0.1:1
y=x.*exp(-x) %为什么用点运算？若不用会怎样
plot(x,y),xlabel('x'),ylabel('y'),title('y=x*exp(-x)')
9）画出衰减振荡曲线 与它的包络线 及 。t 的取值范围是[0, 4π] 。
t=0:pi/50:4*pi;
y0=exp(-t/3);
y=exp(-t/3).*sin(3*t);
plot(t,y,'-r',t,y0,':b',t,-y0,':b') % -r表示红色实线，:b表示蓝色点线，看上表
grid
10） 在同一个画面上建立几个坐标系, 用subplot(m,n,p)命令；把一个画面分成m×n个图形区域, p代表当前的区域号，在每个区域中分别画一个图,如
x=linspace(0,2*pi,30); y=sin(x); z=cos(x);
u=2*sin(x).*cos(x); v=sin(x)./cos(x);
subplot(2,2,1),plot(x,y),axis([0 2*pi -1 1]),title('sin(x)')
subplot(2,2,2),plot(x,z),axis([0 2*pi -1 1]),title('cos(x)')
subplot(2,2,3),plot(x,u),axis([0 2*pi -1 1]),title('2sin(x)cos(x)')
subplot(2,2,4),plot(x,v),axis([0 2*pi -20 20]),title('sin(x)/cos(x)')
%}
figure(1);
x=0:0.001:10; % 0到10的1000个点(每隔0.001画一个点)的x座标
y=sin(x); % 对应的y座标
plot(x,y); % 绘图

figure(2);
Y=sin(2*x);
plot(x,y,'r:',x,Y,'b') % 同时画两个函数

figure(3);
x=0:0.01:10;
plot(x,sin(x),'r')
plot(x,sin(x)+1,'r.')

% 5） 用axis([xmin,xmax,ymin,ymax])函数来调整图轴的范围
% axis([0,6,-1.5,1])
% 6）MATLAB也可对图形加上各种注解与处理：（见上表）
xlabel('x轴'); % x轴注解
ylabel('y轴'); % y轴注解
title('余弦函数'); % 图形标题
legend('y = cos(x)'); % 图形注解
% gtext('y = cos(x)'); % 图形注解 ,用鼠标定位注解位置
grid on; % 显示格线
% 7）画椭圆

figure(4);
a = [0:pi/50:2*pi]'; %角度
X = cos(a)*3; %参数方程
Y = sin(a)*2;
plot(X,Y);
xlabel('x'), ylabel('y');
title('椭圆')

figure(5);
x=0:0.1:1;
y=x.*exp(-x); %为什么用点运算？若不用会怎样
plot(x,y),xlabel('x'),ylabel('y'),title('y=x*exp(-x)')
% 9）画出衰减振荡曲线 与它的包络线 及 。t 的取值范围是[0, 4π] 。

figure(6);
t=0:pi/50:4*pi;
y0=exp(-t/3);
y=exp(-t/3).*sin(3*t);
plot(t,y,'-r',t,y0,':b',t,-y0,':b') % -r表示红色实线，:b表示蓝色点线，看上表
grid
% 10） 在同一个画面上建立几个坐标系, 用subplot(m,n,p)命令；把一个画面分成m×n个图形区域, p代表当前的区域号，在每个区域中分别画一个图,如
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

%% 圆
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
xlabel('标注x');ylabel('标注y');


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
%% 一窗多图

figure(11);
x=linspace(0,2*pi,30);%x=0:pi/15:2*pi;
y=sin(x); z=cos(x);
u=2*sin(x).*cos(x); v=sin(x)./cos(x);
subplot(2,2,1),plot(x,y),axis([0 2*pi -1 1]),title('sin(x)')
subplot(2,2,2),plot(x,z),axis([0 2*pi -1 1]),title('cos(x)')
subplot(2,2,3),plot(x,u),axis([0 2*pi -1 1]),title('2sin(x)cos(x)')
subplot(2,2,4),plot(x,v),axis([0 2*pi -20 20]),title('sin(x)/cos(x)')

%% 分段函数
figure(12);
x=0:0.01:10;
y=sqrt(x).*(x>=0&x<4)+2.*(x>=4&x<6)+(5-x/2).*(x>=6&x<8)+1.*(x>=8);
plot(x,y);
axis([0 10 0 2.2]) %限定xy图像范围
grid on %网格线

%% 三维图形
% 11）三维螺旋线：
figure(13);
t=0:pi/50:10*pi;
plot3(sin(t),cos(t),t) %参数方程
grid %添加网格

% 12） 
figure(14);
t=linspace(0,20*pi, 501);
plot3(t.*sin(t), t.*cos(t), t); %注意点乘
% 也可以同时画出两条曲线，格式与二维情况类似，兹不举例。

% 13）用mesh命令画曲面
% 画出由函数 形成的立体网状图:
figure(15);
a=linspace(-2, 2, 25); % 在x轴上从（-2，2）取25点
b=linspace(-2, 2, 25); % 在y轴上取25点
[x,y]=meshgrid(a, b); % x和y都是21x21的矩阵
z=x.*exp(-x.^2-y.^2); % 计算函数值，z也是21x21的矩阵
mesh(x, y, z); % 画出立体网状图

% 14） surf和mesh的用法类似：
figure(16);
a=linspace(-2, 2, 25); % 在x轴上取25点
b=linspace(-2, 2, 25); % 在y轴上取25点
[x,y]=meshgrid(a, b); % x和y都是21x21的矩阵
z=x.*exp(-x.^2-y.^2); % 计算函数值，z也是21x21的矩阵
surf(x, y, z); % 画出立体曲面图

%% Figure
% f = figure('Units','normalized','OuterPosition',[0 0.5 1 0.5]);
% 
% 使用方法：subplot（m,n,p）或者subplot（m n p）。
% subplot是将多个图画到一个平面上的工具。其中，m表示是图排成m行，n表示图排成n列，p表示图所在的位置
% ax1 = subplot(1,3,1);
% ax2 = subplot(1,3,2);
% ax3 = subplot(1,3,3);
%%Plot some surfaces on 1st subplot
% [X,Y,Z] = peaks(100); %[X,Y,Z] = peaks;
% z=peaks;默认返回一个49*49的矩阵
% z=peaks(n);返回一个n*n的矩阵
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

% ttl={'主视图','左视图','俯视图','三维图'};
% angle={[0,0],[-90,0],[0 90],[-37.5,30]};
% for i=1:4
% subplot(2,2,i);
% ezmesh('1/((1-X)^2+Y^2)^0.5+1/((1+X)^2+Y^2)^0.5',[-2 2]);
% view(angle{i});title(ttl{i});
% end

%% 函数命令拟合：
% y1=polyfit(x,y,N) %这里函数polyfit第一个参数传递的是拟合数据的自变量，第二个参数是因变量，第三个参数是拟合多项式的阶数
% y1=polyfit(x,y,3)

% x=[0    0.3000    0.6000    0.9000    1.2000    1.5000    1.8000    2.1000    2.4000    2.7000    3.0000]
% y=[2.0000    2.3780    3.9440    7.3460   13.2320   22.2500   35.0480   52.2740   74.5760  102.6020  137.0000]
% % y1=polyfit(x,y,3)
% 
% for i=1:5
%     y2=polyfit(x,y,i);
%     Y=polyval(y2,x);%计算拟合函数在x处的值。
%     if sum((Y-y).^2)<0.1
%         c=i  
%         break;
%     end
% end

%%  曼德勃罗集
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




