clc;clear all;
format short g;
%% vision
% 10 光与色彩
%{
lambda=[300:10:1000]*1e-9; %波长的定义范围；
for T=1000:1000:6000 %可见光310nm-750nm
%     plot(lambda*1e9,blackbody(lambda,T));hold all; %黑体光谱函数；
end
% 光谱的表示
lamp=blackbody(lambda,2600);
sun=blackbody(lambda,6500);
% plot(lambda*1e9,[lamp/max(lamp),sun/max(sun)]);
% 地球大气层的吸收-物体表面的吸收；
% 色彩，眼睛的两种视觉细胞，视锥形细胞对特别的颜色有反应，视感细胞对强光又反应；
human=luminos(lambda);
% plot(lambda*1e9,human);
% 显示器每个像素产生可变三原色，标准CIE三原色
% 色度空间
% 三色刺激的值不仅描述了颜色，还包含亮度
% C=RR+GG+BB;
lambda=[400:10:700]*1e-9;
cmf=cmfrgb(lambda);
% plot(lambda*1e9,cmf);
orange=cmfrgb(600e-9); %创造600nm的橙光，需要的原色
[r,g]=lambda2rg([400:700]*1e9);
% plot(r,g);
rg_addticks;
% XYZ 虚构的非物理三色系统；XZ具有0亮度，亮度完全由Y提供，
cmf=cmfxyz(lambda);
% plot(lambda*1e9,cmf);%光谱轨迹绘制
% xycolorspace
lambda2xy(550e-9);
colorname('blue');colorname('blue','xy');
colorspace('RGB->HSV',[1 0 0]);% 实现不同颜色空间之间的转换，RGB->色相饱和度强度；
% 白平衡，调整光源色温，rgb1=J*rgb
% -------------------------------
flowers=iread('flowers4.png','double','gamma','sRGB'); %640*426
hsv=colorspace('RGB->HSV',flowers);
% idisp(hsv(:,:,1)); %色相
% idisp(hsv(:,:,2)); %饱和度
XYZ=colorspace('RGB->XYZ',flowers);
[x,y]=tristim2cc(XYZ);
xbins=[0 0.01 100];
ybins=[0 0.01 100];
% [h vx vy]=hist2d(x,y,xbins,ybins); %图
% xycolorspace; %bug
% hold on;
% contour(vx,vy,h);
% [cls,cxy]=colorkmeans(flowers,7);
%}
% 11图像形成
% 小孔成像，相机成像过程是把三维世界投影在二维平面上，失去了深度信息，称为透视投影
cam=CentralCamera('focal',0.015) %中央投影照相机模型，15mm透镜
P=[0.3 0.4 3]';
cam.project(P)
% 
% 
%% 
% im=imread('IMG_20170619_121931.jpg'); 
% figure,imshow(im);
% title('源图像');
%  
% %选取图像上的一个正方形的ROI区域；  
% im0 = imcrop(im,[1198 54 2210 2210]);  
% figure,
% imshow(im0,'DisplayRange',[])  
% title('选取ROI后的图像');
 
%%%%%高斯滤波%%%%%
sigma = 1.6;
gausFilter = fspecial('gaussian',[5 5],sigma);
blur=imfilter(im0,gausFilter,'replicate');
figure,imshow(blur);
title('高斯滤波后的图像');
 
level = graythresh(blur);   %%%ostu算法求阈值进行二值化
im1 = im2bw(blur,level);
figure,imshow(im1);
title('二值图像');
 
bw1=bwlabel(im1,8);
stats=regionprops(bw1,'Area');
bw2 = ismember(bw1, find([stats.Area]) == max([stats.Area])); %%找到面积最大的对象
figure,imshow(bw2);
title('二值图像');
[B L] = bwboundaries(bw2);  %寻找边缘，不包括孔,L是标记矩阵
figure,   %%%创建空白的图像
hold on
for k = 1:length(B)
    boundary = B{k};  %B是一个胞元数组，所以是B{k}
    plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2);%% 画出边缘
end%整个循环表示的是描边
 
 
%% 
%%%%%寻找hough圆的圆心%%%%%%%
Rmin = 100;
Rmax = 200;
[centersBright, radiiBright] = imfindcircles(im1,[Rmin Rmax],'ObjectPolarity' ,'bright');
viscircles(centersBright,radiiBright,'EdgeColor','b');
hold on 
plot(centersBright(1),centersBright(2),'*');
hold off;
 
%% 等分圆
R=1050; t=0:pi/20:2*pi;
x=R*cos(t);
y=R*sin(t);
axis equal
n=36;a=2*pi/n;
for k=0:n-1
    hold on         %%%%%其中(1077,1055)是半径，1090是半径，即直线的长度
    plot([1077-1090*cos(pi+k*a),1077+1090*cos(pi+k*a)],[1114-1090*sin(pi+k*a),1114+1090*sin(pi+k*a)])
end

