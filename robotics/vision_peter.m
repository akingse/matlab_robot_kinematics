clc;clear all;
format short g;
%% vision
% 10 ����ɫ��
%{
lambda=[300:10:1000]*1e-9; %�����Ķ��巶Χ��
for T=1000:1000:6000 %�ɼ���310nm-750nm
%     plot(lambda*1e9,blackbody(lambda,T));hold all; %������׺�����
end
% ���׵ı�ʾ
lamp=blackbody(lambda,2600);
sun=blackbody(lambda,6500);
% plot(lambda*1e9,[lamp/max(lamp),sun/max(sun)]);
% ��������������-�����������գ�
% ɫ�ʣ��۾��������Ӿ�ϸ������׶��ϸ�����ر����ɫ�з�Ӧ���Ӹ�ϸ����ǿ���ַ�Ӧ��
human=luminos(lambda);
% plot(lambda*1e9,human);
% ��ʾ��ÿ�����ز����ɱ���ԭɫ����׼CIE��ԭɫ
% ɫ�ȿռ�
% ��ɫ�̼���ֵ������������ɫ������������
% C=RR+GG+BB;
lambda=[400:10:700]*1e-9;
cmf=cmfrgb(lambda);
% plot(lambda*1e9,cmf);
orange=cmfrgb(600e-9); %����600nm�ĳȹ⣬��Ҫ��ԭɫ
[r,g]=lambda2rg([400:700]*1e9);
% plot(r,g);
rg_addticks;
% XYZ �鹹�ķ�������ɫϵͳ��XZ����0���ȣ�������ȫ��Y�ṩ��
cmf=cmfxyz(lambda);
% plot(lambda*1e9,cmf);%���׹켣����
% xycolorspace
lambda2xy(550e-9);
colorname('blue');colorname('blue','xy');
colorspace('RGB->HSV',[1 0 0]);% ʵ�ֲ�ͬ��ɫ�ռ�֮���ת����RGB->ɫ�౥�Ͷ�ǿ�ȣ�
% ��ƽ�⣬������Դɫ�£�rgb1=J*rgb
% -------------------------------
flowers=iread('flowers4.png','double','gamma','sRGB'); %640*426
hsv=colorspace('RGB->HSV',flowers);
% idisp(hsv(:,:,1)); %ɫ��
% idisp(hsv(:,:,2)); %���Ͷ�
XYZ=colorspace('RGB->XYZ',flowers);
[x,y]=tristim2cc(XYZ);
xbins=[0 0.01 100];
ybins=[0 0.01 100];
% [h vx vy]=hist2d(x,y,xbins,ybins); %ͼ
% xycolorspace; %bug
% hold on;
% contour(vx,vy,h);
% [cls,cxy]=colorkmeans(flowers,7);
%}
% 11ͼ���γ�
% С�׳��������������ǰ���ά����ͶӰ�ڶ�άƽ���ϣ�ʧȥ�������Ϣ����Ϊ͸��ͶӰ
cam=CentralCamera('focal',0.015) %����ͶӰ�����ģ�ͣ�15mm͸��
P=[0.3 0.4 3]';
cam.project(P)
% 
% 
%% 
% im=imread('IMG_20170619_121931.jpg'); 
% figure,imshow(im);
% title('Դͼ��');
%  
% %ѡȡͼ���ϵ�һ�������ε�ROI����  
% im0 = imcrop(im,[1198 54 2210 2210]);  
% figure,
% imshow(im0,'DisplayRange',[])  
% title('ѡȡROI���ͼ��');
 
%%%%%��˹�˲�%%%%%
sigma = 1.6;
gausFilter = fspecial('gaussian',[5 5],sigma);
blur=imfilter(im0,gausFilter,'replicate');
figure,imshow(blur);
title('��˹�˲����ͼ��');
 
level = graythresh(blur);   %%%ostu�㷨����ֵ���ж�ֵ��
im1 = im2bw(blur,level);
figure,imshow(im1);
title('��ֵͼ��');
 
bw1=bwlabel(im1,8);
stats=regionprops(bw1,'Area');
bw2 = ismember(bw1, find([stats.Area]) == max([stats.Area])); %%�ҵ�������Ķ���
figure,imshow(bw2);
title('��ֵͼ��');
[B L] = bwboundaries(bw2);  %Ѱ�ұ�Ե����������,L�Ǳ�Ǿ���
figure,   %%%�����հ׵�ͼ��
hold on
for k = 1:length(B)
    boundary = B{k};  %B��һ����Ԫ���飬������B{k}
    plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2);%% ������Ե
end%����ѭ����ʾ�������
 
 
%% 
%%%%%Ѱ��houghԲ��Բ��%%%%%%%
Rmin = 100;
Rmax = 200;
[centersBright, radiiBright] = imfindcircles(im1,[Rmin Rmax],'ObjectPolarity' ,'bright');
viscircles(centersBright,radiiBright,'EdgeColor','b');
hold on 
plot(centersBright(1),centersBright(2),'*');
hold off;
 
%% �ȷ�Բ
R=1050; t=0:pi/20:2*pi;
x=R*cos(t);
y=R*sin(t);
axis equal
n=36;a=2*pi/n;
for k=0:n-1
    hold on         %%%%%����(1077,1055)�ǰ뾶��1090�ǰ뾶����ֱ�ߵĳ���
    plot([1077-1090*cos(pi+k*a),1077+1090*cos(pi+k*a)],[1114-1090*sin(pi+k*a),1114+1090*sin(pi+k*a)])
end

