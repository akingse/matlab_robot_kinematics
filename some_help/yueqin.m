%% initialize
clc; close all; clear all;
format shortg; format compact;


%% code area
delimiterIn = ' '; 
headerlinesIn = 2;
filename='C:\Users\Akingse\Documents\MATLAB\code\1.asc';
% filename='D:\Commun\WeChat Files\WeChat Files\wxid_vpyiiw64hbfq42\FileStorage\File\2021-01\SDK0900\8.asc';
% a=csvread(filename);
A = importdata(filename, delimiterIn, headerlinesIn); 
data=A.data;

x=data(:,1);%length(x)
y=data(:,2);%length(y)
z=data(:,3);
plot(x,z)

% lenz=length(zi);
% for i=1:lenz
%     Z(i,:)=zi;
% end
% % Z;
% [X,Y]=meshgrid(x,y);

% mesh(x,y,zi);
% colorbar %显示色阶的颜色栏

%peaks函数
% [x,y,z]=peaks(11);%只要数据足够多，就是光滑曲面；
% xyz为相同尺寸矩阵，索引位置一一对应，


% mesh(X,Y,Z);
% colorbar
% eps

%% 

N=length(z) %2067
% x=x(1:N-2);
% z=z(1:N-2);
% N=N-2;
Sigma_iz=0;

for i=0:N-1
    Sigma_iz=Sigma_iz+i*z(i+1);    
end

Sigma_z=0;
for i=0:N-1
    Sigma_z=Sigma_z+z(i+1);    
end

b1=(12*Sigma_iz-6*(N-1)*Sigma_z)/(N*(N+1)*(N-1))
b0=Sigma_z/N-b1*(N-1)/2

for i=0:N-1
    Z(i+1)=z(i+1)-b1*i-b0;    
end
% Z=Z'
% figure
x=1:1:N;%
% plot(x(1:N-2),z(1:N-2))
% plot(x(1:N),z(1:N))

% x=x(1:N-2);
% z=z(1:N-2);
% plot(x',z')
x=x';
% p=polyfit(x,z,1);
% plot(x,z,x,polyval(p,x));

% plot(x,Z);

% p=polyfit(x,Z,1);
% plot(x,Z,x,polyval(p,x));

% x=[1 2 3 4 5];
% y=[3 6 8 11 15];
% p=polyfit(x,y,1);
% plot(x,y,x,polyval(p,x));




%%
%{
B = [1 0 0;1 1 0;0 1 0];
q = [30 30 30];
p = size(B,1);
A = [];
for k=1:p-1
n = q(k)+2;
s = (B(k+1,:)-B(k,:))/(n-1); 
d = B(k,:);
D = d;
for l=2:n
d = d+s;
D = [D; d];
end
if k==1
A = [A; D];
else
A = [A; D(2:size(D,1),:)];
end
end
A(find(A>1))=1;
% 以上过程全部是生成颜色映射矩阵A
% 以下过程是根据颜色映射矩阵A绘制magic（9）的颜色对应图
C = magic(9);
[m n]=size(C);
C_min = min(min(C));
C_max = max(max(C));
h = (C_max - C_min)/size(A,1);
figure;hold on
for i=1:m
for j=1:n
t = max(ceil((C(i,j) - C_min)/h),1);
a = A(t,:);
fill([j j+1 j+1 j],[i i i+1 i+1],a) 
end
end
C_rot = rot90(C,1)
for i=1:m
for j=1:n
text(i+0.5,j+0.5,sprintf('%0.0d',C(i,j)))
end
end
axis off equal
%}




