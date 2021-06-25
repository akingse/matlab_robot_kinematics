 %% initialize
clc; close all; clear all;
format shortg; format compact;

%% code area

% for j = 1:21 %文件数目，文件需按1-21数字命名


delimiterIn = ' '; 
headerlinesIn = 2;
filename = [num2str(j) '.asc'];
A = importdata(filename, delimiterIn, headerlinesIn); 
data=A.data;

x=data(:,1); %列数据
y=data(:,2);
z=data(:,3);
% x=x'; %行数据



%% 截面调平
% 断面轮廓预处理
N=length(z)
x=x(1:N-2);
z=z(1:N-2);
N=N-2; %去除文件 1.asc 最后两个错误点坐标


Sigma_iz=0;

for i=0:N-1
    Sigma_iz=Sigma_iz+i*z(i+1);    
end

Sigma_z=0;
for i=0:N-1
    Sigma_z=Sigma_z+z(i+1);    
end

b1=(12*Sigma_iz-6*(N-1)*Sigma_z)/(N*(N+1)*(N-1));
b0=Sigma_z/N-b1*(N-1)/2;

for i=0:N-1
    Z(i+1)=z(i+1)-b1*i-b0;    
end

%{
figure('Name','x,z'); %初始数据 x,z 的数据图
plot(x,z)

figure('Name','x,Z'); %初始数据 x,和Z 的数据图
plot(x,Z);

figure('Name','x,z 校正前')
p=polyfit(x,z,1);
plot(x,z,x,polyval(p,x));

figure('Name','x,Z 校正后')
x=x';
p=polyfit(x,Z,1);
plot(x,Z,x,polyval(p,x));


x=1:1:N;
x=x';
figure('Name','i,z 校正前')
p=polyfit(x,z,1);
plot(x,z,x,polyval(p,x));

figure('Name','i,Z 校正后')
x=x';
p=polyfit(x,Z,1);
plot(x,Z,x,polyval(p,x));
%}
%% MPD 平均坡度和平均斜率
delta_x=(x(N)-x(1))/N

% MPD 
Sigma_Z=0;
for i=1:N
    Sigma_Z=Sigma_Z+Z(i);
end
mid=ceil(N/2);
Z_ave=Sigma_Z/N;
MPD=(max(Z(1:ceil(N/2)))+max(Z(ceil(N/2):N)))/2-Z_ave;

% 平均坡度
Sigma_s=0;
for i=1:N-1
    Sigma_s=Sigma_s+(z(i+1)-z(i))/delta_x;
end
S1=Sigma_s/N;

% 平均斜率
Sigma_c=0;
for i=2:N-1
    Sigma_c=Sigma_c+(2*z(i)-z(i-1)-z(i+1))/(delta_x^2);
end
C=Sigma_c/(N-2);

%% 计算指标(原程序)
%Ra

Ra=sum(abs(z-mean(z)))/length(z);

%Rq
Rq=sqrt(sum((z-mean(z)).^2)/(length(z)-1));

%Rz
Rz=max(z)-min(z);

%Rsk

sigma=std(z);
Rsk=sum((z-mean(z)).^3)/length(z)/sigma^3   ; 

%Rku
Rku=sum((z-mean(z)).^4)/length(z)/sigma^4;

% %S1
% sum1=0;
% for i=1:(length(x)-1)
%     deltz=z(i+1)-z(i);
%     deltx=x(i+1)-x(i);
%     sum1=sum1+deltz/deltx;
% end
% S1=sum1/length(x);
% 
% %C
% sum2=0;
% for i=2:(length(x)-1)
%     deltz=2*z(i)-z(i-1)-z(i+1);
%     deltx=x(i+1)-x(i);
%     sum2=sum2+deltz/deltx^2;
% end
% C=sum2/(length(x)-2);
    
%La
sum3=0;
for i=1:(length(x)-1)
    deltz=z(i+1)-z(i);
    deltx=x(i+1)-x(i);
    sum3=sum3+abs(deltz/deltx);
end
Da=sum3/length(x);
La=2*pi*Ra/Da;

%Lq
sum4=0;
for i=1:(length(x)-1)
    deltz=z(i+1)-z(i);
    deltx=x(i+1)-x(i);
    sum4=sum4+(deltz/deltx)^2;
end
Dq=sqrt(sum4/length(x));
Lq=2*pi*Rq/Dq;

% MPD=0;
% for i=1:length(z)
%     MPD=MPD+max(z)-z(i);
% end
% MPD=MPD/length(x)

% output=[Ra,Rq,Rz,Rsk,Rku,Da,Dq,La,Lq,MPD]
output=[Da,Dq,La,Lq,Ra,Rku,Rq,Rsk,Rz,MPD,C,S1]
% xlswrite('Auto_1.xls',output,['A',num2str(j),':L',num2str(j)])
% end


