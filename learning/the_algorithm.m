%% initialize
clc; close all; clear all;
format shortg; format compact;
% https://wenku.baidu.com/view/a31ce5c489eb172ded63b73c.html
%% 
syms x y 
float gaussfunc(float miu,float sigma,float x):
return exp(-1/2*(x-miu)*(x-miu)/(sigma*sigma))/sqrt(sigma*sigma*2*pi)

% x=linspace(-6,6,500);
% plot(x,gaussfunc(0,1,x))
% hold on
% plot(x,gaussfunc(0,2^2,x),'--')
% X1 = [0:0.1:10];
% X2 = [0:0.1:10];
% [X,Y] = meshgrid(X1,X2);
% Z = exp(-((X-5).*(X-5)+(Y-5).*(Y-5))/18);
% surf(X,Y,Z);


%{
  note
%}

%% bubbling 
A=[1 3 5 6 7 10 20 15 2 8]
B=bubsort(A)
function y=bubsort(x)
    r=length(x);
    for i=1:r-1
        for j=i+1:r
            if x(i)>x(j)
            t=x(j);
            x(j)=x(i);
            x(i)=t;
            end
        end
    end
    y=x;
end
%{
%% 
function mainfunc()  
% 测试高斯函数，递增的方法实现高斯函数参数的改变对整个高斯函数的影响，  
% 并自动保存为gif格式输出。  
% created by zhao.buaa 2016.09.28  
  
%保存gif动画  
item = 10;      % 迭代次数  
dt = 1;             % 步长大小  
ksize =20;      % 高斯大小  
sigma = 2;      % 方差大小  
% filename = ['ksize-' num2str(ksize) '--' num2str(ksize+dt*item) '-sigma-' num2str(sigma) '.gif']; %必须预先建立gif文件  
filename = ['ksize-' num2str(ksize)  '-sigma-' num2str(sigma) '--' num2str(sigma+dt*item) '.gif']; %必须预先建立gif文件  
  
% main loop  
for i = 1:item  
    center  = round(ksize/2);          % 中心点  
    bias       = ksize*10/10;              % 偏移中心点量  
    ksigma = ksigma(ksize, sigma, center, bias);    % 构建核函数  
    tname  = ['ksize-' num2str(ksize) '-sigma-' num2str(sigma) '-center-' num2str(center)];  
    figure(i), mesh(ksigma), title(tname);  
    %设置固定的x-y-z坐标范围，便于观察，axis([xmin xmax ymin ymax zmin zmax])  
    axis([0 ksize 0 ksize 0 0.008]);  view([0, 90]);% 改变可视角度     
    % ksize 递增  
%     ksize = ksize + 10;  
    % sigma 递增  
    sigma = sigma + dt;       
      
    % 自动保存为gif图像  
    frame = getframe(i);  
    im = frame2im(frame);  
    [I,map] = rgb2ind(im,256);  
    if i==1  
        imwrite(I,map,filename,'gif','Loopcount',inf, 'DelayTime',0.4);  
    else  
        imwrite(I,map,filename,'gif','WriteMode','append','DelayTime',0.4);  
    end  
end
%% 截断高斯核函数，截断的程度取决于参数bias  
function ksigma = ksigma(ksize, sigma, center,bias)  
%ksize = 80;    sigma = 15;  
ksigma=fspecial('gaussian',ksize, sigma);   % 构建高斯函数  
[m, n] =size(ksigma);  
for i = 1:m  
    for j = 1:n  
        if(  (i<center-bias)||(i>center+bias)||(j<center-bias)||(j>center+bias)  )  
            ksigma(i,j) = 0;  
        end;  
    end;  
end; 

%}

%{
%% 数论算法

% 求两数的最大公约数
function gcd(a,b:integer):integer;
begin
if b=0 then gcd:=a
else gcd:=gcd (b,a mod b);
end 

%% 求两数的最小公倍数

function lcm(a,b:integer):integer;

begin

if a< b then swap(a,b);

lcm:=a;

while lcm mod b >0 do inc(lcm,a);

end;

%% 素数的求法

% A.小范围内判断一个数是否为质数：

function prime (n: integer): Boolean;
var I: integer;
begin
for I:=2 to trunc(sqrt(n)) do
if n mod I=0 then
begin
prime:=false; exit;
end

prime:=true;
end

% B.判断longint范围内的数是否为素数（包含求50000以内的素数表）：

procedure getprime;

var
i,j:longint;
p:array[1..50000] of boolean;
begin
fillchar(p,sizeof(p),true);
p[1]:=false;
i:=2;
while i< 50000 do
begin
if p then
begin
j:=i*2;
while j< 50000 do

begin

p[j]:=false;
inc(j,i);
end
end

inc(i);

end;

l:=0;

for i:=1 to 50000 do

if p then

begin

inc(l);

pr[l]:=i;

end;

end;{getprime}

function prime(x:longint):integer;

var i:integer;

begin

prime:=false;

for i:=1 to l do

if pr >=x then break

else if x mod pr=0 then exit;

prime:=true;

end;{prime}

2．

3．

%% 4.求最小生成树

A.Prim算法：

procedure prim(v0:integer);
var
lowcost,closest:array[1..maxn] of integer;
i,j,k,min:integer;
begin
for i:=1 to n do
begin
lowcost:=cost[v0,i];
closest:=v0;
end
for i:=1 to n-1 do

begin
% {寻找离生成树最近的未加入顶点k}

min:=maxlongint;

for j:=1 to n do

if (lowcost[j]< min) and (lowcost[j]< >0) then

begin

min:=lowcost[j];

k:=j;

end;

lowcost[k]:=0; {将顶点k加入生成树}

{生成树中增加一条新的边k到closest[k]}

{修正各点的lowcost和closest值}

for j:=1 to n do

if cost[k,j]< lwocost[j] then

begin

lowcost[j]:=cost[k,j];

closest[j]:=k;

end;

end;

end;{prim}

B.Kruskal算法：(贪心)

按权值递增

顺序删去图中的边，若不形成回路则将此边加入最小生成树。

function find(v:integer):integer; {返回顶点v所在的集合}

var i:integer;

begin

i:=1;

while (i< =n) and (not v in vset) do inc(i);

if i< =n then find:=i

else find:=0;

end;

procedure kruskal;

var

tot,i,j:integer;

begin

for i:=1 to n do vset:=;{初始化定义n个集合，第I个集合包含一个元素I}

p:=n-1; q:=1; tot:=0; {p为尚待加入的边数，q为边集指针}

sort;

{对所有边按权值递增排序，存于e[I]中，e[I].v1与e[I].v2为边I所连接的两个顶点的序号，e[I].len为第I条边的长度}

while p >0 do

begin

i:=find(e[q].v1);j:=find(e[q].v2);

if i< >j then

begin

inc(tot,e[q].len);

vset:=vset+vset[j];vset[j]:=[];

dec(p);

end;

inc(q);

end;

writeln(tot);

end;

5.最短路径

A.标号法求解单源点最短路径：

var

a:array[1..maxn,1..maxn] of integer;

b:array[1..maxn] of integer; {b指顶点i到源点的最短路径}

mark:array[1..maxn] of boolean;

procedure bhf;

var

best,best_j:integer;

begin

fillchar(mark,sizeof(mark),false);

mark[1]:=true; b[1]:=0;{1为源点}

repeat

best:=0;

for i:=1 to n do

If mark then {对每一个已计算出最短路径的点}

for j:=1 to n do

if (not mark[j]) and (a[i,j] >0) then

if (best=0) or (b+a[i,j]< best) then

begin

best:=b+a[i,j]; best_j:=j;

end;

if best >0 then

begin

b[best_j]:=best；mark[best_j]:=true;

end;

until best=0;

end;{bhf}

B.Floyed算法求解所有顶点对之间的最短路径：

procedure floyed;

begin

for I:=1 to n do

for j:=1 to n do

if a[I,j] >0 then p[I,j]:=I else p[I,j]:=0;

{p[I,j]表示I到j的最短路径上j的前驱结点}

for k:=1 to n do {枚举中间结点}

for i:=1 to n do

for j:=1 to n do

if a[i,k]+a[j,k]< a[i,j] then

begin

a[i,j]:=a[i,k]+a[k,j];

p[I,j]:=p[k,j];

end;

end;

%% C.Dijkstra 算法
% 类似标号法，本质为贪心算法。

var

a:array[1..maxn,1..maxn] of integer;

b,pre:array[1..maxn] of integer; {pre指最短路径上I的前驱结点}

mark:array[1..maxn] of boolean;

procedure dijkstra(v0:integer);

begin

fillch

ar(mark,sizeof(mark),false);

for i:=1 to n do

begin

d:=a[v0,i];

if d< >0 then pre:=v0 else pre:=0;

end;

mark[v0]:=true;

repeat {每循环一次加入一个离1集合最近的结点并调整其他结点的参数}

min:=maxint; u:=0; {u记录离1集合最近的结点}

for i:=1 to n do
if (not mark) and (d< min) then
begin
u:=i; min:=d;
end;

if u<>0 then
begin
mark:=true;
for i:=1 to n do
if (not mark) and (a[u,i]+d< d) then
begin
d:=a[u,i]+d;
pre:=u;
end
end
until u=0;
end
%}

%% 
%{
迪克斯特拉（Dijkstra）算法
a=zeros(6);
a(1,2)=50;a(1,4)=40;a(1,5)=25;a(1,6)=10;               
a(2,3)=15;a(2,4)=20;a(2,6)=25;
a(3,4)=10;a(3,5)=20;
a(4,5)=10;a(4,6)=25;
a(5,6)=55;
a=a+a'                                                  
a(find(a==0))=inf %将a=0的数全部替换为无强大               
pb(1:length(a))=0;pb(1)=1;  %当一个点已经求出到原点的最短距离时，其下标i对应的pb(i)赋1
index1=1; %存放存入S集合的顺序
index2=ones(1,length(a)); %存放始点到第i点最短通路中第i顶点前一顶点的序号
d(1:length(a))=inf;d(1)=0;  %存放由始点到第i点最短通路的值
temp=1;  %temp表示c1,算c1到其它点的最短路。
while sum(pb)<length(a)  %看是否所有的点都标记为P标号
tb=find(pb==0); %找到标号为0的所有点,即找到还没有存入S的点
d(tb)=min(d(tb),d(temp)+a(temp,tb));%计算标号为0的点的最短路，或者是从原点直接到这个点，又或者是原点经过r1,间接到达这个点
tmpb=find(d(tb)==min(d(tb)));  %求d[tb]序列最小值的下标
temp=tb(tmpb(1));%可能有多条路径同时到达最小值，却其中一个，temp也从原点变为下一个点
pb(temp)=1;%找到最小路径的表对应的pb(i)=1
index1=[index1,temp];  %存放存入S集合的顺序
temp2=find(d(index1)==d(temp)-a(temp,index1));
index2(temp)=index1(temp2(1)); %记录标号索引
end
d, index1, index2
%}
