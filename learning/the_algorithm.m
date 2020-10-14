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
% ���Ը�˹�����������ķ���ʵ�ָ�˹���������ĸı��������˹������Ӱ�죬  
% ���Զ�����Ϊgif��ʽ�����  
% created by zhao.buaa 2016.09.28  
  
%����gif����  
item = 10;      % ��������  
dt = 1;             % ������С  
ksize =20;      % ��˹��С  
sigma = 2;      % �����С  
% filename = ['ksize-' num2str(ksize) '--' num2str(ksize+dt*item) '-sigma-' num2str(sigma) '.gif']; %����Ԥ�Ƚ���gif�ļ�  
filename = ['ksize-' num2str(ksize)  '-sigma-' num2str(sigma) '--' num2str(sigma+dt*item) '.gif']; %����Ԥ�Ƚ���gif�ļ�  
  
% main loop  
for i = 1:item  
    center  = round(ksize/2);          % ���ĵ�  
    bias       = ksize*10/10;              % ƫ�����ĵ���  
    ksigma = ksigma(ksize, sigma, center, bias);    % �����˺���  
    tname  = ['ksize-' num2str(ksize) '-sigma-' num2str(sigma) '-center-' num2str(center)];  
    figure(i), mesh(ksigma), title(tname);  
    %���ù̶���x-y-z���귶Χ�����ڹ۲죬axis([xmin xmax ymin ymax zmin zmax])  
    axis([0 ksize 0 ksize 0 0.008]);  view([0, 90]);% �ı���ӽǶ�     
    % ksize ����  
%     ksize = ksize + 10;  
    % sigma ����  
    sigma = sigma + dt;       
      
    % �Զ�����Ϊgifͼ��  
    frame = getframe(i);  
    im = frame2im(frame);  
    [I,map] = rgb2ind(im,256);  
    if i==1  
        imwrite(I,map,filename,'gif','Loopcount',inf, 'DelayTime',0.4);  
    else  
        imwrite(I,map,filename,'gif','WriteMode','append','DelayTime',0.4);  
    end  
end
%% �ضϸ�˹�˺������ضϵĳ̶�ȡ���ڲ���bias  
function ksigma = ksigma(ksize, sigma, center,bias)  
%ksize = 80;    sigma = 15;  
ksigma=fspecial('gaussian',ksize, sigma);   % ������˹����  
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
%% �����㷨

% �����������Լ��
function gcd(a,b:integer):integer;
begin
if b=0 then gcd:=a
else gcd:=gcd (b,a mod b);
end 

%% ����������С������

function lcm(a,b:integer):integer;

begin

if a< b then swap(a,b);

lcm:=a;

while lcm mod b >0 do inc(lcm,a);

end;

%% ��������

% A.С��Χ���ж�һ�����Ƿ�Ϊ������

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

% B.�ж�longint��Χ�ڵ����Ƿ�Ϊ������������50000���ڵ���������

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

2��

3��

%% 4.����С������

A.Prim�㷨��

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
% {Ѱ���������������δ���붥��k}

min:=maxlongint;

for j:=1 to n do

if (lowcost[j]< min) and (lowcost[j]< >0) then

begin

min:=lowcost[j];

k:=j;

end;

lowcost[k]:=0; {������k����������}

{������������һ���µı�k��closest[k]}

{���������lowcost��closestֵ}

for j:=1 to n do

if cost[k,j]< lwocost[j] then

begin

lowcost[j]:=cost[k,j];

closest[j]:=k;

end;

end;

end;{prim}

B.Kruskal�㷨��(̰��)

��Ȩֵ����

˳��ɾȥͼ�еıߣ������γɻ�·�򽫴˱߼�����С��������

function find(v:integer):integer; {���ض���v���ڵļ���}

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

for i:=1 to n do vset:=;{��ʼ������n�����ϣ���I�����ϰ���һ��Ԫ��I}

p:=n-1; q:=1; tot:=0; {pΪ�д�����ı�����qΪ�߼�ָ��}

sort;

{�����б߰�Ȩֵ�������򣬴���e[I]�У�e[I].v1��e[I].v2Ϊ��I�����ӵ������������ţ�e[I].lenΪ��I���ߵĳ���}

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

5.���·��

A.��ŷ���ⵥԴ�����·����

var

a:array[1..maxn,1..maxn] of integer;

b:array[1..maxn] of integer; {bָ����i��Դ������·��}

mark:array[1..maxn] of boolean;

procedure bhf;

var

best,best_j:integer;

begin

fillchar(mark,sizeof(mark),false);

mark[1]:=true; b[1]:=0;{1ΪԴ��}

repeat

best:=0;

for i:=1 to n do

If mark then {��ÿһ���Ѽ�������·���ĵ�}

for j:=1 to n do

if (not mark[j]) and (a[i,j] >0) then

if (best=0) or (b+a[i,j]< best) then

begin

best:=b+a[i,j]; best_j:=j;

end;

if best >0 then

begin

b[best_j]:=best��mark[best_j]:=true;

end;

until best=0;

end;{bhf}

B.Floyed�㷨������ж����֮������·����

procedure floyed;

begin

for I:=1 to n do

for j:=1 to n do

if a[I,j] >0 then p[I,j]:=I else p[I,j]:=0;

{p[I,j]��ʾI��j�����·����j��ǰ�����}

for k:=1 to n do {ö���м���}

for i:=1 to n do

for j:=1 to n do

if a[i,k]+a[j,k]< a[i,j] then

begin

a[i,j]:=a[i,k]+a[k,j];

p[I,j]:=p[k,j];

end;

end;

%% C.Dijkstra �㷨
% ���Ʊ�ŷ�������Ϊ̰���㷨��

var

a:array[1..maxn,1..maxn] of integer;

b,pre:array[1..maxn] of integer; {preָ���·����I��ǰ�����}

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

repeat {ÿѭ��һ�μ���һ����1��������Ľ�㲢�����������Ĳ���}

min:=maxint; u:=0; {u��¼��1��������Ľ��}

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
�Ͽ�˹������Dijkstra���㷨
a=zeros(6);
a(1,2)=50;a(1,4)=40;a(1,5)=25;a(1,6)=10;               
a(2,3)=15;a(2,4)=20;a(2,6)=25;
a(3,4)=10;a(3,5)=20;
a(4,5)=10;a(4,6)=25;
a(5,6)=55;
a=a+a'                                                  
a(find(a==0))=inf %��a=0����ȫ���滻Ϊ��ǿ��               
pb(1:length(a))=0;pb(1)=1;  %��һ�����Ѿ������ԭ�����̾���ʱ�����±�i��Ӧ��pb(i)��1
index1=1; %��Ŵ���S���ϵ�˳��
index2=ones(1,length(a)); %���ʼ�㵽��i�����ͨ·�е�i����ǰһ��������
d(1:length(a))=inf;d(1)=0;  %�����ʼ�㵽��i�����ͨ·��ֵ
temp=1;  %temp��ʾc1,��c1������������·��
while sum(pb)<length(a)  %���Ƿ����еĵ㶼���ΪP���
tb=find(pb==0); %�ҵ����Ϊ0�����е�,���ҵ���û�д���S�ĵ�
d(tb)=min(d(tb),d(temp)+a(temp,tb));%������Ϊ0�ĵ�����·�������Ǵ�ԭ��ֱ�ӵ�����㣬�ֻ�����ԭ�㾭��r1,��ӵ��������
tmpb=find(d(tb)==min(d(tb)));  %��d[tb]������Сֵ���±�
temp=tb(tmpb(1));%�����ж���·��ͬʱ������Сֵ��ȴ����һ����tempҲ��ԭ���Ϊ��һ����
pb(temp)=1;%�ҵ���С·���ı��Ӧ��pb(i)=1
index1=[index1,temp];  %��Ŵ���S���ϵ�˳��
temp2=find(d(index1)==d(temp)-a(temp,index1));
index2(temp)=index1(temp2(1)); %��¼�������
end
d, index1, index2
%}
