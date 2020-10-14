%% initialize
clc; close all; clear all;
format shortg; format compact;
%{

%}
%% 数据类型

% Matlab中有15种基本数据类型，主要是整型、浮点、逻辑、字符、日期和时间、结构数组、单元格数组以及函数句柄等。

% 1.整型
% 整型：通过intmax(class)和intmin(class) 函数返回该类整型的最大值和最小值，例如intmax(‘int8’)=127
% 
% 2.浮点
% 浮点数：REALMAX('double')和REALMAX('single')分别返回双精度浮点和单精度浮点的最大值，REALMIN('double')和REALMIN ('single')分别返回双精度浮点和单精度浮点的最小值。
% 
% 3.逻辑
% Logical：下例是逻辑索引在矩阵操作中的应用，将5*5矩阵中大于0.5的元素设定为0：

% 4.字符
%{
数据类型	描述
int8	8位有符号整数
uint8	8位无符号整数
int16	16位有符号整数
uint16	16位无符号整数
int32	32位有符号整数
uint32	32位无符号整数
int64	64位有符号整数
uint64	64位无符号整数
single	单精度数值数据
double	双精度数值数据
logical	逻辑值为1或0，分别代表true和false
char	字符数据(字符串作为字符向量存储)
单元格阵列	索引单元阵列，每个都能够存储不同维数和数据类型的数组
结构体	C型结构，每个结构具有能够存储不同维数和数据类型的数组的命名字段
函数处理	指向一个函数的指针
用户类	用户定义的类构造的对象
Java类	从Java类构造的对象

%}
str = 'Hello World!'
n = 2345
d = double(n)
un = uint32(789.50)
rn = 5678.92347
c = int32(rn)
%% 转换
%{
函数	描述说明
char	转换为字符数组(字符串)
int2str	将整数数据转换为字符串
mat2str	将矩阵转换为字符串
num2str	将数字转换为字符串
str2double	将字符串转换为双精度值
str2num	将字符串转换为数字
native2unicode	将数字字节转换为Unicode字符
unicode2native	将Unicode字符转换为数字字节
base2dec	将基数N字符串转换为十进制数
bin2dec	将二进制数字串转换为十进制数
dec2base	将十进制转换为字符串中的N数字
dec2bin	将十进制转换为字符串中的二进制数
dec2hex	将十进制转换为十六进制数字
hex2dec	将十六进制数字字符串转换为十进制数
hex2num	将十六进制数字字符串转换为双精度数字
num2hex	将单数转换为IEEE十六进制字符串
cell2mat	将单元格数组转换为数组
cell2struct	将单元格数组转换为结构数组
cellstr	从字符数组创建字符串数组
mat2cell	将数组转换为具有潜在不同大小的单元格的单元阵列
num2cell	将数组转换为具有一致大小的单元格的单元阵列
struct2cell	将结构转换为单元格数组
%}



%% 字符
% Matlab中的输入字符需使用单引号。字符串存储为字符数组，每个元素占用一个ASCII字符。
% 如日期字符：DateString=’9/16/2001’ 实际上是一个1行9列向量。构成矩阵或向量的行字符串长度必须相同。可以使用char函数构建字符数组，使用strcat函数连接字符。
% 
% 例如，命令 name = ['abc' ; 'abcd'] 将触发错误警告，因为两个字符串的长度不等，
% 此时可以通过空字符凑齐如：name = ['abc ' ; 'abcd']，更简单的办法是使用char函数：
% char(‘abc’,’abcd’)，Matlab自动填充空字符以使长度相等，因此字符串矩阵的列纬总是等于最长字符串的字符数，
% 例如size(char(‘abc’,’abcd’))返回结果[2,4]，即字符串’abc’实际存在的是’abc ’，
% 此时如需提取矩阵中的某一字符元素，需要使用deblank函数移除空格如name =char(‘abc’,’abcd’); deblank(name(1,:))。
% 此外，Matlab同时提供一种更灵活的单元格数组方法，使用函数cellstr可以将字符串数组转换为单元格数组：
% % 
% data= char(‘abc’,’abcd’)
% length(data(1,:))  ->  4
% cdata=cellstr(data)
% length(cdata{1})   -> 3
% 
% 常用的字符操作函数
% blanks(n)   返回n个空字符
% deblank(s)   移除字符串尾部包含的空字符
% eval(string)   将字符串作为命令执行
% findstr(s1,s2)  搜索字符串
% ischar(s)   判断是否字符串
% isletter(s)   判断是否字母
% lower(s)   转换小写
% upper(s)   转换大写
% strcmp(s1,s2) 比较字符串是否相同
% strncmp(s1,s2,n) 比较字符串中的前n个字符是否相同
% strrep(s1,s2,s3)  将s1中的字符s2替换为s3
%% Example program
x= 3;
% isinteger(x)
% isfloat(x)
% isvector(x)
% isscalar(x)
isnumeric(x)

% s=sym(A)将非符号对象（如，数字，表达式，变量等）A转换为符号对象，并存储在符号变量s中;
% inf为无穷大量，-inf为无穷小量,NaN表示非数值的值，产生一般是由于0做了分母或者运算溢出;


%% 5.日期和时间
% Matlab提供三种日期格式：日期字符串如’1996-10-02’，日期序列数如729300（0000年1月1日为1）以及日期向量如 1996 10 2 0 0 0，依次为年月日时分秒。
% 
% 常用的日期操作函数
% datestr(d,f)   将日期数字转换为字符串
% datenum(str,f)  将字符串转换为日期数字
% datevec(str)  日期字符串转换向量
% weekday(d)   计算星期数
% eomday(yr,mth)  计算指定月份最后一天
% calendar(str)  返回日历矩阵
% clock   当前日期和时间的日期向量
% date    当前日期字符串
% now    当前日期和时间的序列数
% 
now
date

%% 6.结构

% 结构是包含已命名“数据容器”或字段的数组。结构中的字段可以包含任何数据，例如：
%% 7.构建结构数组：赋值方法
% 下面的赋值命令产生一个名为patient的结构数组，该数组包含三个字段：
% 直接定义
patient.name = 'John Doe';
patient.billing = 127.00;
patient.test = [79 75 73; 180 178 177.5; 220 210 205];
patient

% 在命令区内输入patient可以查看结构信息：
% name: 'John Doe'
% billing: 127
% test: [3x3 double]
% 
% 继续赋值可扩展该结构数组：
% patient(2).name = 'Ann Lane';
% patient(2).billing = 28.50;
% patient(2).test = [68 70 68; 118 118 119; 172 170 169];
% 赋值后结构数组变为[1 2]。
% 
% 构建结构数组：struct函数
% 函数基本形式为：strArray = struct('field1',val1,'field2',val2, ...)
% 
% 例如：
% weather(1) = struct('temp', 72,'rainfall', 0.0); weather(2) = struct('temp', 71,'rainfall', 0.1);
% weather = repmat(struct('temp', 72, 'rainfall', 0.0), 1, 3);
% weather = struct('temp', {68, 80, 72}, 'rainfall', {0.2, 0.4, 0.0});

% 访问结构数据
% 以下都是合法的结构数组访问命令：
% mypatients = patient(1:2)  获取子结构数据
% mypatients(1)    访问结构数据
% patient(2).name    访问结构数据中的特定字段
% patient(3).test(2,2)   访问结构数据中的特定字段（该字段为数组）
% bills = [patient.billing]   访问多个结构
% tests = {patient(1:2).test}  提取结构数据转换成单元格数组
% 
% 使用结构字段的动态名称
% 通过structName.(expression)可以赋予结构字段名称并访问数据。例如字段名为expression、
% 结构名为structName，访问其中第7行1至25列数据可以使用命令：structName.(expression)(7,1:25)。
% 例如，存在一个学生每周成绩数据结构数组，其数据通过以下方式建立：
% testscores.wang.week(1:25) = ...
% 
% [95 89 76 82 79 92 94 92 89 81 75 93 ...
% 85 84 83 86 85 90 82 82 84 79 96 88 98];
% 
% testscores.chen.week(1:25) = ...
% [87 80 91 84 99 87 93 87 97 87 82 89 ...
% 86 82 90 98 75 79 92 84 90 93 84 78 81];
% 
% 即结构名为testscores，字段使用每个学生的名称命名，分别为wang和chen，每个学生下面包含名为week的成绩结构数组。
% 现计算给定结构名称、学生名称和起止周数的平均分数。
% 在命令窗口中输入 edit avgscore.m，输入以下代码后保存文件：
% function avg = avgscore(struct,student, first, last)
% avg = sum(struct.(student).week(first:last))/(last - first + 1);
% 在命名窗口中输入：avgscore(testscores, 'chen', 7, 22) 计算学生陈从第7周到第22周的平均分数。
% 添加和删除结构字段
% 
% 命令[struct](index).(field)可添加或修改字段。如patient(2).ssn = '000-00-0000' 在结构patient中添加一个名为ssn的字段。
% 删除字段使用rmfield函数，如patient2 = rmfield(patient, 'name') 删除name字段并产生新的结构。
% 
%% 单元格数组
% 单元格数组提供了不同类型数据的存储机制，可以储存任意类型和任意纬度的数组。
% 访问单元格数组的规则和其他数组相同，区别在于需要使用花括号{}访问，例如A{2,5}访问单元格数组A中的第2行第5列单元格。
% 构建单元格数组：赋值方法
% 使用花括号标识可直接创建单元格数组，如：
% 
% A(1,1) = {[1 4 3; 0 5 8; 7 2 9]};
% A(1,2) = {'abcd'};
% A(2,1) = {3+7i};
% A(2,2) = {-pi:pi/10:pi};
% 上述命令创建2*2的单元格数组A。继续添加单元格元素直接使用赋值如A(2,3)={5}即可，
% 注意需使用花括号标识。简化的方法是结合使用花括号（单元格数组）和方括号（）创建，如C = {[1 2], [3 4]; [5 6], [7 8]};
% 
% 构建单元格数组：函数方法
% Cell函数。如：
% B = cell(2, 3);
% B(1,3) = {1:3};
% 访问数据
% 通过索引可直接访问单元格数组中的数据元素，例如：
% N{1,1} = [1 2; 4 5];
% N{1,2} = 'Name';
% N{2,1} = 2-4i;
% N{2,2} = 7;
% c = N{1,2}
% d = N{1,1}(2,2)
% 
% 函数句柄
% 函数句柄是用于间接调用一个函数的Matlab值或数据类型。在调用其它函数时可以传递函数句柄，也可在数据结构中保存函数句柄备用。
% 通过命令形式 fhandle = @functionname 可以创建函数句柄，例如 trigFun=@sin，或匿名函数sqr = @(x) x.^2;。
% 使用句柄调用函数的形式是 fhandle(arg1, arg2, ..., argn) 或 fhandle()（无参数）

