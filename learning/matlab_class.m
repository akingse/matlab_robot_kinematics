%% initialize
clc; close all; clear all;
format shortg; format compact;

%% 类与对象
% classdef Unitied8 %< handle  %注释避免冲突；
% % <表示继承，Matlab允许多重继承，继承自handle类，handle类定义了很多关于object的处理函数。
%     methods (Static = true)  
%         function a = test(b, c)  
%             a = b + c;  
%         end  
%     end  
% end 

% classdef Unitied8
%     properties               %定义属性－－－类变量
%         x;
%         y;
%     end
%     properties (Constant)    % 定义类常量
%         z =0;
%     end
%     methods                  % 定义类的方法
%         function obj = Unitied8 (a,b)   %构造函数，函数类名一致，完成类中变量的初始化
%             obj.x = a;
%             obj.y = b;
%         end
%         function display(obj)   % 自定义函数
%             fprintf ('calcul+:\t ');
%             fprintf('x+y = %d',obj.x + obj.y);
%         end
%     end
% end
