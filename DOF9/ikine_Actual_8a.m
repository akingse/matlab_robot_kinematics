%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
整理Actual6函数（alpha6=pi/2版），一个纯函数脚本，已使用最简计算方法

% cos(th2+th3+th4)*cos(th5)=sin(th6)*(ax*cos(th1)+ay*sin(th1))+cos(th6)*(nx*cos(th1)+ny*sin(th1))
% sin(th2+th3+th4)*cos(th5)=nz*cos(th6)+az*sin(th6)
                   sin(th5)=-sin(th6)*(ay*cos(th1)-ax*sin(th1))-cos(th6)*(ny*cos(th1)-nx*sin(th1))  ①

-sin(th2+th3+th4)=sin(th6)*(nx*cos(th1)+ny*sin(th1))-cos(th6)*(ax*cos(th1)+ay*sin(th1))  ②
 cos(th2+th3+th4)=nz*sin(th6)-az*cos(th6)                                                ③
                0=cos(th6)*(ay*cos(th1)-ax*sin(th1))-sin(th6)*(ny*cos(th1)-nx*sin(th1))  ④

% -cos(th2+th3+th4)*sin(th5)=ox*cos(th1)+oy*sin(th1)  
% -sin(th2+th3+th4)*sin(th5)=oz                       
                  cos(th5)=ox*sin(th1)-oy*cos(th1)  ⑤

a2*cos(th2)+a3*cos(th2+th3)+d5*sin(th2+th3+th4)=px*cos(th1)-d6*(ox*cos(th1)+oy*sin(th1))+py*sin(th1)  ⑥
a2*sin(th2)+a3*sin(th2+th3)-d5*cos(th2+th3+th4)=pz-d1-d6*oz                                           ⑦
                                             d4=d6*(oy*cos(th1)-ox*sin(th1))-py*cos(th1)+px*sin(th1)  ⑧

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

%% ikine_Actual_8a
function th=ikine_Actual_8a(T)
    d1=90;d4=90;d5=90;d6=800;a2=-420;a3=-400;
    nx=T(1,1);ox=T(1,2);ax=T(1,3);px=T(1,4);
    ny=T(2,1);oy=T(2,2);ay=T(2,3);py=T(2,4);
    nz=T(3,1);oz=T(3,2);az=T(3,3);pz=T(3,4);
    %数据预标准化处理
    vn=nx^2+ny^2+nz^2;
    vo=ox^2+oy^2+oz^2;
    va=ax^2+ay^2+az^2;
    no=nx*ox+ny*oy+nz*oz;
    na=nx*ax+ny*ay+nz*az;
    oa=ox*ax+oy*ay+oz*az;
    if N_zero(vn-1)==0 && N_zero(vo-1)==0 && N_zero(va-1)==0 && N_zero(no)==0 && N_zero(na)==0 && N_zero(oa)==0
        fprintf("a8a correct\n");
    else
        fprintf("a8a error\n");
    end
    
    % th1
    m1=d6*ox-px;
    n1=d6*oy-py;
    k1=d4/sqrt(m1^2+n1^2);
    if (k1<=1) %工作空间判定
            th1(1)=acos(k1)-atan2(m1,n1);
            th1(2)=-acos(k1)-atan2(m1,n1);
    else
        th1(1:2)=NaN;
    end
    
    % th6
    m6=ay*cos(th1)-ax*sin(th1);  m6=N_zero(m6);
    n6=ny*cos(th1)-nx*sin(th1);  n6=N_zero(n6);
    th6(1:2)=atan(m6./n6);
    if m6(1)==0 && n6(1)==0 %奇异判定1
        th6(1)=0;
    end
    if m6(2)==0 && n6(2)==0
        th6(2)=0;
    end
    th6(3:4)=th6(1:2)+pi;
    
    % th5
    th1(3:4)=th1(1:2); 
    m5=-sin(th6).*(ay*cos(th1)-ax*sin(th1))-cos(th6).*(ny*cos(th1)-nx*sin(th1));
    n5=ox*sin(th1)-oy*cos(th1);
    th5=atan2(m5,n5); %无奇异；理论上无需参数取值范围判定
    
    % th234
    m=px*cos(th1)+py*sin(th1)-d6*(ox*cos(th1)+oy*sin(th1))+d5*(sin(th6).*(nx*cos(th1)+ny*sin(th1))-cos(th6).*(ax*cos(th1)+ay*sin(th1)));
    n=pz-d1-d6*oz+d5*(nz*sin(th6)-az*cos(th6));
    mn=m.^2+n.^2; %4个不同且无关的值
    
    for i=1:4
        if (a2-a3)^2<=mn(i) && mn(i)<=(a2+a3)^2
            k2=(m(i)^2+n(i)^2+a2^2-a3^2)/sqrt((2*a2*m(i))^2+(2*a2*n(i))^2);
            th2(i)=atan2(n(i)*a2,m(i)*a2)-acos(k2);
            th2(i+4)=atan2(n(i)*a2,m(i)*a2)+acos(k2);
            k3=(m(i)^2+n(i)^2-a2^2-a3^2)/(2*a2*a3);
            th3(i)=acos(k3); %注意使用+/-acos函数时th2 th3唯一对应关系；
            th3(i+4)=-th3(i);
        else
            th2(i)=NaN;
            th2(i+4)=NaN;
            th3(i)=NaN;
            th3(i+4)=NaN;
        end
    end
    
% th4 无奇异
    m234=-sin(th6).*(nx*cos(th1)+ny*sin(th1))+cos(th6).*(ax*cos(th1)+ay*sin(th1));
    n234=nz*sin(th6)-az*cos(th6);
    th234=atan2(m234,n234);
    th234(5:8)=th234(1:4);
    th4=th234-th3-th2;
    % sort out;
    th1(5:8)=th1(1:4);
    th5(5:8)=th5(1:4);
    th6(5:8)=th6(1:4);

    for i=1:8
        th(i,1)=In_pi(th1(i));
        th(i,2)=In_pi(th2(i));
        th(i,3)=th3(i);
        th(i,4)=In_pi(th4(i));
        th(i,5)=th5(i);
        th(i,6)=In_pi(th6(i));
    end
end

%%  standard
function mn = N_zero(mn)
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<1E-5 
                mn(i,j)=0;
            end
        end
    end
end   

function theta = In_pi(theta)
    S=size(theta);
    for i=1:S(1)
        for j=1:S(2)
                while (abs(theta(i,j))>pi)
                    if (theta(i,j)>pi)
                        theta(i,j)=theta(i,j)-2*pi;
                    elseif (theta(i,j)<-pi)
                        theta(i,j)=theta(i,j)+2*pi;
                    end
                end
        end
    end
end