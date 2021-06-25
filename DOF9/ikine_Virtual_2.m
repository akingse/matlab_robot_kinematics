
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Readme
整理Virtual6函数，一个纯函数脚本，已使用最简计算方法

% cos(th8)*(nx*cos(th9) - ox*sin(th9)) - ax*sin(th8)==cos(th4)*cos(th5)*cos(th6) - sin(th4)*sin(th6)
% cos(th8)*(ny*cos(th9) - oy*sin(th9)) - ay*sin(th8)==sin(th4)*cos(th5)*cos(th6) + cos(th4)*sin(th6)
% cos(th8)*(nz*cos(th9) - oz*sin(th9)) - az*sin(th8)==-cos(th6)*sin(th5)
% 
% sin(th8)*(nx*cos(th9) - ox*sin(th9)) + ax*cos(th8)==cos(th4)*sin(th5)
% sin(th8)*(ny*cos(th9) - oy*sin(th9)) + ay*cos(th8)==sin(th4)*sin(th5)
% sin(th8)*(nz*cos(th9) - oz*sin(th9)) + az*cos(th8)==cos(th5)
% 
% - ox*cos(th9) - nx*sin(th9)==cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6)
% - oy*cos(th9) - ny*sin(th9)==cos(th5)*sin(th4)*sin(th6) - cos(th4)*cos(th6)
% - oz*cos(th9) - nz*sin(th9)==-sin(th5)*sin(th6)
% 
% px - a9*nx - a8*(nx*cos(th9) - ox*sin(th9))==d7*cos(th4)*sin(th5)
% py - a9*ny - a8*(ny*cos(th9) - oy*sin(th9))==d7*sin(th4)*sin(th5)
% pz - a9*nz - a8*(nz*cos(th9) - oz*sin(th9))==d7*cos(th5)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

%% ikine_Virtual_2
function th=ikine_Virtual_2(T)
    a8=50;a9=50;
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
        fprintf("v2 correct\n");
    else
        fprintf("v2 error\n");
    end
   

% th9
    Px=px-a9*nx;
    Py=py-a9*ny;
    Pz=pz-a9*nz;    
    Pm=Px*(az*ny-ay*nz)+Py*(ax*nz-az*nx)+Pz*(ay*nx-ax*ny);  N_zero(Pm);
    Pn=Px*(az*oy-ay*oz)+Py*(ax*oz-az*ox)+Pz*(ay*ox-ax*oy);  N_zero(Pn);

    if Pm==0 && Pn==0  %奇异点1（a8+d7*s8=0）
        th9(1)=0;
    else
        th9(1)=atan(Pm/Pn);
    end
    th9(2)=th9(1)+pi;
% th8
    Rx=nx*cos(th9)-ox*sin(th9);
    Ry=ny*cos(th9)-oy*sin(th9);
    Rz=nz*cos(th9)-oz*sin(th9);

    m81=ax*(Pz-a8*Rz)-az*(Px-a8*Rx); m81=N_zero(m81);
    n81=Rz.*(Px-a8*Rx)-Rx.*(Pz-a8*Rz); n81=N_zero(n81);
    m82=ay*(Pz-a8*Rz)-az*(Py-a8*Ry); m82=N_zero(m82);
    n82=Rz.*(Py-a8*Ry)-Ry.*(Pz-a8*Rz); n82=N_zero(n82);
    m83=ax*(Py-Ry*a8)-ay*(Px-Rx*a8); m83=N_zero(m83);
    n83=Ry.*(Px-Rx*a8)-Rx.*(Py-Ry*a8); n83=N_zero(n83);
    if m81(1)==0&&m81(2)==0 %
        if m82(1)==0&&m82(2)==0
            th8(1:2)=atan(m83./n83);
        else
            th8(1:2)=atan(m82./n82);
        end
    else
        th8(1:2)=atan(m81./n81);
    end
    th8(3:4)=th8(1:2)+pi; %theta8有4个值；
    th9(3:4)=th9(1:2);
%d7
    m71=Px-a8*(nx*cos(th9)-ox*sin(th9)); m71=N_zero(m71);
    n71=ax*cos(th8)+sin(th8).*(nx*cos(th9)-ox*sin(th9)); n71=N_zero(n71);
    m72=Py-a8*(ny*cos(th9)-oy*sin(th9)); m72=N_zero(m72);
    n72=ay*cos(th8)+sin(th8).*(ny*cos(th9)-oy*sin(th9)); n72=N_zero(n72);
    m73=Pz-a8*(nz*cos(th9)-oz*sin(th9)); m73=N_zero(m73);
    n73=az*cos(th8)+sin(th8).*(nz*cos(th9)-oz*sin(th9)); n73=N_zero(n73);
    if m71(1)==0
        if m72(1)==0
            d7(1:4)=m73./n73;
        else
            d7(1:4)=m72./n72;
        end
    else
        d7(1:4)=m71./n71;
    end
    
    i=1;
    for j=1:4
        if d7(j)>0 %取d7大于0的解
            th(i,9)=In_pi(th9(j));
            th(i,8)=In_pi(th8(j));
            th(i,7)=d7(j);
            i=i+1;
        end
    end
end

%%  standard
function mn = N_zero(mn)
    S=size(mn);
    for i=1:S(1)
        for j=1:S(2)
            if abs(mn(i,j))<1E-4
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