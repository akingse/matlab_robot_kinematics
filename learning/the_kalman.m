clc;clear all;
format short g;
%% MATLAB �������˲�
N=200;
w(1)=0;  w=randn(1,N);
x(1)=0;  a=1;
for k=2:N
    x(k)=a*x(k-1)+w(k-1);
end
V=randn(1,N);
q1=std(V); %�����׼ƫ��;
Rvv=q1.^2;  q2=std(x);
Rxx=q2.^2;  q3=std(w);
Rww=q3.^2;
c=0.2;
Y=c*x+V;
p(1)=0;
s(1)=0;
for t=2:N;
    p1(t)=a.^2*p(t-1)+Rww;
    b(t)=c*p1(t)/(c.^2*p1(t)+Rvv);
    s(t)=a*s(t-1)+b(t)*(Y(t)-a*c*s(t-1));
    p(t)=p1(t)-c*b(t)*p1(t);
end
t=1:N;
% plot(t,s,'r',t,Y,'g',t,x,'b');
%% �ִ�����
% ss��������ʵ��������״̬�ռ�ģ�ͣ����߽����ݺ������㼫������ģ��תΪ״̬�ռ�ģ��
% ����״̬�ռ�ģ��
% sys = ss(a,b,c,d) ����������״̬�ռ�ģ��
% x��=Ax+Bu
% y=Cx+Du
% ���ģ�;���Nx��״̬��Ny�������Nu������
% a��Nx*Nx�ľ���
% b��Nx-by-Nu����
% 
% c ��Ny-by-Nx����
% d ��Ny-by-Nu ����
% ��d��Ϊ����0����������D����Ϊ0��������Ϣ������Ϊ״̬�ռ�ģ��matlab��state-space models����
% sys = ss(a,b,c,d,Ts) ������ɢʱ��ģ��
% x��n+1��=Ax��n��+Bu��n��
% y��n��=Cx��n��+Du��n��
% ����ʱ�䵥λTsΪS������Ts = -1 or Ts = []����ָ������ʱ��
A=[0 1/2 -1/2;
    -1/4 -1/2 0;
    1/4 0 -1/4];
B=[0 1/4 0]';
C=[1 0 0]; 
D=0;
ss(A,B,C,D);

% M=ctrb(A,B); %�ܿ���
% r=rank(M)
% N=obsv(A,C); %�ܹ���
% r=rank(N)
% 
% P=lyap(A',eye(3)); %ϵͳ�ȶ���?
% r=det(P(2,2))
% r=det(P)

%% kalman_filter ����
function [x, V, VV, loglik] = kalman_filter(y, A, C, Q, R, init_x, init_V, varargin)
%{ Kalman filter.
% [x, V, VV, loglik] = kalman_filter(y, A, C, Q, R, init_x, init_V, ...)
%
% INPUTS:
% y(:,t) - the observation at time t
% A - the system matrix
% C - the observation matrix
% Q - the system covariance
% R - the observation covariance
% init_x - the initial state (column) vector
% init_V - the initial state covariance
%
% OPTIONAL INPUTS (string/value pairs [default in brackets])
% 'model' - model(t)=m means use params from model m at time t [ones(1,T) ]
% In this case, all the above matrices take an additional final dimension,
% i.e., A(:,:,m), C(:,:,m), Q(:,:,m), R(:,:,m).
% However, init_x and init_V are independent of model(1).
% 'u' - u(:,t) the control signal at time t [ [] ]
% 'B' - B(:,:,m) the input regression matrix for model m
%
% OUTPUTS (where X is the hidden state being estimated)
% x(:,t) = E[X(:,t) | y(:,1:t)]
% V(:,:,t) = Cov[X(:,t) | y(:,1:t)]
% VV(:,:,t) = Cov[X(:,t), X(:,t-1) | y(:,1:t)] t >= 2
% loglik = sum{t=1}^T log P(y(:,t))
%
% If an input signal is specified, we also condition on it:
% e.g., x(:,t) = E[X(:,t) | y(:,1:t), u(:, 1:t)]
% If a model sequence is specified, we also condition on it:
% e.g., x(:,t) = E[X(:,t) | y(:,1:t), u(:, 1:t), m(1:t)]
%}
[os T] = size(y);
ss = size(A,1); % size of state space;% set default params
model = ones(1,T);
u = [];
B = [];
ndx = [];
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args
        case 'model', model = args{i+1};
        case 'u', u = args{i+1};
        case 'B', B = args{i+1};
        case 'ndx', ndx = args{i+1};
        otherwise, error(['unrecognized argument ' args])
    end
end
x = zeros(ss, T);
V = zeros(ss, ss, T);
VV = zeros(ss, ss, T);
loglik = 0;
for t=1:T
    m = model(t);
if t==1
    prevx = init_x;%prevx = init_x(:,m);
    prevV = init_V;%prevV = init_V(:,:,m);
initial = 1;
else
    prevx = x(:,t-1);
    prevV = V(:,:,t-1);
    initial = 0;
end
if isempty(u)
    [x(:,t), V(:,:,t), LL, VV(:,:,t)] = ...
    kalman_update(A(:,:,m), C(:,:,m), Q(:,:,m), R(:,:,m), y(:,t), prevx, prevV, 'initial', initial);
else
if isempty(ndx)
    [x(:,t), V(:,:,t), LL, VV(:,:,t)] = ...
    kalman_update(A(:,:,m), C(:,:,m), Q(:,:,m), R(:,:,m), y(:,t), prevx, prevV,'initial', initial, 'u', u(:,t), 'B', B(:,:,m));
else
i = ndx;
% copy over all elements; only some will get updated
x(:,t) = prevx;
prevP = inv(prevV);
prevPsmall = prevP(i,i);
prevVsmall = inv(prevPsmall);
[x(i,t), smallV, LL, VV(i,i,t)] = ...
kalman_update(A(i,i,m), C(:,i,m), Q(i,i,m), R(:,:,m), y(:,t), prevx(i), prevVsmall, ...
'initial', initial, 'u', u(:,t), 'B', B(i,:,m));
smallP = inv(smallV);
prevP(i,i) = smallP;
V(:,:,t) = inv(prevP);
end
end
loglik = loglik + LL;
end
% end
Z=(1:100); %�۲�ֵ
noise=randn(1,100); %����Ϊ1�ĸ�˹����
Z=Z+noise;
X=[0;0]; %״̬
P=[10;01]; %״̬Э�������
F=[11;01]; %״̬ת�ƾ���
Q=[0.0001,0;00.0001]; %״̬ת��Э�������
H=[10]; %�۲����
R=1; %�۲���������
figure;
hold on;
for i=1:100
    X_ = F*X;
    P_ = F*P*F'+Q;
    K = P_*H'/(H*P_*H'+R);
    X = X_+K*(Z(i)-H*X_);
    P = (eye(2)-K*H)*P_;
    plot(X(1), X(2)); %���㣬�����ʾλ�ã������ʾ�ٶ�
end
end





