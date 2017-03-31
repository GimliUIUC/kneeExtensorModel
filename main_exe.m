%% main
addpath('fcns')

k_w = 1;
k_t = 1

[N,s,M,Nq] = getParams();

%% initial guess
z0 = 0.1;
alpha0 = [3 6 9 12 15]; 
T_st0 = 0.1;
q0 = [-0.5738*ones(1,N),-2.3664*ones(1,N)];
dq0 = zeros(1,2*N); 

x0 = [z0 alpha0 T_st0 q0 dq0]; % initial condition
%% bounds
Lbalpha = [0 1 1 1 1];
Ubalpha = [100 300 300 300 300];

GR = 23.3594;
speed_max = 7451*2*pi/(60*GR)*k_w; % rpm to rad/s
Lbq = [ones(1,N)*(-pi/2),ones(1,N)*(-pi)];
Ubq = [ones(1,N)*(0),ones(1,N)*(0)];
Lbdq = ones(size(dq0))*(-speed_max);
Ubdq = ones(size(dq0))*(speed_max);

A = [];
b = [];
Aeq = [];
beq = [];
% bounds on optimization variables
lb = [0.08 Lbalpha 0.05 Lbq Lbdq];
ub = [0.2 Ubalpha 1.00 Ubq Ubdq];

%% optimization
options = optimset( 'Display','iter',...
                    'Algorithm','sqp', ...
                    'MaxIter', 10000,...
                    'MaxFunEvals', 10000,...
                    'RelLineSrchBnd',0.1,...
                    'TolX',1e-7,...
                    'AlwaysHonorConstraints','Bounds',...
                    'TolFun',1e-6,...
                    'TolCon',1e-6);

[x, fval] = fmincon(@my_cost,x0,A,b,Aeq,beq,lb,ub,@(x)non_con(x,k_w,k_t),options)

% [x_new fval_new] = fmincon(...
%     @(x)0,x0,A,b,Aeq,beq,lb,ub,@non_con,options)




