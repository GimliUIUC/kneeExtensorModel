%% setting up
addpath('fcns')
addpath('visualization')

k_w = 1;
k_t = 1;

[g,GR,N,l,Mass,Nq] = getParams();

%% initial guess
z0 = 0.1;
T_st0 = 0.1;
delta_x0 = 0;
alpha0 = [3 6 9 12 15]; 
q0 = [-0.5738*ones(1,N),-2.3664*ones(1,N)];
dq0 = zeros(1,2*N); 

xOpt0 = [z0 T_st0 delta_x0 alpha0  q0 dq0]; % initial condition
%% bounds
Lbz = 0.08;
Ubz = 0.2;
LbTst = 0.05;
UbTst = 1;
Lbdel_x = 0;
Ubdel_x = 0;
Lbalpha = [0 1 1 1 1];
Ubalpha = [100 300 300 300 300];

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
lb = [Lbz LbTst Lbdel_x Lbalpha Lbq Lbdq];
ub = [Ubz UbTst Ubdel_x Ubalpha Ubq Ubdq];

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

[xOpt, fval] = fmincon(@KEM_cost,xOpt0,A,b,Aeq,beq,lb,ub,@(x)KEM_con(x,k_w,k_t),options)







