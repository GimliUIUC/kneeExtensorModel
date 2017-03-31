function cost = my_cost(x,param)
[N,s,M,Nq] = getParams();
g = 9.81;

%% --- decompose x ---
z0 = x(1);
param.alpha = x(2:6);       % 5th order bezier polynomial
param.T_stance = x(7);
q1 = x(Nq:Nq+N-1);
q2 = x(Nq+N:Nq+2*N-1);
dq1 = x(Nq+2*N:Nq+3*N-1);
dq2 = x(Nq+3*N:Nq+4*N-1);

%% 
ic = [z0;0];
t = linspace(0, param.T_stance,N);
[t,X] = ode45(@(t,X)my_dynamics(t,X,param),t,ic);


%%
vz_f = X(end,2);

cost = -vz_f^2;

end
