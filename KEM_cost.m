function cost = KEM_cost(xOpt)

[g,GR,N,l,Mass,Nq] = getParams();


%% --- decompose x ---
[z0, T_st, delta_x, alpha_x, q, dq] = decompose_x(xOpt);

%% 
ic = [z0;0];
t = linspace(0, T_st,N);
[t,X] = ode45(@(t,X)my_dynamics(t,X,xOpt),t,ic);


%%
vz_f = X(end,2);

cost = -vz_f^2;

end
