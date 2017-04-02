function dX = my_dynamics(t,X,xOpt)

%% constants
[g,GR,N,l,Mass,Nq] = getParams();

%% decompose xOpt
[z0, T_st, delta_x, alpha_x, q, dq] = decompose_x(xOpt);

% dynamics
z = X(1);
dz = X(2);

ss = t/T_st;
Fz = polyval_bz([0, alpha_x, 0],ss);     % 6th order Bezier polynomial
ddz = -g + Fz/Mass;

dX = [dz;ddz];

end