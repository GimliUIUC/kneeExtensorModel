function dX = my_dynamics(t,X,param)
[N,s,M,Nq] = getParams();
g = 9.81;

alpha = param.alpha;
T_stance = param.T_stance;

z = X(1);
dz = X(2);

s = t/T_stance;
Fz = polyval_bz([0, alpha],s);
ddz = -g + Fz/M;

dX = [dz;ddz];

end