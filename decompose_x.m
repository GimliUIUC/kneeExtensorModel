function [z0, T_st, delta_x, alpha_x, q, dq] = decompose_x(xOpt)

[g,GR,N,l,Mass,Nq] = getParams();

z0 = xOpt(1);
T_st = xOpt(2);
delta_x = xOpt(3);

alpha_x = xOpt(4:8);

q = xOpt(Nq:Nq+2*N-1);
dq = xOpt(Nq+2*N:end);



