function q = fcn_inv(p)
[N,s,M,Nq] = getParams();

h = p(2);  % vertical position of the hip
r = sqrt(h^2);
numA = r^2;
denA = 2*r*s;
A = acos(numA/denA);
B = atan(0);

q1 = -(pi/2-A-B);

numC = 2*s^2-r^2;
denC = 2*s^2;
C = acos(numC/denC);
q2 = -(pi-C);

q = [q1,q2];


end