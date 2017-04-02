function P = forward(q)
%% calculates the position of hip motor given q1 and q2
[g,GR,N,l,Mass,Nq] = getParams();

q1 = q(1);
q2 = q(2);
P = rot(q1)*[l;0] + rot(q1+q2)*[l;0];

P(2) = -P(2);

end
