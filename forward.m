function P = forward(q)
%% calculates the position of hip motor given q1 and q2
[N,s,M,Nq] = getParams();
q1 = q(1);
q2 = q(2);
P = rot(q1)*[s;0] + rot(q1+q2)*[s;0];

P(2) = -P(2);

end
