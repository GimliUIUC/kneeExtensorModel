%% forward kinematics
syms l1 l2 q1 q2
q = [q1;q2];

P1 = rot(q1)*[l1;0];
P2 = P1 + rot(q1+q2)*[l2;0];
P2 = -P2;

J = jacobian(P2,q)




