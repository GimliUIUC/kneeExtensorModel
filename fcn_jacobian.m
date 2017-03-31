function J = fcn_jacobian(q,param);
l1 = param.l1;
l2 = param.l2;
q1 = q(1);
q2 = q(2);

J = [ - l2*sin(q1 + q2) - l1*sin(q1), -l2*sin(q1 + q2);
    l2*cos(q1 + q2) + l1*cos(q1),  l2*cos(q1 + q2)];
end