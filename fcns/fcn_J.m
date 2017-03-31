function [J] = fcn_J(q,params)

J = zeros(2,2);

  J(1,1)=- params(1)*sin(q(1) + q(2)) - params(1)*sin(q(1));
  J(1,2)=-params(1)*sin(q(1) + q(2));
  J(2,1)=params(1)*cos(q(1) + q(2)) + params(1)*cos(q(1));
  J(2,2)=params(1)*cos(q(1) + q(2));

 