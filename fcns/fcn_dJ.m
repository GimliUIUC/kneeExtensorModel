function [dJ] = fcn_dJ(q,dq,params)

dJ = zeros(2,2);

  dJ(1,1)=- dq(1)*(params(1)*cos(q(1) + q(2)) + params(1)*cos(q(1))) - dq(2)*params(1)*cos(q(1) + q(2));
  dJ(1,2)=- dq(1)*params(1)*cos(q(1) + q(2)) - dq(2)*params(1)*cos(q(1) + q(2));
  dJ(2,1)=- dq(1)*(params(1)*sin(q(1) + q(2)) + params(1)*sin(q(1))) - dq(2)*params(1)*sin(q(1) + q(2));
  dJ(2,2)=- dq(1)*params(1)*sin(q(1) + q(2)) - dq(2)*params(1)*sin(q(1) + q(2));

 