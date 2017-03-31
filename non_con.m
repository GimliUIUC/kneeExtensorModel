function [cineq,ceq] = non_con(x,k_w,k_t)
[N,s,M,Nq] = getParams();
g = 9.81;
GR = 23;

%% --- decompose x ---
z0 = x(1);
param.alpha = x(2:6);
param.T_stance = x(7);
q1 = x(Nq:Nq+N-1);
q2 = x(Nq+N:Nq+2*N-1);
dq1 = x(Nq+2*N:Nq+3*N-1);
dq2 = x(Nq+3*N:Nq+4*N-1);

q = [q1;q2]';       % n by 2
dq = [dq1;dq2]';    % n by 2

%% --- simulate dynamics ---
ic = [z0;0];
t = linspace(0, param.T_stance,N);
[t, X] = ode45(@(t,X)my_dynamics(t,X,param),t,ic);

ss = t/param.T_stance;
Fz = polyval_bz([0, param.alpha],ss);


parameters = [s,M];
J = zeros(2,2,size(X,1));
u = zeros(2,size(X,1));
for ii=1:size(X,1)
    z = X(ii,1);
    qq = [q1(ii),q2(ii)];
    J(:,:,ii) = fcn_J(qq,parameters);
    u(:,ii) = J(:,:,ii)'*[0;Fz(ii)];
end
tau = u;
%% --- constraints ---
speed_max = 7451*2*pi/(60*GR)*k_w;% rpm to rad/s
tor_max = 0.42*GR*k_t;
Ubdq = ones(size(dq))*(speed_max);
Lbdq = ones(size(dq))*(-speed_max);
Ubtau = ones(size(tau))*(tor_max);
Lbtau = ones(size(tau))*(-tor_max);

% --- inequality constraints ---
cineq = [];
cineq = [cineq;vec(dq-Ubdq)];   % vec reshapes matrix to vector
cineq = [cineq;vec(Lbdq-dq)];
cineq = [cineq;vec(tau-Ubtau)];
cineq = [cineq;vec(Lbtau-tau)];
cineq = [cineq;vec(-X(:,1)+0.02)];% constraints on z displacement
cineq = [cineq;vec(X(:,1)-0.23)];

% --- equality constraints ---
q = q';     % 2 by n
dq = dq';   % 2 by n
ceq = [];
for jj = 1:size(dq,2)
    ceq = [ceq;forward(q(:,jj))-[0;X(jj,1)]];    % no slip
    ceq = [ceq;-J(:,:,jj)*dq(:,jj) - [0;X(jj,2)]];       % no relative v
end

% [value index] = max(ceq);
% if value>0 
%     disp(['maximum = ' num2str(value) '  index = ' num2str(index)]);
% end

