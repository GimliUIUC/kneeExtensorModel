function [cineq,ceq] = KEM_con(xOpt,k_w,k_t)

[g,GR,N,l,Mass,Nq] = getParams();

%% --- decompose xOpt ---
[z0, T_st, delta_x, alpha_x, q, dq] = decompose_x(xOpt);

q1 = q(1:N)';
q2 = q(N+1:2*N)';
dq1 = dq(1:N)';
dq2 = dq(N+1:2*N)';
q2col = [q1,q2];
dq2col = [dq1,dq2];

%% --- simulate dynamics ---
ic = [z0;0];
t = linspace(0, T_st,N);
[t, X] = ode45(@(t,X)my_dynamics(t,X,xOpt),t,ic);

% ss and Fz are vectors of length(t)
ss = t/T_st;
Fz = polyval_bz([0, alpha_x,0],ss);

parameters = [l,Mass];          % parameter for Jacobian calculation
J = zeros(2,2,N);
tau = zeros(2,N);
for ii=1:N
    qq = [q1(ii),q2(ii)];
    J(:,:,ii) = fcn_J(qq,parameters);
    tau(:,ii) = J(:,:,ii)'*[0;Fz(ii)];
end

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
ceq = [];
for jj = 1:N
    ceq = [ceq;forward(q2col(jj,:)')-[0;X(jj,1)]];    % no slip
    ceq = [ceq;-J(:,:,jj)*dq2col(jj,:)' - [0;X(jj,2)]];       % no relative v
end

% [value index] = max(ceq);
% if value>0 
%     disp(['maximum = ' num2str(value) '  index = ' num2str(index)]);
% end

