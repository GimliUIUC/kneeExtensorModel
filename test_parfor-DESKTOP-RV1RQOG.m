
% w_nom = 33.4027 rad/s
% t_norm = 9.8109 Nm

[N,s,M,Nq] = getParams();
N_x = Nq + 4*N-1;
% k_w = [0.7 0.8 0.9 1 1.1 1.2];
k = [0.7 0.8 0.8 0.8 0.9 0.9 1;
     0.7 0.8 0.9 1.0 0.9 1.0 1];

x_opt = zeros(length(k),N_x);
for i = 1:length(k)
    temp = main_exe(k(1,i),k(2,i));
    x_opt(i,:) = temp;
end

%%
for i = 1:length(k)
    tau_vs_omega(x_opt(i,:))
    grid on
    visualization(x_opt(i,:))

end



