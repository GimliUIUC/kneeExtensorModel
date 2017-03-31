% x_travel = 0.4;
% s_max = 0.5;
% M = 5;
% alpha_x0 = alpha_x;
% alpha_y0 = alpha_y;
% x(1) = x_out(1,1);
% x(2) = x_out(1,2);
% x(3) = x_out(1,3);
% x(4) = x_out(1,4);
% x(5) = x_out(1,5);
% x(6) = x_out(1,6);
% x(7) = alpha_x0(5);
% x(8) = alpha_x0(6);
% x(9) = alpha_y0(5);
% x(10) = alpha_y0(6);
% x(11) = alpha_x_0(6);

function dynamics_Bezier_7()

m_dir = 'C:\Users\Haewon\Box Sync\DynamicRoboticsLaboratory\Simulation\CMM\Generate CMM\CMM_App_0731_Chad\fcn_gen\SimpleModel\';

syms MTor JTor LTor real
g = 9.81;
M = 5;
% s_max_f = 0.5;
% s_max_h = 0.5;
syms s_max_f real
% T_stance = 0.4/vd;
% T_swing = 0.22;
% T_air = (T_swing-T_stance)/2;

% T_air_back = (T_swing-T_stance)/2;
syms T_stance T_air real% M s_max_f real
T = [T_stance T_air T_stance T_air];
syms cth0 th0 x0 y0 dth0 dx0 dy0 x_travel Fx Fy real
syms xf yf dxf dyf real
mag_x_f = Fx;
mag_y_f = Fy;


syms s_max_f_T real
AA = [M*(M-1)/(s_max_f_T)^2*[1 -2 1 0 0 0 ; 0 1 -2 1 0 0 ; 0 0 1 -2 1 0 ; 0 0 0 1 -2 1]; ...
    -M/(s_max_f_T) M/(s_max_f_T) 0 0 0 0; 1 0 0 0 0 0];
beta_x = mag_x_f*[0 1 1 1];
bb = [beta_x(1)/MTor;beta_x(2)/MTor;beta_x(3)/MTor;beta_x(4)/MTor;dx0;x0];
alpha_x_af = (((AA)\bb)');

% AA_back = [M*(M-1)/(s_max_f*T(1))^2*[1 -2 1 0 0 0 ; 0 1 -2 1 0 0 ; 0 0 1 -2 1 0 ; 0 0 0 1 -2 1]; -M/(s_max_f*T(1)) M/(s_max_f*T(1)) 0 0 0 0; 1 0 0 0 0 0];
% bb_back = [beta_x(4)/MTor;beta_x(3)/MTor;beta_x(2)/MTor;beta_x(1)/MTor;-dxf;xf];
% alpha_x_af_back = (((AA_back)\bb_back)');



AA = [M*(M-1)/(s_max_f_T)^2*[1 -2 1 0 0 0 ; 0 1 -2 1 0 0 ; 0 0 1 -2 1 0 ; 0 0 0 1 -2 1];...
    -M/(s_max_f_T) M/(s_max_f_T) 0 0 0 0; 1 0 0 0 0 0];
beta_y = mag_y_f*[0 1 1 1];
bb = [beta_y(1)/MTor-g;beta_y(2)/MTor-g;beta_y(3)/MTor-g;beta_y(4)/MTor-g;dy0;y0];
alpha_y_af = (((AA)\bb)');

% AA_back = [M*(M-1)/(s_max_f*T(1))^2*[1 -2 1 0 0 0 ; 0 1 -2 1 0 0 ; 0 0 1 -2 1 0 ; 0 0 0 1 -2 1]; -M/ (s_max_f*T(1)) M/(s_max_f*T(1)) 0 0 0 0; 1 0 0 0 0 0];
% bb_back = [beta_y(4)/MTor-g;beta_y(3)/MTor-g;beta_y(2)/MTor-g;beta_y(1)/MTor-g;-dyf;yf];
% alpha_y_af_back = (((AA_back)\bb_back)');


% AA = [M*(M-1)/(s_max_f*T(1))^2*[1 -2 1 0 0 0 ; 0 1 -2 1 0 0 ; 0 0 1 -2 1 0 ; 0 0 0 1 -2 1]; -M/(s_max_f*T(1)) M/(s_max_f*T(1)) 0 0 0 0; 1 0 0 0 0 0];
% beta_y = mag_y_f*[0 1 1 1];
% bb = [beta_y(4)/MTor-g;beta_y(3)/MTor-g;beta_y(2)/MTor-g;beta_y(1)/MTor-g;dy0;y0];


beta_pitch = ((get_bezier_multiplication(-alpha_x_af,beta_y)+get_bezier_multiplication(alpha_y_af,beta_x))/JTor);
M_pitch = 10;
AA = [M_pitch*(M_pitch-1)/(s_max_f_T)^2*[1 -2 1 0 0 0 0 0 0 0 0; 0 1 -2 1 0 0 0 0 0 0 0; ...
    0 0 1 -2 1 0 0 0 0 0 0; 0 0 0 1 -2 1 0 0 0 0 0;
     0 0 0 0 1 -2 1  0 0 0 0; 0 0 0 0 0 1 -2 1  0 0 0 ; 0 0 0 0 0 0 1 -2 1  0 0 ;...
     0 0 0 0 0 0 0 1 -2 1  0 ; 0 0 0 0 0 0 0 0 1 -2 1];...
    -M_pitch/(s_max_f_T) M_pitch/(s_max_f_T) 0 0 0 0 0 0 0 0 0; 1 0 0 0 0 0 0 0 0 0 0];
BB = [beta_pitch';dth0;th0];
alpha_pitch_af = ((AA\BB)');

% x_sol(1,:) = [y_minus];
% x_sol(1,2) = x_sol(1,2)-x_offset_1;
y_plus = forward_stance_1(alpha_x_af,alpha_y_af,alpha_pitch_af, s_max_f_T);


% x_sol(2,:) = [y_plus];
% x_sol(2,2) = x_sol(2,2)-x_offset_1;
th0 = y_plus(1);
x0 = y_plus(2);
y0 = y_plus(3);
dth0 = y_plus(4);
dx0 = y_plus(5);
dy0 = y_plus(6);

syms s_max_f_T_1 real
AA = [M*(M-1)/(s_max_f_T_1)^2*[1 -2 1 0 0 0 ; 0 1 -2 1 0 0 ; 0 0 1 -2 1 0 ; 0 0 0 1 -2 1]; ...
    -M/(s_max_f_T_1) M/(s_max_f_T_1) 0 0 0 0; 1 0 0 0 0 0];
beta_x = mag_x_f*[1 1 1 0];
bb = [beta_x(1)/MTor;beta_x(2)/MTor;beta_x(3)/MTor;beta_x(4)/MTor;dx0;x0];
alpha_x_bf = (((AA)\bb)');

AA = [M*(M-1)/(s_max_f_T_1)^2*[1 -2 1 0 0 0 ; 0 1 -2 1 0 0 ; 0 0 1 -2 1 0 ; 0 0 0 1 -2 1];...
    -M/(s_max_f_T_1) M/(s_max_f_T_1) 0 0 0 0; 1 0 0 0 0 0];
beta_y = mag_y_f*[1 1 1 0];
bb = [beta_y(1)/MTor-g;beta_y(2)/MTor-g;beta_y(3)/MTor-g;beta_y(4)/MTor-g;dy0;y0];
alpha_y_bf = (((AA)\bb)');



beta_pitch = ((get_bezier_multiplication(-alpha_x_bf,beta_y)+get_bezier_multiplication(alpha_y_bf,beta_x))/JTor);
M_pitch = 10;
AA = [M_pitch*(M_pitch-1)/(s_max_f_T_1)^2*[1 -2 1 0 0 0 0 0 0 0 0; 0 1 -2 1 0 0 0 0 0 0 0; ...
    0 0 1 -2 1 0 0 0 0 0 0; 0 0 0 1 -2 1 0 0 0 0 0;...
     0 0 0 0 1 -2 1  0 0 0 0; 0 0 0 0 0 1 -2 1  0 0 0 ; 0 0 0 0 0 0 1 -2 1  0 0 ; ...
     0 0 0 0 0 0 0 1 -2 1  0 ; 0 0 0 0 0 0 0 0 1 -2 1];...
    -M_pitch/(s_max_f_T_1) M_pitch/(s_max_f_T_1) 0 0 0 0 0 0 0 0 0; 1 0 0 0 0 0 0 0 0 0 0];
BB = [beta_pitch';dth0;th0];
alpha_pitch_bf = ((AA\BB)');


m_list_params = {'MTor' 'params(1)'; 'JTor' 'params(2)'; 'LTor' 'params(3)'};

m_list_s_max_T = {'s_max_f_T' 's_max_T(1)';'s_max_f_T_1' 's_max_T(2)'};

T = [T_stance T_air T_stance T_air];
syms th0 x0 y0 dth0 dx0 dy0 x_travel Fx Fy real


    
m_list_q0 = {'th0','q0(1)';'x0','q0(2)';'y0','q0(3)';'dth0','q0(4)';'dx0','q0(5)';'dy0','q0(6)'};
m_list_Force = {'Fx','F(1)';'Fy','F(2)'};



alpha_x_af = simplify(alpha_x_af);
alpha_x_bf = simplify(alpha_x_bf);
alpha_y_af = simplify(alpha_y_af);
alpha_y_bf = simplify(alpha_y_bf);
alpha_pitch_af = simplify(alpha_pitch_af);
alpha_pitch_bf = simplify(alpha_pitch_bf);
% 
% 
write_fcn_m([m_dir,'fcn_alpha_x_af_s.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_x_af,'alpha_x_af'});
write_fcn_m([m_dir,'fcn_alpha_x_bf_s.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_x_bf,'alpha_x_bf'});
write_fcn_m([m_dir,'fcn_alpha_y_af_s.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_y_af,'alpha_y_af'});
write_fcn_m([m_dir,'fcn_alpha_y_bf_s.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_y_bf,'alpha_y_bf'});
write_fcn_m([m_dir,'fcn_alpha_pitch_af_s.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_pitch_af,'alpha_pitch_af'});
write_fcn_m([m_dir,'fcn_alpha_pitch_bf_s.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_pitch_bf,'alpha_pitch_bf'});

write_fcn_m_sdp([m_dir,'fcn_alpha_x_af_s_sdp.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_x_af,'alpha_x_af'});
write_fcn_m_sdp([m_dir,'fcn_alpha_x_bf_s_sdp.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_x_bf,'alpha_x_bf'});
write_fcn_m_sdp([m_dir,'fcn_alpha_y_af_s_sdp.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_y_af,'alpha_y_af'});
write_fcn_m_sdp([m_dir,'fcn_alpha_y_bf_s_sdp.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_y_bf,'alpha_y_bf'});
write_fcn_m_sdp([m_dir,'fcn_alpha_pitch_af_s_sdp.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_pitch_af,'alpha_pitch_af'});
write_fcn_m_sdp([m_dir,'fcn_alpha_pitch_bf_s_sdp.m'],{'q0','F','params','s_max_T'},[m_list_q0;m_list_Force;m_list_params;m_list_s_max_T],{alpha_pitch_bf,'alpha_pitch_bf'});

% y_plus = (forward_stance_1(alpha_x_bf,alpha_y_bf, alpha_pitch_bf,(1-s_max_f)*T(1)));

% x_sol(3,:) = [y_plus];
% x_sol(3,2) = x_sol(3,2)-x_offset_1;
% y_plus(2) = x_sol(3,2);
