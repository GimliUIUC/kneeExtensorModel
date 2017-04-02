        %%%generate 1 link inverted pendulum
        %%%Author: Hae-Won Park
        syms x z q1 q2 real         %generalized Coordinate 
        syms dx dz dq1 dq2 real
        syms s real                 % leg segment length
        syms M m real               % mass of each link
        syms J1 J2 real             % inertia of transmission
        g = 9.81;
        
        q =[q1 q2]';
        dq = [dq1 dq2]';
        
        m_list_params = {
            's', 'params(1)';       % leg segment length
            'M', 'params(2)'};       % body weight

        m_list_q = {
            'q1', 'q(1)';
            'q2', 'q(2)'};
        
        m_list_dq = {
            'dq1', 'dq(1)';
            'dq2', 'dq(2)'};

%% forward kinematics
        p_HIP = [0;0];
        p_KNEE = p_HIP + rot(q1)*[s;0];
        p_FOOT = p_KNEE + rot(q1+q2)*[s;0];
        J = jacobian(p_FOOT,q);
        Jx = jacobian(p_HIP(1),q);
        
        dJ = sym('dJ',size(J));
        for i = 1:size(J,2)
            dJ(:,i) = jacobian(J(:,i),q)*dq;
        end
        
        dJx = sym('dJx',size(Jx));
        for i = 1:size(Jx,2)
            dJx(:,i) = jacobian(Jx(:,i),q)*dq;
        end

%% Velocity kinematics
        v_COM = [dx;dz];
        p_thigh_COM = (p_HIP+p_KNEE)/2;
        p_shank_COM = (p_FOOT+p_KNEE)/2;
        v_upper = jacobian(p_thigh_COM,q)*dq;
        v_lower = jacobian(p_shank_COM,q)*dq;

%% Kinetic Energy, Potential Energy, and Lagrangian
        J_HIP = 1/12*m*s^2 + J1;
        J_KNEE = 1/12*m*s^2 + J2;
        KE = 0.5*(M)*(v_COM.'*v_COM);

        PE = g*(M*z);

        Upsilon = q2; % where control torques go
        
%% Euler-Lagrange Equation
        [D, C, G, B] = std_dynamics(KE,PE,q,dq, Upsilon);
          
        m_output_dir = [pwd '\fcns'];
        write_fcn_m([m_output_dir '\fcn_D.m'],{'q', 'params'},[m_list_q;m_list_params],{D,'D'});
        write_fcn_m([m_output_dir '\fcn_C.m'],{'q','dq', 'params'},[m_list_q;m_list_dq;m_list_params],{C,'C'});
        write_fcn_m([m_output_dir '\fcn_G.m'],{'q', 'params'},[m_list_q;m_list_params],{G,'G'});
        write_fcn_m([m_output_dir '\fcn_B.m'],{'q', 'params'},[m_list_q;m_list_params],{B,'B'});
        write_fcn_m([m_output_dir '\fcn_J.m'],{'q', 'params'},[m_list_q;m_list_params],{J,'J'});
        write_fcn_m([m_output_dir '\fcn_dJ.m'],{'q','dq', 'params'},[m_list_q;m_list_dq;m_list_params],{dJ,'dJ'});


        
        