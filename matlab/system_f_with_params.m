function dx = system_f_with_params(x, u, ...
    m_c, m_h, l_c, l_h, l_p, d_c, d_h,...
    mup, mue, mul, ...
    T_w, K_w, ...
    p1, q1, p2, q2, ...
    J_m, K_m)

    term_centripetal = false;
    term_friction = true;
    term_dynamic_inertia = true;
    term_motor_pt1 = true;
    term_motor_nonlinear = true;
    term_motor_reaction = false;
    term_rotor_gyro = false;

    g = 9.81;
    
    phi = x(1);
    eps = x(2);
    lamb = x(3);
    dphi = x(4);
    deps = x(5);
    dlamb = x(6);
    wf = x(7);
    wb = x(8);

    vf = u(1);
    vb = u(2);

    if term_dynamic_inertia
        p_phi_1 = m_h*(l_p^2+d_h^2);
        p_eps_1 = m_c*(l_c^2+d_c^2) + m_h*(l_h^2+d_h^2) + m_h*sin(phi)^2*(l_p^2-d_h^2);
        p_lamb_1 = -d_c^2*m_c*cos(eps)^2-d_c*l_c*m_c*sin(2*eps)-d_h^2*m_h*cos(eps)^2*cos(phi)^2+d_h^2*m_h - d_h*l_h*m_h/2*(sin(2*eps-phi)+sin(2*eps+phi))+l_c^2*m_c*cos(eps)^2+l_h^2*m_h*cos(eps)^2+l_p^2*m_h*cos(eps)^2*cos(phi)^2-l_p^2*m_h*cos(eps)^2+l_p^2*m_h;
    else
        p_phi_1 = m_h*(l_p^2+d_h^2);
        p_eps_1 = m_c*(l_c^2+d_c^2) + m_h*(l_h^2+d_h^2);
        p_lamb_1 = m_h*(l_h^2+l_p^2) + m_c*l_c^2;
    end
    
    p_phi_2 = - g*d_h*m_h*cos(eps);
    p_eps_2 = g*(d_c*m_c - d_h*m_h*cos(phi));
    p_eps_3 = g*(l_h*m_h - m_c*l_c);

    if term_motor_pt1
        dwf = 1/T_w * (K_w * vf - wf);
        dwb = 1/T_w * (K_w * vb - wb);
    else
        wf = vf;
        wb = vb;
        dwf = 0;
        dwb = 0;
    end

    if term_motor_nonlinear
%         syms Fr(w)
%         Fr(w) = piecewise(w<=-2*q2/p2, p2*w+q2, -2*q2/p2<w<=0,-p2^2/(4*q2)*w^2,0<w<=2*q1/p1,p1^2/(4*q1) * w^2,p1*w-q1);

        Ff = Fr(wf, p1, q1, p2, q2);
        Fb = Fr(wb, p1, q1, p2, q2);
    else
        Ff = wf;
        Fb = wb;
    end

    Fs = Ff + Fb;
    Fd = Ff - Fb;

    ws = wf + wb;
    wd = wf - wb;

    ddphi_rhs = -p_phi_2 * sin(phi) + Fd * l_p;
    ddeps_rhs = -p_eps_2 * sin(eps) - p_eps_3 * cos(eps) + Fs * l_h * cos(phi);
    ddlamb_rhs = Fs * l_h * cos(eps) * sin(phi) - Fd * l_p * sin(eps);

    if term_centripetal
        ddphi_rhs = ddphi_rhs + J_phi * cos(phi) * sin(phi) * (deps^2 - cos(eps)^2*dlamb^2);
        ddeps_rhs = ddeps_rhs - J_eps_2 * cos(eps) * sin(eps) * dlamb^2;
    end

    if term_friction
        ddphi_rhs = ddphi_rhs - mup * dphi;
        ddeps_rhs = ddeps_rhs - mue * deps;
        ddlamb_rhs = ddlamb_rhs - mul * dlamb;
    end

    if term_motor_reaction
        ddeps_rhs = ddeps_rhs + sin(phi) * K_m * wd;
        ddlamb_rhs = ddlamb_rhs - cos(eps) * cos(phi) * K_m * wd;
    end

    if term_rotor_gyro
        ddphi_rhs = ddphi_rhs - J_m * cos(phi) * deps * wd + J_m * sin(phi) * cos(eps) * dlamb * wd;
        ddeps_rhs = ddeps_rhs + J_m * cos(phi) * dphi * wd - J_m * cos(phi) * sin(eps) * dlamb * wd;
        ddlamb_rhs = ddlamb_rhs + J_m * sin(phi) * cos(eps) * dphi * wd;
    end

    ddphi = ddphi_rhs / p_phi_1;
    ddeps = ddeps_rhs / p_eps_1;
    ddlamb = ddlamb_rhs / p_lamb_1;

    dx = [dphi; deps; dlamb; ddphi; ddeps; ddlamb; dwf; dwb];
end

