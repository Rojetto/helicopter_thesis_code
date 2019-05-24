function dx = system_f(x, u)
    term_centripetal = true;
    term_friction = true;
    term_dynamic_intertia = true;
    term_motor_pt1 = true;
    term_motor_reaction = true;
    term_rotor_gyro = true;
    

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

    c = Constants();

    if term_dynamic_intertia
        J_phi = c.Jp;
        J_eps_1 = c.m_c*c.l_c^2 + 2*c.m_p*(c.l_h^2+c.l_p^2*sin(phi)^2);
        J_eps_2 = c.m_c*c.l_c^2 + 2*c.m_p*(c.l_h^2-c.l_p^2*sin(phi)^2);
        J_lamb = c.m_c*(c.l_c*cos(eps))^2 + 2*c.m_p*((c.l_h*cos(eps))^2 + (c.l_p*sin(phi)*sin(eps))^2 + (c.l_p * cos(phi))^2);
    else
        J_phi = c.Jp;
        J_eps_1 = c.Je;
        J_eps_2 = c.Je;
        J_lamb = c.Jl;
    end

    if term_motor_pt1
        dwf = 1/c.T_f * (c.K_f * vf - wf);
        dwb = 1/c.T_b * (c.K_b * vb - wb);
    else
        wf = vf;
        wb = vb;
        dwf = 0;
        dwb = 0;
    end

    ws = wf + wb;
    wd = wf - wb;

    ddphi_rhs = wd * c.L1;
    ddeps_rhs = c.L2 * cos(eps) + ws * c.L3 * cos(phi);
    ddlamb_rhs = ws * c.L4 * cos(eps) * sin(phi);

    if term_centripetal
        ddphi_rhs = ddphi_rhs + J_phi * cos(phi) * sin(phi) * (deps^2 - cos(eps)^2*dlamb^2);
        ddeps_rhs = ddeps_rhs - J_eps_2 * cos(eps) * sin(eps) * dlamb^2;
    end

    if term_friction
        ddphi_rhs = ddphi_rhs - c.mup * dphi;
        ddeps_rhs = ddeps_rhs - c.mue * deps;
        ddlamb_rhs = ddlamb_rhs - c.mul * dlamb;
    end

    if term_motor_reaction
        ddeps_rhs = ddeps_rhs + sin(phi) * c.K_m * wd;
        ddlamb_rhs = ddlamb_rhs - cos(eps) * cos(phi) * c.K_m * wd;
    end

    if term_rotor_gyro
        ddphi_rhs = ddphi_rhs - c.J_m * cos(phi) * deps * wd + c.J_m * sin(phi) * cos(eps) * dlamb * wd;
        ddeps_rhs = ddeps_rhs + c.J_m * cos(phi) * dphi * wd - c.J_m * cos(phi) * sin(eps) * dlamb * wd;
        ddlamb_rhs = ddlamb_rhs + c.J_m * sin(phi) * cos(eps) * dphi * wd;
    end

    ddphi = ddphi_rhs / J_phi;
    ddeps = ddeps_rhs / J_eps_1;
    ddlamb = ddlamb_rhs / J_lamb;

    dx = [dphi; deps; dlamb; ddphi; ddeps; ddlamb; dwf; dwb];
end

