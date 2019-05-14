function u = cascade_pid(t, x, e_traj, lambda_traj, params)
    persistent elevation_pid
    persistent travel_pitch_pid
    persistent pitch_vd_pid
    
    if isempty(elevation_pid)
        elevation_pid = pidAlgorithm(params(1:3));
        travel_pitch_pid = pidAlgorithm(params(4:6));
        pitch_vd_pid = pidAlgorithm(params(7:9));
    end
    
    e_error = x(2) - e_traj(1);
    delta_ws_d = - elevation_pid.compute(t, e_error, x(5));

    % outer loop: travel --> pitch
    lambda_error = x(3) - lambda_traj(1);
    p_op = - travel_pitch_pid.compute(t, lambda_error, x(6));

    % inner loop: pitch --> Vd
    p_error = x(1) - p_op;
    delta_wd_d = - pitch_vd_pid.compute(t, p_error, x(4));

    delta_wf_d = (delta_ws_d + delta_wd_d) / 2;
    delta_wb_d = (delta_ws_d - delta_wd_d) / 2;

    Vf = delta_wf_d;
    Vb = delta_wb_d;

    u = [Vf; Vb];
end

