function out = eval_trajectory(traj, t)
    if t <= traj.t(1)
        out.phi = traj.phi(1,:);
        out.eps = traj.eps(1,:);
        out.lamb = traj.lamb(1,:);
        out.vf = traj.vf(1,:);
        out.vb = traj.vb(1,:);
    elseif t > traj.t(end)
        out.phi = traj.phi(end,:);
        out.eps = traj.eps(end,:);
        out.lamb = traj.lamb(end,:);
        out.vf = traj.vf(end,:);
        out.vb = traj.vb(end,:);
    else
        out.phi = interp1(traj.t, traj.phi, t);
        out.eps = interp1(traj.t, traj.eps, t);
        out.lamb = interp1(traj.t, traj.lamb, t);
        out.vf = interp1(traj.t, traj.vf, t);
        out.vb = interp1(traj.t, traj.vb, t);
    end
end

