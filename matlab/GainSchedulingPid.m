classdef GainSchedulingPid < HeliController
    %GainSchedulingPid Linear PID controllers in cascade structure.
    properties (Access = private)
        elevation_pid
        travel_pitch_pid
        pitch_vd_pid

        front_rotor_pid
        back_rotor_pid

        trajectory
    end
    
    properties (Nontunable)
        elevation_levels = [-60, -30, 0, 30, 60]
        %elevation_pid_gains Elevation PID
        elevation_pid_gains = [10, 4, 5]
        %travel_pitch_pid_gains Travel-Pitch PID
        travel_pitch_pid_gains = [3, 0.1, 1.5;
                                  4, 0.05, 1.7;
                                  6, 0.01, 2;
                                  4, 0.05, 1.7;
                                  3, 0.1, 1.5]
        %pitch_vd_pid_gains Pitch-Vd PID
        pitch_vd_pid_gains = [20, 0, 2.3]
        %k_rotor Rotor PD
        k_rotor = [5, 0]
    end
    
    methods
        function obj = GainSchedulingPid()
            obj.elevation_pid = pidAlgorithm(zeros(3,1));
            obj.travel_pitch_pid = pidAlgorithm(zeros(3,1));
            obj.pitch_vd_pid = pidAlgorithm(zeros(3,1));

            obj.front_rotor_pid = pidAlgorithm(zeros(3,1));
            obj.back_rotor_pid = pidAlgorithm(zeros(3,1));
            
            obj.trajectory = Trajectory([], [], [], [], [], []);
        end

        function initialize(obj, trajectory)
            obj.trajectory = trajectory;
            
            obj.elevation_pid.gains = obj.elevation_pid_gains;
            obj.travel_pitch_pid.gains = obj.travel_pitch_pid_gains;
            obj.pitch_vd_pid.gains = obj.pitch_vd_pid_gains;

            obj.front_rotor_pid.gains = [obj.k_rotor(1), 0, obj.k_rotor(2)];
            obj.back_rotor_pid.gains = [obj.k_rotor(1), 0, obj.k_rotor(2)];
        end
        
        function u = control(obj, t, x)
            obj.retune(x(2))
            
            traj_eval = obj.trajectory.eval(t);
            phi_traj = traj_eval.phi;
            eps_traj = traj_eval.eps;
            lamb_traj = traj_eval.lamb;
            
            
            eps_error = x(2) - eps_traj(1);
            if numel(eps_traj) >= 2
                deps_error = x(5) - eps_traj(2);
            else
                deps_error = x(5);
            end
            delta_ws_d = - obj.elevation_pid.compute(t, eps_error, deps_error);

            % outer loop: travel --> pitch
            lamb_error = x(3) - lamb_traj(1);
            if numel(lamb_traj) >= 2
                dlamb_error = x(6) - lamb_traj(2);
            else
                dlamb_error = x(6);
            end
            phi_op = - obj.travel_pitch_pid.compute(t, lamb_error, dlamb_error);

            % inner loop: pitch --> Vd
            phi_error = x(1) - phi_op;
            if numel(phi_traj) >= 2
                dphi_error = x(4) - phi_traj(2);
            else
                dphi_error = x(4);
            end
            delta_wd_d = - obj.pitch_vd_pid.compute(t, phi_error, dphi_error);

            delta_wf_d = (delta_ws_d + delta_wd_d) / 2;
            delta_wb_d = (delta_ws_d - delta_wd_d) / 2;

            wf_d = traj_eval.vf(1) + delta_wf_d;
            wb_d = traj_eval.vb(1) + delta_wb_d;

            Vf = wf_d - obj.front_rotor_pid.compute_fd(t, x(7) - wf_d);
            Vb = wb_d - obj.back_rotor_pid.compute_fd(t, x(8) - wb_d);

            u = [Vf; Vb];
        end
        
        function retune(obj, eps)
            deg = pi / 180;
            if eps < obj.elevation_levels(1) * deg
                eps = obj.elevation_levels(1) * deg;
            elseif eps > obj.elevation_levels(end) * deg
                eps = obj.elevation_levels(end) * deg;
            end

            gains = interp1(obj.elevation_levels*deg, obj.travel_pitch_pid_gains, eps);
            obj.travel_pitch_pid.gains = gains;
        end
    end
end

