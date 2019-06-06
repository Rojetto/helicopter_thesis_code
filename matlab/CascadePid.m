classdef CascadePid < HeliController
    %CascadePid Linear PID controllers in cascade structure.
    properties (Access = private)
        elevation_pid
        travel_pitch_pid
        pitch_vd_pid
        
        front_rotor_pid
        back_rotor_pid
        
        trajectory

        c
    end
    
    properties (Nontunable)
        %elevation_pid_gains Elevation PID
        elevation_pid_gains = [10, 4, 5]
        %travel_pitch_pid_gains Travel-Pitch PID
        travel_pitch_pid_gains = [6, 0.1, 1.5]
        %pitch_vd_pid_gains Pitch-Vd PID
        pitch_vd_pid_gains = [20, 0, 2.3]
        %k_rotor Rotor PD
        k_rotor = [5, 0]
    end
    
    methods
        function obj = CascadePid()
            obj.elevation_pid = pidAlgorithm(zeros(3,1));
            obj.travel_pitch_pid = pidAlgorithm(zeros(3,1));
            obj.pitch_vd_pid = pidAlgorithm(zeros(3,1));
            
            obj.front_rotor_pid = pidAlgorithm(zeros(3,1));
            obj.back_rotor_pid = pidAlgorithm(zeros(3,1));
            
            obj.trajectory = Trajectory([], [], [], [], [], []);

            obj.c = Constants();
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
            delta_Fs_d = - obj.elevation_pid.compute(t, eps_error, deps_error);

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
            delta_Fd_d = - obj.pitch_vd_pid.compute(t, phi_error, dphi_error);
            
            Fs_ff = Fr(traj_eval.vf(1)) + Fr(traj_eval.vb(1));
            Fd_ff = Fr(traj_eval.vf(1)) - Fr(traj_eval.vb(1));
            
            Fs_d = Fs_ff + delta_Fs_d;
            Fd_d = Fd_ff + delta_Fd_d;
            
            Ff_d = (Fs_d + Fd_d) / 2;
            Fb_d = (Fs_d - Fd_d) / 2;

            wf_d = Fr_inv(Ff_d);
            wb_d = Fr_inv(Fb_d);
            
            extra_ws = evalin('base', 'extra');
            extra_ws(end+1,:) = [x(7), wf_d, x(8), wb_d];
            assignin('base', 'extra', extra_ws)
            

            uf = wf_d - obj.front_rotor_pid.compute_fd(t, x(7) - wf_d);
            ub = wb_d - obj.back_rotor_pid.compute_fd(t, x(8) - wb_d);

            u = [uf; ub];

            function F = Fr(w)
                if w <= -2 * obj.c.q2 / obj.c.p2
                    F = obj.c.p2*w + obj.c.q2;
                elseif -2 * obj.c.q2 / obj.c.p2 < w && w <= 0
                    F = - obj.c.p2^2/(4*obj.c.q2) * w^2;
                elseif 0 < w && w <= 2 * obj.c.q1/obj.c.p1
                    F = obj.c.p1^2/(4*obj.c.q1) * w^2;
                else
                    F = obj.c.p1 * w - obj.c.q1;
                end
            end
            
            function w = Fr_inv(F)
                if F <= - obj.c.q2
                    w = (F - obj.c.q2) / obj.c.p2;
                elseif -obj.c.q2 <= F && F < 0
                    w = sqrt(-4*obj.c.q2*F) / obj.c.p2;
                elseif 0 <= F && F < obj.c.q1
                    w = sqrt(4*obj.c.q1*F) / obj.c.p1;
                else
                    w = (F + obj.c.q1) / obj.c.p1;
                end
            end
        end
    end
end

