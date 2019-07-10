classdef CascadePid < HeliController
    %CascadePid Linear PID controllers in cascade structure.
    properties (Access = private)
        elevation_pid
        travel_pitch_pid
        pitch_vd_pid
        
        front_rotor_pid
        back_rotor_pid

        c
    end
    
    properties
        %elevation_pid_gains Elevation PID
        elevation_pid_gains = [30, 20, 20]
        %travel_pitch_pid_gains Travel-Pitch PID
        travel_pitch_pid_gains = [0.8, 0.005, 1.3]
        %pitch_vd_pid_gains Pitch-Vd PID
        pitch_vd_pid_gains = [8, 0.5, 3.5]
        %k_rotor Rotor PD
        k_rotor = [0, 0]
    end
    
    properties (Nontunable, Logical)
        %feed_forward Use feed forward
        feed_forward = false
        %rotor_speed_control Rotor speed controller
        rotor_speed_control = false
    end
    
    methods
        function obj = CascadePid()
            obj.elevation_pid = pidAlgorithm(obj.elevation_pid_gains);
            obj.travel_pitch_pid = pidAlgorithm(obj.travel_pitch_pid_gains);
            obj.pitch_vd_pid = pidAlgorithm(obj.pitch_vd_pid_gains);

            obj.front_rotor_pid = pidAlgorithm([obj.k_rotor(1), 0, obj.k_rotor(2)]);
            obj.back_rotor_pid = pidAlgorithm([obj.k_rotor(1), 0, obj.k_rotor(2)]);

            obj.c = Constants();
        end

        function initialize(obj)
        end
        
        function u = control(obj, t, x)
            obj.elevation_pid.gains = obj.elevation_pid_gains;
            obj.travel_pitch_pid.gains = obj.travel_pitch_pid_gains;
            obj.pitch_vd_pid.gains = obj.pitch_vd_pid_gains;
            
            obj.front_rotor_pid.gains = [obj.k_rotor(1), 0, obj.k_rotor(2)];
            obj.back_rotor_pid.gains = [obj.k_rotor(1), 0, obj.k_rotor(2)];
            
            traj_eval = eval_trajectory(obj.trajectory, t);
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
            
            if obj.feed_forward
                Fs_ff = obj.Fr(traj_eval.vf(1)) + obj.Fr(traj_eval.vb(1));
                Fd_ff = obj.Fr(traj_eval.vf(1)) - obj.Fr(traj_eval.vb(1));
            else
                Fs_ff = 0;
                Fd_ff = 0;
            end
            
            Fs_d = Fs_ff + delta_Fs_d;
            Fd_d = Fd_ff + delta_Fd_d;
            
            Ff_d = (Fs_d + Fd_d) / 2;
            Fb_d = (Fs_d - Fd_d) / 2;
            
            if obj.rotor_speed_control
                wf_d = obj.Fr_inv(Ff_d);
                wb_d = obj.Fr_inv(Fb_d);
                
                uf = wf_d - obj.front_rotor_pid.compute_fd(t, x(7) - wf_d);
                ub = wb_d - obj.back_rotor_pid.compute_fd(t, x(8) - wb_d);
            else
                uf = obj.Fr_inv(Ff_d);
                ub = obj.Fr_inv(Fb_d);
            end

            u = [uf; ub];
        end
        
        function F = Fr(obj, w)
            if w <= -2 * obj.c.q2 / obj.c.p2
                F = obj.c.p2*w + obj.c.q2;
            elseif w <= 0
                F = - obj.c.p2^2/(4*obj.c.q2) * w^2;
            elseif w <= 2 * obj.c.q1/obj.c.p1
                F = obj.c.p1^2/(4*obj.c.q1) * w^2;
            else
                F = obj.c.p1 * w - obj.c.q1;
            end
        end
        
        function w = Fr_inv(obj, F)
            if F <= - obj.c.q2
                w = (F - obj.c.q2) / obj.c.p2;
            elseif F < 0
                w = - sqrt(-4*obj.c.q2*F) / obj.c.p2;
            elseif F < obj.c.q1
                w = sqrt(4*obj.c.q1*F) / obj.c.p1;
            else
                w = (F + obj.c.q1) / obj.c.p1;
            end
        end
    end
end

