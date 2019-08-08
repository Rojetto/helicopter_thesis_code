classdef CascadePid < HeliController
    %CascadePid Linear PID controllers in cascade structure.
    properties (Access = private)
        elevation_pid
        travel_pitch_pid
        pitch_vd_pid

        c
    end
    
    properties
        %elevation_pid_gains Elevation PID
        elevation_pid_gains = [15, 0.1, 7]
        %travel_pitch_pid_gains Travel-Pitch PID
        travel_pitch_pid_gains = [1.75, 0.05, 2.25]
        %pitch_vd_pid_gains Pitch-Vd PID
        pitch_vd_pid_gains = [4, 0.1, 2]
    end
    
    properties (Nontunable, Logical)
        %feed_forward Use feed forward
        feed_forward = false
    end
    
    methods
        function obj = CascadePid()
            obj.elevation_pid = pidAlgorithm(obj.elevation_pid_gains);
            obj.travel_pitch_pid = pidAlgorithm(obj.travel_pitch_pid_gains);
            obj.pitch_vd_pid = pidAlgorithm(obj.pitch_vd_pid_gains);

            obj.c = Constants();
        end

        function initialize(obj)
            obj.elevation_pid.reset();
            obj.travel_pitch_pid.reset();
            obj.pitch_vd_pid.reset();
        end
        
        function [u, debug_out] = control(obj, t, x)
            obj.elevation_pid.gains = obj.elevation_pid_gains;
            obj.travel_pitch_pid.gains = obj.travel_pitch_pid_gains;
            obj.pitch_vd_pid.gains = obj.pitch_vd_pid_gains;
            
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
            debug_out = obj.travel_pitch_pid.ix;

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

            uf = obj.Fr_inv(Ff_d);
            ub = obj.Fr_inv(Fb_d);

            u = [uf; ub];
        end
        
        function F = Fr(obj, w)            
            F = Fr(w, obj.c.p1, obj.c.q1, obj.c.p2, obj.c.q2);
        end
        
        function w = Fr_inv(obj, F)
            w = Fr_inv(F, obj.c.p1, obj.c.q1, obj.c.p2, obj.c.q2);
        end
    end
    
    methods (Access = protected)
        function out = getDebugOutputSize(~)
            out = 1;
        end
    end
end

