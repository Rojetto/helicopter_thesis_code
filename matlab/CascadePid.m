classdef CascadePid < HeliController
    %CascadePid Linear PID controllers in cascade structure.
    properties (Access = private)
        elevation_pid
        travel_pitch_pid
        pitch_vd_pid
        
        trajectory
    end
    
    properties (Nontunable)
        %elevation_pid_gains Elevation PID
        elevation_pid_gains = [10, 2, 5]
        %travel_pitch_pid_gains Travel-Pitch PID
        travel_pitch_pid_gains = [1.5, 0.1, 1.5]
        %pitch_vd_pid_gains Pitch-Vd PID
        pitch_vd_pid_gains = [20, 0, 2.3]
    end
    
    methods
        function obj = CascadePid()
            obj.elevation_pid = pidAlgorithm(zeros(3,1));
            obj.travel_pitch_pid = pidAlgorithm(zeros(3,1));
            obj.pitch_vd_pid = pidAlgorithm(zeros(3,1));
            
            obj.trajectory = Trajectory([], [], [], [], [], []);
        end

        function initialize(obj, trajectory)
            obj.trajectory = trajectory;
            
            obj.elevation_pid.gains = obj.elevation_pid_gains;
            obj.travel_pitch_pid.gains = obj.travel_pitch_pid_gains;
            obj.pitch_vd_pid.gains = obj.pitch_vd_pid_gains;
        end
        
        function u = control(obj, t, x)
            traj_eval = obj.trajectory.eval(t);
            elevation_traj = traj_eval.eps;
            lambda_traj = traj_eval.lamb;
            
            e_error = x(2) - elevation_traj(1);
            delta_ws_d = - obj.elevation_pid.compute(t, e_error, x(5));

            % outer loop: travel --> pitch
            lambda_error = x(3) - lambda_traj(1);
            p_op = - obj.travel_pitch_pid.compute(t, lambda_error, x(6));

            % inner loop: pitch --> Vd
            p_error = x(1) - p_op;
            delta_wd_d = - obj.pitch_vd_pid.compute(t, p_error, x(4));

            delta_wf_d = (delta_ws_d + delta_wd_d) / 2;
            delta_wb_d = (delta_ws_d - delta_wd_d) / 2;

            Vf = traj_eval.vf(1) + delta_wf_d;
            Vb = traj_eval.vb(1) + delta_wb_d;

            u = [Vf; Vb];
        end
    end
end

