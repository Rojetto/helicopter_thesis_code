classdef FeedbackLinearization < HeliController
    %CascadePid Linear PID controllers in cascade structure.
    properties (Access = private)
        front_rotor_pid
        back_rotor_pid
        c
        
        trajectory
    end

    properties (Nontunable, Logical)
        %friction_centripetal Include friction and centripetal forces
        friction_centripetal = true
        %rotor_speed_controller Rotor speed controller
        rotor_speed_controller = true
    end

    properties (Nontunable)
        %ke k Elevation
        ke = [20, 6]
        %kl k Travel
        kl = [1, 0.8]
        %kp k Pitch
        kp = [150, 20]
        %k_rotor Rotor PD
        k_rotor = [5, 0]
    end
    
    methods
        function obj = FeedbackLinearization()
            obj.front_rotor_pid = pidAlgorithm(zeros(3,1));
            obj.back_rotor_pid = pidAlgorithm(zeros(3,1));

            obj.c = Constants();
            obj.trajectory = Trajectory([], [], [], [], [], []);
        end

        function initialize(obj, trajectory)
            obj.front_rotor_pid.gains = [obj.k_rotor(1), 0, obj.k_rotor(2)];
            obj.front_rotor_pid.gains = [obj.k_rotor(1), 0, obj.k_rotor(2)];
            
            obj.trajectory = trajectory;
        end
        
        function u = control(obj, t, x)
            p = x(1);
            e = x(2);
            l = x(3);
            dp = x(4);
            de = x(5);
            dl = x(6);
            
            traj_eval = obj.trajectory.eval(t);
            e_traj = traj_eval.eps;
            lambda_traj = traj_eval.lamb;

            if obj.rotor_speed_controller
                wf = x(7);
                wb = x(8);
            end

            v1 = e_traj(3) - obj.ke(2) * (de - e_traj(2)) - obj.ke(1) * (e - e_traj(1));
            v2 = lambda_traj(3) - obj.kl(2) * (dl - lambda_traj(2)) - obj.kl(1) * (l - lambda_traj(1));

            if obj.friction_centripetal
                u1v = 1/obj.c.L3 * (obj.c.Je*v1 - obj.c.L2*cos(e) + obj.c.mue*de + obj.c.Je*cos(e)*sin(e)*dl^2);
                u2v = 1/(obj.c.L4*cos(e))*(obj.c.Jl*v2 + obj.c.mul*dl);
            else
                u1v = 1/obj.c.L3 * (- obj.c.L2 * cos(e) + obj.c.Je * v1);
                u2v = obj.c.Jl / (obj.c.L4 * cos(e)) * v2;
            end

            Vs = sqrt(u1v^2 + u2v^2)*sign(u1v);

            pd = atan(u2v / u1v);

            dpd = 0;
            ddpd = 0;

            v3 = ddpd - obj.kp(2) * (dp - dpd) - obj.kp(1) * (p - pd);

            if obj.friction_centripetal
                Vd = 1/obj.c.L1 * (obj.c.Jp * v3 + obj.c.mup * dp - obj.c.Jp*cos(p)*sin(p)*(de^2-cos(e)^2*dl^2));
            else
                Vd = obj.c.Jp/obj.c.L1 * v3;
            end

            Vf = (Vs + Vd) / 2;
            Vb = (Vs - Vd) / 2;

            if obj.rotor_speed_controller
                Vf = Vf + obj.front_rotor_pid.compute_fd(t, Vf - wf);
                Vb = Vb + obj.back_rotor_pid.compute_fd(t, Vb - wb);
            end

            u = [Vf; Vb];
        end
    end
end

