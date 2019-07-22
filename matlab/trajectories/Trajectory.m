classdef Trajectory
    properties
        t
        phi
        eps
        lamb
        vf
        vb
    end
    
    methods
        function obj = Trajectory(t, phi, eps, lamb, vf, vb)
            obj.t = t;
            obj.phi = phi;
            obj.eps = eps;
            obj.lamb = lamb;
            obj.vf = vf;
            obj.vb = vb;
        end
        
        function out = eval(obj, t)
            if t <= obj.t(1)
                out.phi = obj.phi(1,:);
                out.eps = obj.eps(1,:);
                out.lamb = obj.lamb(1,:);
                out.vf = obj.vf(1,:);
                out.vb = obj.vb(1,:);
            elseif t > obj.t(end)
                out.phi = obj.phi(end,:);
                out.eps = obj.eps(end,:);
                out.lamb = obj.lamb(end,:);
                out.vf = obj.vf(end,:);
                out.vb = obj.vb(end,:);
            else
                out.phi = interp1(obj.t, obj.phi, t);
                out.eps = interp1(obj.t, obj.eps, t);
                out.lamb = interp1(obj.t, obj.lamb, t);
                out.vf = interp1(obj.t, obj.vf, t);
                out.vb = interp1(obj.t, obj.vb, t);
            end
        end
    end
end