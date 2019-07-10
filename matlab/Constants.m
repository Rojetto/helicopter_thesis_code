classdef Constants
    properties
        g = 9.81
        l_h = 0.67
        l_p = 0.178
        l_c = 0.4069
        
        d_h = 0.0027
        d_c = 0.0639        
        
        m_c = 1.7638
        m_h = 1.2006

        mup = 0.0334
        mue = 0.0755
        mul = 0.2569

        T_w = 0.01
        K_w = 1
        
        p1 = 0.3117
        q1 = 0.4623
        p2 = 0.1396
        q2 = 0.3819

        m_m
        r_m
        J_m
        K_m = 0.1
        
        L1
        L2
        L3
        L4
        
        Jp
        Je
        Jl
    end
    
    methods
        function obj = Constants()
            obj.m_m = 0.2 * obj.m_h;
            obj.r_m = obj.l_p / 8;
            obj.J_m = 0.5 * obj.m_m * obj.r_m^2;

            obj.L1 = obj.l_p;
            obj.L2 = obj.g * (obj.l_c * obj.m_c - obj.l_h * obj.m_h);
            obj.L3 = obj.l_h;
            obj.L4 = obj.l_h;

            obj.Jp = obj.m_h * obj.l_p^2;
            obj.Je = obj.m_c * obj.l_c^2 + obj.m_h * obj.l_h^2;
            obj.Jl = obj.m_c * obj.l_c^2 + obj.m_h * (obj.l_h^2 + obj.l_p^2);
        end
    end
end

