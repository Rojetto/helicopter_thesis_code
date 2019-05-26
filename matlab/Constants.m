classdef Constants
    properties
        g = 9.81
        l_h = 0.65
        l_p = 0.18
        l_c = 0.52
        m_c = 0.9
        m_p = 0.5 * 0.771

        mup = 0.048
        mue = 0.053
        mul = 0.274

        T_f = 0.1
        T_b = 0.1
        K_f = 1
        K_b = 1

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
            obj.m_m = 0.2 * obj.m_p;
            obj.r_m = obj.l_p / 8;
            obj.J_m = 0.5 * obj.m_m * obj.r_m^2;

            obj.L1 = obj.l_p;
            obj.L2 = obj.g * (obj.l_c * obj.m_c - 2 * obj.l_h * obj.m_p);
            obj.L3 = obj.l_h;
            obj.L4 = obj.l_h;

            obj.Jp = 2 * obj.m_p * obj.l_p^2;
            obj.Je = obj.m_c * obj.l_c^2 + 2 * obj.m_p * obj.l_h^2;
            obj.Jl = obj.m_c * obj.l_c^2 + 2 * obj.m_p * (obj.l_h^2 + obj.l_p^2);
        end
    end
end

