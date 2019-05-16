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

