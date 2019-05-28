from enum import Enum


class ModelType(Enum):
    EASY = 1
    FRICTION = 2
    CENTRIPETAL = 3
    ROTORSPEED = 4
    GYROMOMENT = 5
    NO_SIMPLIFICATIONS = 6


# l_a = ? (not given in Fig. 7)
l_p = 0.18  # m
l_h = 0.65
l_c = 0.52
m_p = 0.5 * 0.771  # kg
m_c_max = 2 * l_h/l_c * m_p
m_c = 0.9  # arbitrary, but has to be smaller than m_c_max

# motor characteristics
m_m = 0.2 * m_p
r_m = l_p/8
J_m = 0.5 * m_m * r_m**2

d_p = 0.048  # dissipation coefficient in Nms
d_e = 0.053
d_l = 0.274

# PT1 charakteristics for ROTOR SPEED
T_f = 0.1
T_b = 0.1
K_f = 1
K_b = 1
K = 1
K_m = 0.1

g = 9.81  # m/s^2
