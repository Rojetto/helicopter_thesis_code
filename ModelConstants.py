from enum import Enum
import random


class ModelType(Enum):
    EASY = 1
    FRICTION = 2
    CENTRIPETAL = 3
    ROTORSPEED = 4
    GYROMOMENT = 5
    NO_SIMPLIFICATIONS = 6


class OriginalConstants:
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


class RandomizedConstants:
    random.seed(1234)
    # l_a = ? (not given in Fig. 7)
    l_p = OriginalConstants.l_p * random.uniform(0.9, 1.1)
    l_h = OriginalConstants.l_h * random.uniform(0.9, 1.1)
    l_c = OriginalConstants.l_c * random.uniform(0.9, 1.1)
    m_p = OriginalConstants.m_p * random.uniform(0.9, 1.1)
    m_c_max = 2 * l_h / l_c * m_p
    m_c = m_c_max*random.uniform(0.7, 0.9)  # arbitrary, but has to be smaller than m_c_max

    # motor characteristics
    m_m = 0.2 * m_p
    r_m = l_p / 8
    J_m = 0.5 * m_m * r_m ** 2

    d_p = OriginalConstants.d_p * random.uniform(0.9, 1.1)
    d_e = OriginalConstants.d_e * random.uniform(0.9, 1.1)
    d_l = OriginalConstants.d_l * random.uniform(0.9, 1.1)

    # PT1 charakteristics for ROTOR SPEED
    T_f = OriginalConstants.T_f * random.uniform(0.9, 1.1)
    T_b = OriginalConstants.T_b * random.uniform(0.9, 1.1)
    K_f = OriginalConstants.K_f * random.uniform(0.9, 1.1)
    K_b = OriginalConstants.K_b * random.uniform(0.9, 1.1)
    K = OriginalConstants.K * random.uniform(0.9, 1.1)
    K_m = OriginalConstants.K_m * random.uniform(0.9, 1.1)

    g = OriginalConstants.g
