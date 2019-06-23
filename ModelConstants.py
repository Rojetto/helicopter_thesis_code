from enum import Enum
import random


class OrderedEnum(Enum):
    def __ge__(self, other):
        if self.__class__ is other.__class__:
            return self.value >= other.value
        return NotImplemented
    def __gt__(self, other):
        if self.__class__ is other.__class__:
            return self.value > other.value
        return NotImplemented
    def __le__(self, other):
        if self.__class__ is other.__class__:
            return self.value <= other.value
        return NotImplemented
    def __lt__(self, other):
        if self.__class__ is other.__class__:
            return self.value < other.value
        return NotImplemented


class ModelType(OrderedEnum):
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
    d_c = 0.2
    d_1 = 0.05
    d_2 = 0.03
    m_p = 0.5 * 0.771  # kg
    m_c_max = 2 * l_h/l_c * m_p
    m_c = 0.93 * m_c_max  # arbitrary, but has to be smaller than m_c_max

    # motor characteristics
    m_m = 0.2 * m_p
    r_m = l_p/8
    J_m = 0.5 * m_m * r_m**2

    p1 = 1
    q1 = 0.5
    p2 = 0.6
    q2 = 0.3

    d_p = 0.048  # dissipation coefficient in Nms
    d_e = 0.053
    d_l = 0.274

    # PT1 charakteristics for ROTOR SPEED
    T_w = 0.1
    K_w = 1
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
    m_c = m_c_max*random.uniform(0.9, 0.96)  # arbitrary, but has to be smaller than m_c_max

    # motor characteristics
    m_m = 0.2 * m_p
    r_m = l_p / 8
    J_m = 0.5 * m_m * r_m ** 2

    p1 = OriginalConstants.p1 * random.uniform(0.9, 1.1)
    q1 = OriginalConstants.q1 * random.uniform(0.9, 1.1)
    p2 = OriginalConstants.p2 * random.uniform(0.9, 1.1)
    q2 = OriginalConstants.q2 * random.uniform(0.9, 1.1)

    d_p = OriginalConstants.d_p * random.uniform(0.9, 1.1)
    d_e = OriginalConstants.d_e * random.uniform(0.9, 1.1)
    d_l = OriginalConstants.d_l * random.uniform(0.9, 1.1)

    # PT1 charakteristics for ROTOR SPEED
    T_w = OriginalConstants.T_w * random.uniform(0.9, 1.1)
    K_w = OriginalConstants.K_w * random.uniform(0.9, 1.1)
    K = OriginalConstants.K * random.uniform(0.9, 1.1)
    K_m = OriginalConstants.K_m * random.uniform(0.9, 1.1)

    g = OriginalConstants.g


class LowerBoundConstants:
    # l_a = ? (not given in Fig. 7)
    l_p = OriginalConstants.l_p * 0.9
    l_h = OriginalConstants.l_h * 0.9
    l_c = OriginalConstants.l_c * 0.9
    m_p = OriginalConstants.m_p * 0.9
    m_c_max = 2 * l_h / l_c * m_p
    m_c = m_c_max*0.9  # arbitrary, but has to be smaller than m_c_max

    # motor characteristics
    m_m = 0.2 * m_p
    r_m = l_p / 8
    J_m = 0.5 * m_m * r_m ** 2

    p1 = OriginalConstants.p1 * 0.9
    q1 = OriginalConstants.q1 * 0.9
    p2 = OriginalConstants.p2 * 0.9
    q2 = OriginalConstants.q2 * 0.9

    d_p = OriginalConstants.d_p * 0.9
    d_e = OriginalConstants.d_e * 0.9
    d_l = OriginalConstants.d_l * 0.9

    # PT1 charakteristics for ROTOR SPEED
    T_w = OriginalConstants.T_w * 0.9
    K_w = OriginalConstants.K_w * 0.9
    K = OriginalConstants.K * 0.9
    K_m = OriginalConstants.K_m * 0.9

    g = OriginalConstants.g


class UpperBoundConstants:
    # l_a = ? (not given in Fig. 7)
    l_p = OriginalConstants.l_p * 1.1
    l_h = OriginalConstants.l_h * 1.1
    l_c = OriginalConstants.l_c * 1.1
    m_p = OriginalConstants.m_p * 1.1
    m_c_max = 2 * l_h / l_c * m_p
    m_c = m_c_max*0.96  # arbitrary, but has to be smaller than m_c_max

    # motor characteristics
    m_m = 0.2 * m_p
    r_m = l_p / 8
    J_m = 0.5 * m_m * r_m ** 2

    p1 = OriginalConstants.p1 * 1.1
    q1 = OriginalConstants.q1 * 1.1
    p2 = OriginalConstants.p2 * 1.1
    q2 = OriginalConstants.q2 * 1.1

    d_p = OriginalConstants.d_p * 1.1
    d_e = OriginalConstants.d_e * 1.1
    d_l = OriginalConstants.d_l * 1.1

    # PT1 charakteristics for ROTOR SPEED
    T_w = OriginalConstants.T_w * 1.1
    K_w = OriginalConstants.K_w * 1.1
    K = OriginalConstants.K * 1.1
    K_m = OriginalConstants.K_m * 1.1

    g = OriginalConstants.g
