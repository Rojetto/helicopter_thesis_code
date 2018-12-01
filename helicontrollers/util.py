from enum import Enum
import ModelConstants as mc
from numpy.ma import cos, sin, arctan
import numpy as np

L1 = mc.l_p
L2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
L3 = mc.l_h
L4 = mc.l_h

Jp = 2 * mc.m_p * mc.l_p ** 2
Je = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * mc.l_h ** 2
Jl = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * (mc.l_h ** 2 + mc.l_p ** 2)


class ModelType(Enum):
    EASY = 1
    FRICTION = 2
    CENTRIPETAL = 3
    ROTORSPEED = 4


class FeedForwardMethod(Enum):
    NONE = 1
    STATIC = 2
    FLATNESS = 3


class PidAlgorithm:
    def __init__(self, gains):
        self.gains = gains
        self.ix = 0.0
        self.last_t = 0.0

    def compute(self, t, x, dx):
        dt = t - self.last_t
        self.ix += dt * x

        out = self.gains[0] * x + self.gains[1] * self.ix + self.gains[2] * dx

        self.last_t = t

        return out


def compute_linear_ss(model_type: ModelType, e_op):
    """
    Computes the A and B matrices of the linear state space model at the specified operating point for all control
    algorithms that require it.

    :param model_type: the type that should be linearized
    :param e_op: elevation angle (rad) of the desired operating point
    :return: A, B
    """
    Vf_op, Vb_op = compute_feed_forward_static([e_op, 0, 0, 0, 0], [0, 0, 0, 0, 0])
    Vs_op = Vf_op + Vb_op

    if model_type == ModelType.EASY:
        A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, 0, 0, 0, 0],
                      [0, -L2 * sin(e_op) / Je, 0, 0, 0, 0],
                      [L4 * Vs_op * cos(e_op) / Jl, 0, 0, 0, 0, 0]])

    if model_type == ModelType.FRICTION or model_type == ModelType.CENTRIPETAL:
        A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, 0, - mc.d_p / Jp, 0, 0],
                      [0, -L2 * sin(e_op) / Je, 0, 0, - mc.d_e / Je, 0],
                      [L4 * Vs_op * cos(e_op) / Jl, 0, 0, 0, 0, - mc.d_l / Jl]])

    B = np.array([[0, 0],
                  [0, 0],
                  [0, 0],
                  [L1 / Jp, -L1 / Jp],
                  [L3 / Je, L3 / Je],
                  [0, 0]])

    return A, B


def compute_feed_forward_static(e_and_derivatives, lambda_and_derivatives):
    e = e_and_derivatives[0]
    Vs = - L2 / L3 * cos(e)
    Vd = 0

    Vf = (Vs + Vd) / 2
    Vb = (Vs - Vd) / 2

    return Vf, Vb


def compute_feed_forward_flatness_simple(e_and_derivatives, lambda_and_derivatives):
    pitch, system_in = compute_pitch_and_inputs_flatness_simple(e_and_derivatives, lambda_and_derivatives)

    return system_in


def compute_pitch_flatness_simple(e_and_derivatives, lambda_and_derivatives):
    pitch, system_in = compute_pitch_and_inputs_flatness_simple(e_and_derivatives, lambda_and_derivatives)

    return pitch


def compute_pitch_and_inputs_flatness_simple(e_and_derivatives, lambda_and_derivatives):
    e = e_and_derivatives[0]
    de1 = e_and_derivatives[1]
    de2 = e_and_derivatives[2]
    de3 = e_and_derivatives[3]
    de4 = e_and_derivatives[4]

    l = lambda_and_derivatives[0]
    dl1 = lambda_and_derivatives[1]
    dl2 = lambda_and_derivatives[2]
    dl3 = lambda_and_derivatives[3]
    dl4 = lambda_and_derivatives[4]

    b = L3 * Jl * dl2
    c = L4 * cos(e)
    d = Je * de2 - L2 * cos(e)
    a = b * c / d

    db1 = L3 * Jl * dl3
    db2 = L3 * Jl * dl4
    dc1 = - L4 * sin(e) * de1
    dc2 = - L4 * (cos(e) * de1 ** 2 + sin(e) * de2)
    dd1 = Je * de3 + L2 * sin(e) * de1
    dd2 = Je * de4 + L2 * (cos(e) * de1 ** 2 + sin(e) * de2)
    f = db1 * c * d
    g = dc1 * d + c * dd1
    h = c * c * d * d
    da1 = (f - b * g) / h

    df1 = db2 * c * d + db1 * g
    dg1 = dc2 * d + 2 * dc1 * dd1 + c * dd2
    dh1 = 2 * c * dc1 * d ** 2 + 2 * c ** 2 * d * dd1
    da2 = ((df1 - (db1 * g + b * dg1)) * h - (f - b * g) * dh1) / h ** 2

    p = arctan(a)
    dp1 = da1 / (1 + a**2)
    dp2 = (da2 * (1 + a ** 2) - 2 * a * da1 ** 2) / (1 + a ** 2) ** 2

    Vs = ((Jl * dl2 / (L4 * cos(e))) ** 2 + ((Je * de2 - L2 * cos(e)) / L3) ** 2) ** (1 / 2)
    Vd = Jp * dp2 / L1

    Vf = (Vs + Vd) / 2
    Vb = (Vs - Vd) / 2

    return np.array([p, dp1, dp2]), np.array([Vf, Vb])
