from enum import Enum
import ModelConstants as mc
from numpy.ma import cos, sin
import numpy as np


class ModelType(Enum):
    EASY = 1
    FRICTION = 2
    CENTRIPETAL = 3


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


def compute_linear_ss_and_op_output(model_type: ModelType, e_op):
    """
    Computes the A and B matrices of the linear state space model at the specified operating point for all control
    algorithms that require it. Also returns the front and back rotor voltages to stay in equilibrium.

    :param model_type: the type that should be linearized
    :param e_op: elevation angle (rad) of the desired operating point
    :return: A, B, Vf_op, Vb_op
    """
    L1 = mc.l_p
    L2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
    L3 = mc.l_h
    L4 = mc.l_h

    Jp = 2 * mc.m_p * mc.l_p ** 2
    Je = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * mc.l_h ** 2
    Jl = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * (mc.l_h ** 2 + mc.l_p ** 2)

    Vs_op = - L2 / L3 * cos(e_op)
    Vd_op = 0

    Vf_op = (Vs_op + Vd_op) / 2
    Vb_op = (Vs_op - Vd_op) / 2

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

    return A, B, Vf_op, Vb_op
