from helicontrollers.AbstractController import AbstractController, ParamEnum, ParamFloatArray
from helicontrollers.util import ModelType, compute_feed_forward_flatness
import control as ctr
import numpy as np

import ModelConstants as mc
from numpy.ma import cos, sin

L1 = mc.l_p
L2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
L3 = mc.l_h
L4 = mc.l_h

Jp = 2 * mc.m_p * mc.l_p ** 2
Je = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * mc.l_h ** 2
Jl = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * (mc.l_h ** 2 + mc.l_p ** 2)


def getLinearizedMatrices(model_type: ModelType, operating_point, Vf_op, Vb_op):
    """Computes the matrices of the linearized model at the given operating point.

    :param model_type: the type of model that should be linearized
    :param operating_point: list of state variables (p, e, lambda, dp, de, dlambda)
    :param Vf_op: value of Vf for linearization
    :param Vb_op: value of Vb for linearization"""

    p_op, e_op, lamb_op, dp_op, de_op, dlamb_op = operating_point

    # Vf_op, Vb_op = compute_feed_forward_flatness(e_and_derivatives, lambda_and_derivatives)
    Vs_op = Vf_op + Vb_op
    Vd_op = Vf_op - Vb_op

    if model_type == ModelType.EASY:
        A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, 0, 0, 0, 0],
                      [-L3 * Vs_op * sin(p_op) / Je, -L2 * sin(e_op) / Je, 0, 0, 0, 0],
                      [L4 * Vs_op * cos(p_op) * cos(e_op) / Jl, 0, 0, 0, 0, 0]])
    elif model_type == ModelType.FRICTION:
        A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, 0, -mc.d_p / Jp, 0, 0],
                      [-L3 * Vs_op * sin(p_op) / Je, -L2 * sin(e_op) / Je, 0, 0, -mc.d_e / Je, 0],
                      [L4 * Vs_op * cos(p_op) * cos(e_op) / Jl, -L4 * Vs_op * sin(p_op) * sin(e_op) / Jl, 0, 0, 0, -mc.d_l / Jl]])
    elif model_type == ModelType.CENTRIPETAL:
        A = np.array([[0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1],
                      [-(de_op ** 2 - dlamb_op ** 2 * cos(e_op) ** 2) * sin(p_op) ** 2 + (de_op ** 2 - dlamb_op ** 2 * cos(e_op) ** 2) * cos(p_op) ** 2, 2 * dlamb_op ** 2 * sin(p_op) * sin(e_op) * cos(p_op) * cos(e_op), 0, -mc.d_p / Jp, 2 * de_op * sin(p_op) * cos(p_op), -2 * dlamb_op * sin(p_op) * cos(p_op) * cos(e_op) ** 2],
                      [-L3 * Vs_op * sin(p_op) / Je, dlamb_op ** 2 * sin(e_op) ** 2 - dlamb_op ** 2 * cos(e_op) ** 2 - L2 * sin(e_op) / Je, 0, 0, -mc.d_e / Je, -2 * dlamb_op * sin(e_op) * cos(e_op)],
                      [L4 * Vs_op * cos(p_op) * cos(e_op) / Jl, -L4 * Vs_op * sin(p_op) * sin(e_op) / Jl, 0, 0, 0, -mc.d_l / Jl]])

    B = np.array([[0, 0],
                  [0, 0],
                  [0, 0],
                  [L1 / Jp, -L1 / Jp],
                  [L3 / Je * cos(p_op), L3 / Je * cos(p_op)],
                  [L4 * sin(p_op) * cos(e_op) / Jl, L4 * sin(p_op) * cos(e_op) / Jl]])

    return A, B, Vf_op, Vb_op


def get_p_and_first_derivative(model_type: ModelType, e_and_derivatives, lambda_and_derivatives):
    "Computes p and dp from the flat output and its derivatives."
    if model_type != ModelType.EASY:
        print("model_type = " + str(model_type))
        raise NotImplementedError("get_p_and_first_derivative is not implemented for other model types than EASY.")

    p = np.arctan((Jl * L3 * lambda_and_derivatives[2]) /
                  (L4 * cos(e_and_derivatives[0]) *(Je * e_and_derivatives[2] - L2 * cos(e_and_derivatives[0]))))
    # the following part is partly copied from compute_feed_forward_flatness
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

    dp = (1/(1+a*a)) * da1
    return p, dp

def get_current_operating_point(model_type: ModelType, e_and_derivatives, lambda_and_derivatives):
    """Wrapper function for getLinearizedMatrices. Computes the linearized matrices from the current flat output
    and its derivative.
    :param model_type: the type of model that should be linearized
    :param e_and_derivatives: planned trajectory for e
    :param lambda_and_derivatives: planned trajectory for lambda
    :return operating point vector, Vf_op, Vb_op"""
    e_op = e_and_derivatives[0]
    de_op = e_and_derivatives[1]
    lamb_op = lambda_and_derivatives[0]
    dlamb_op = lambda_and_derivatives[1]
    Vf_op, Vb_op = compute_feed_forward_flatness(e_and_derivatives, lambda_and_derivatives)
    p_op, dp_op = get_p_and_first_derivative(model_type, e_and_derivatives, lambda_and_derivatives)
    operating_point = [p_op, e_op, lamb_op, dp_op, de_op, dlamb_op]
    return operating_point, Vf_op, Vb_op

class TimeVariantController(AbstractController):
    def __init__(self):
        self.operating_point = [0, 0]  # travel, elevation
        self.model_type = ModelType.CENTRIPETAL
        self.poles = [-1, -2, -3, -4, -5, -6]
        self.Vf_op = 0
        self.Vb_op = 0
        self.K = np.zeros((2, 6))
        self.time_variant_feedback = True
        self.feedback_computed = False

        super().__init__("TimeVariantController", {
            "Model type": ParamEnum(["Simple", "Friction", "Centripetal"],
                                    [ModelType.EASY, ModelType.FRICTION, ModelType.CENTRIPETAL],
                                    self.model_type),
            "Poles": ParamFloatArray([-100, -100, -100, -100, -100, -100],
                                     [-0.01, -0.01, -0.01, -0.01, -0.01, -0.01],
                                     self.poles)
        })

    def control(self, t, x, e_traj, lambda_traj):
        # p, e, lamb, dp, de, dlamb = x
        # u_op = np.array([self.Vf_op, self.Vb_op])
        # x_op = np.array([0, self.operating_point[1], self.operating_point[0], 0, 0, 0])

        operating_point, Vf_op, Vb_op = get_current_operating_point(self.model_type, e_traj, lambda_traj)
        A, B, Vf_op, Vb_op = getLinearizedMatrices(self.model_type, operating_point, Vf_op, Vb_op)

        linearized_state = x - operating_point

        # compute K for this time step
        try:
            K_now = ctr.place(A, B, self.poles)
        except Exception as e:
            print("Error during pole placement: " + str(e))

        u = - K_now @ linearized_state
        return u

    def initialize(self, param_value_dict):
        self.model_type = param_value_dict["Model type"]
        self.poles = param_value_dict["Poles"]
        self.feedback_computed = False
