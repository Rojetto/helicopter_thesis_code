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

#ToDo Unite these calculations for the controller with the other flatness-based formulas


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
    if model_type == ModelType.EASY:
        return get_p_and_first_derivative_simple(model_type, e_and_derivatives, lambda_and_derivatives)
    elif model_type == ModelType.CENTRIPETAL:
        return get_p_and_first_derivative_centripetal(model_type, e_and_derivatives, lambda_and_derivatives)
    else:
        raise NotImplementedError("get_p_and_first_derivative is not implemented for other model types than EASY and CENTRIPETAL.")

def get_p_and_first_derivative_centripetal(model_type: ModelType, e_and_derivatives, lambda_and_derivatives):
    """copied from util.compute_pitch_and_inputs_flatness_centripetal"""
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

    a = Jl * dl2
    b = mc.d_l * dl1
    c = cos(e)
    d = Je * de2
    e_ = mc.d_e * de1
    f = Je * cos(e) * sin(e) * dl1**2
    g = -L2 * cos(e)

    da1 = Jl * dl3
    db1 = mc.d_l * dl2
    dc1 = -sin(e) * de1
    dd1 = Je * de3
    de_1 = mc.d_e * de2
    # f
    k = cos(e) * sin(e)
    l_ = dl1**2
    dk1 = de1 * (cos(e)**2 - sin(e)**2)
    dl_1 = 2 * dl1 * dl2
    df1 = Je * (dk1 * l_ + k * dl_1)
    dg1 = L2 * sin(e) * de1

    da2 = Jl * dl4
    db2 = mc.d_l * dl3
    dc2 = -cos(e) * de1 ** 2 - sin(e) * de2
    dd2 = Je * de4
    de_2 = mc.d_e * de3
    # f
    dk2 = de2 * (cos(e)**2 - sin(e)**2) - 4 * sin(e) * cos(e) * de1 **2
    dl2 = 2 * (dl2**2 + dl1 * dl3)
    df2 = Je * (dk2 * l + 2*dk1 *dl1 + k * dl2)
    dg2 = L2 * (cos(e) * de1**2 + sin(e) * de2)

    h = a+b
    i = d + e_ + f + g
    j = c * i

    dh1 = da1 + db1
    di1 = dd1 + de_1 + df1 + dg1
    dj1 = dc1 * i + c * di1

    dh2 = da2 + db2
    di2 = dd2 + de_2 + df2 + dg2
    dj2 = dc2 * i + 2*dc1*di1 + c * di2

    A =  h / j
    dA1 = (dh1 * j - h * dj1) / (j ** 2)
    dA2 = ((dh2 * j - h * dj2) * j - (dh1 *j -h * dj1) * 2 * dj1) / (j**3)

    x = (L3/L4) * A
    dx1 = (L3/L4) * dA1
    dx2 = (L3/L4) * dA2

    # p = arctan((L3/L4) * A)
    # dp1 = (L3 / L4) * 1/(1+((L3/L4) * A)**2) * dA1
    # dp2 = (L3 / L4) * (dA2 * (1+((L3/L4) * A)**2) - (L3/L4) * dA1**2) / ((1+((L3/L4) * A)**2)**2)
    p = np.arctan(x)
    dp1 = dx1/(1+x**2)
    return p, dp1



def get_p_and_first_derivative_simple(model_type: ModelType, e_and_derivatives, lambda_and_derivatives):
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
    Vf_op, Vb_op = compute_feed_forward_flatness(model_type, e_and_derivatives, lambda_and_derivatives)
    p_op, dp_op = get_p_and_first_derivative(model_type, e_and_derivatives, lambda_and_derivatives)
    operating_point = [p_op, e_op, lamb_op, dp_op, de_op, dlamb_op]
    return operating_point, Vf_op, Vb_op

class TimeVariantController(AbstractController):
    def __init__(self):
        self.operating_point = [0, 0]  # travel, elevation
        self.linearization_model_type = ModelType.EASY
        self.flatness_model_type =  ModelType.EASY
        self.poles = [-1, -2, -3, -4, -5, -6]
        self.Vf_op = 0
        self.Vb_op = 0
        self.K = np.zeros((2, 6))
        self.time_variant_feedback = True
        self.feedback_computed = False

        super().__init__("TimeVariantController", {
            "Linearization Model type": ParamEnum(["Simple", "Friction", "Centripetal", "Rotorspeed"],
                                                  [ModelType.EASY, ModelType.FRICTION, ModelType.CENTRIPETAL, ModelType.ROTORSPEED],
                                                  self.linearization_model_type),
            "Flatness-calculation model type": ParamEnum(["Simple", "Centripetal"],
                                                         [ModelType.EASY, ModelType.CENTRIPETAL],
                                                         self.flatness_model_type),
            "Poles": ParamFloatArray(self.poles)
        })

    def control(self, t, x, e_traj, lambda_traj):
        # delete front and back speed because we dont need it here
        x = x[:6]
        # p, e, lamb, dp, de, dlamb = x
        # u_op = np.array([self.Vf_op, self.Vb_op])
        # x_op = np.array([0, self.operating_point[1], self.operating_point[0], 0, 0, 0])

        operating_point, Vf_op, Vb_op = get_current_operating_point(self.flatness_model_type, e_traj, lambda_traj)
        A, B, Vf_op, Vb_op = getLinearizedMatrices(self.linearization_model_type, operating_point, Vf_op, Vb_op)

        linearized_state = x - operating_point

        # compute K for this time step
        try:
            K_now = ctr.place(A, B, self.poles)
        except Exception as e:
            print("Error during pole placement: " + str(e))

        u = - K_now @ linearized_state
        return u

    def initialize(self, param_value_dict):
        self.linearization_model_type = param_value_dict["Linearization Model type"]
        self.flatness_model_type = param_value_dict["Flatness-calculation model type"]
        self.poles = param_value_dict["Poles"]
        self.feedback_computed = False
