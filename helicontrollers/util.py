import ModelConstants as mc
from ModelConstants import ModelType
from enum import Enum
from numpy.ma import cos, sin, arctan
import numpy as np
import sympy as sp

L1 = mc.l_p
L2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
L3 = mc.l_h
L4 = mc.l_h

Jp = 2 * mc.m_p * mc.l_p ** 2
Je = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * mc.l_h ** 2
Jl = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * (mc.l_h ** 2 + mc.l_p ** 2)


class FeedForwardMethod(Enum):
    NONE = 1
    STATIC = 2
    FLATNESS = 3


class PidAlgorithm:
    def __init__(self, gains):
        self.gains = gains
        self.ix = 0.0
        self.last_t = 0.0
        self.last_x = None

    def compute(self, t, x, dx=None):
        dt = t - self.last_t
        self.ix += dt * x

        if dx is None:
            if self.last_x is None:
                dx = 0
            else:
                dx = (x - self.last_x) / dt

        out = self.gains[0] * x + self.gains[1] * self.ix + self.gains[2] * dx

        self.last_t = t
        self.last_x = x

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
    elif model_type == ModelType.FRICTION or model_type == ModelType.CENTRIPETAL:
        A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, 0, - mc.d_p / Jp, 0, 0],
                      [0, -L2 * sin(e_op) / Je, 0, 0, - mc.d_e / Je, 0],
                      [L4 * Vs_op * cos(e_op) / Jl, 0, 0, 0, 0, - mc.d_l / Jl]])
    else:
        # TODO: Implement for remaining model types
        print("Unsupported model type while trying to linearize system")
        A = np.zeros((6, 6))

    B = np.array([[0, 0],
                  [0, 0],
                  [0, 0],
                  [L1 / Jp, -L1 / Jp],
                  [L3 / Je, L3 / Je],
                  [0, 0]])

    return A, B


def compute_linear_ss_full(x_op, u_op):
    """
    Computes linear state space matrices A and B for the full model (without any simplifications) at operating point
    x_op. This state space model contains all 8 (!) state variables.

    :return: A, B
    """

    p, e, l = sp.symbols("p e \\lambda")
    dp, de, dl = sp.symbols("\\dot{p} \\dot{e} \\dot{\\lambda}")
    wf, wb, Vf, Vb = sp.symbols("\\omega_f \\omega_b V_f V_b")

    x = sp.Matrix([p, e, l, dp, de, dl, wf, wb])
    u = sp.Matrix([Vf, Vb])
    op_sub = list(zip(x, x_op)) + list(zip(u, u_op))

    dp_rhs = dp
    de_rhs = de
    dl_rhs = dl
    ddp_rhs = 1/(2*mc.m_p*mc.l_p**2) * (2*mc.m_p*mc.l_p**2*sp.cos(p)*sp.sin(p)*(de**2-sp.cos(e)**2*dl**2)
                                        - mc.d_p * dp
                                        + mc.l_p*(wf-wb)
                                        + mc.J_m * sp.cos(p) * de * (wb - wf)
                                        + mc.J_m * sp.sin(p) * sp.cos(e) * dp * (wf - wb)
                                        )
    dde_rhs = 1/(mc.m_c*mc.l_c**2+2*mc.m_p*(mc.l_h**2+mc.l_p**2*sp.sin(p)**2))*(
        - sp.cos(e)*sp.sin(e)*(mc.m_c*mc.l_c**2+2*mc.m_p*(mc.l_h**2-mc.l_p**2*sp.sin(p)**2))*dl**2
        - mc.d_e*de
        + mc.g*(mc.m_c*mc.l_c-2*mc.m_p*mc.l_h)*sp.cos(e)
        + mc.l_h*sp.cos(p)*(wf+wb)
        + sp.sin(p)*mc.K_m*(wf-wb)
        + mc.J_m * sp.cos(p) * dp * (wf - wb)
        + mc.J_m * sp.cos(p) * sp.sin(e) * dp * (wb - wf)
    )
    ddl_rhs = 1/(mc.m_c*(mc.l_c*sp.cos(e))**2+2*mc.m_p*((mc.l_h*sp.cos(e))**2+(mc.l_p*sp.sin(p)*sp.sin(e))**2+(mc.l_p*sp.cos(p))**2))*(
        mc.l_h*sp.cos(e)*sp.sin(p)*(wf+wb)
        - mc.d_l*dl
        + sp.cos(e)*sp.cos(p)*mc.K_m*(wb-wf)
        + mc.J_m * sp.sin(p) * sp.cos(e) * dp * (wf - wb)
    )
    dwf_rhs = 1/mc.T_f * (Vf - wf)
    dwb_rhs = 1/mc.T_b * (Vb - wb)

    ss_rhs = sp.Matrix([dp_rhs, de_rhs, dl_rhs, ddp_rhs, dde_rhs, ddl_rhs, dwf_rhs, dwb_rhs])
    A_symbolic = ss_rhs.jacobian(x)
    B_symbolic = ss_rhs.jacobian(u)

    A = A_symbolic.subs(op_sub)
    B = B_symbolic.subs(op_sub)

    return np.array(A).astype(np.float64), np.array(B).astype(np.float64)


def compute_feed_forward_static(e_and_derivatives, lambda_and_derivatives):
    e = e_and_derivatives[0]
    Vs = - L2 / L3 * cos(e)
    Vd = 0

    Vf = (Vs + Vd) / 2
    Vb = (Vs - Vd) / 2

    return Vf, Vb


def compute_feed_forward_flatness(model_type : ModelType, e_and_derivatives, lambda_and_derivatives):
    if model_type == ModelType.EASY:
        # print("simple model is used")
        pitch, system_in = compute_pitch_and_inputs_flatness_simple(e_and_derivatives, lambda_and_derivatives)
    elif model_type == ModelType.CENTRIPETAL:
        # print("centripetal model is used")
        pitch, system_in = compute_pitch_and_inputs_flatness_centripetal(e_and_derivatives, lambda_and_derivatives)
    else:
        print("Model type is not supported for flatness control. Model Type EASY will be used for calculations.")
        pitch, system_in = compute_pitch_and_inputs_flatness_simple(e_and_derivatives, lambda_and_derivatives)
    return system_in


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


def compute_pitch_and_inputs_flatness_centripetal(e_and_derivatives, lambda_and_derivatives):
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
    dl_2 = 2 * (dl2**2 + dl1 * dl3)
    df2 = Je * (dk2 * l_ + 2*dk1 *dl_1 + k * dl_2)
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
    dA2 = ((dh2 * j - h * dj2) * j - (dh1 * j - h * dj1) * 2 * dj1) / (j**3)

    x = (L3/L4) * A
    dx1 = (L3/L4) * dA1
    dx2 = (L3/L4) * dA2

    # p = arctan((L3/L4) * A)
    # dp1 = (L3 / L4) * 1/(1+((L3/L4) * A)**2) * dA1
    # dp2 = (L3 / L4) * (dA2 * (1+((L3/L4) * A)**2) - (L3/L4) * dA1**2) / ((1+((L3/L4) * A)**2)**2)
    p = arctan(x)
    dp1 = dx1/(1+x**2)
    dp2 = ((1+x**2) * dx2 - 2 * x * dx1**2)/((1+x**2)**2)

    # Vs = (Jl * dl2 + mc.d_l * dl1) / (L4 * cos(e) * sin(p))
    # Vs without Singularity
    Vs = (((Jl * dl2 + mc.d_l * dl1) / (L4 * cos(e))) ** 2 + ((Je * de2 + mc.d_e * de1 + Je * cos(e) * sin(e) * dl1**2 - L2 * cos(e)) / L3) ** 2) ** (1 / 2)
    Vd = (1/L1) * (Jp * dp2 + mc.d_p * dp1 - Jp * cos(p) * sin(p) * (de1**2 - cos(e)**2 * dl1**2))


    Vf = (Vs + Vd) / 2
    Vb = (Vs - Vd) / 2

    return np.array([p, dp1, dp2]), np.array([Vf, Vb])
