from ModelConstants import OriginalConstants as mc
from ModelConstants import ModelType
from HeliSimulation import system_f, Fr_inverse
from enum import Enum
from numpy.ma import cos, sin, arctan, sqrt
import numpy as np
import sympy as sp

L1 = mc.l_p
L2 = mc.g * (mc.l_c * mc.m_c - mc.l_h * mc.m_h)
L3 = mc.l_h
L4 = mc.l_h

Jp = mc.m_h * mc.l_p ** 2
Je = mc.m_c * mc.l_c ** 2 + mc.m_h * mc.l_h ** 2
Jl = mc.m_c * mc.l_c ** 2 + mc.m_h * (mc.l_h ** 2 + mc.l_p ** 2)


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
                      [0, 0, 0, - mc.mu_phi / Jp, 0, 0],
                      [0, -L2 * sin(e_op) / Je, 0, 0, - mc.mu_eps / Je, 0],
                      [L4 * Vs_op * cos(e_op) / Jl, 0, 0, 0, 0, - mc.mu_lamb / Jl]])
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


def generate_A_and_B_functions():
    phi, eps, lamb = sp.symbols(r"p e \lambda")
    dphi, deps, dlamb = sp.symbols(r"\dot{\varphi} \dot{\varepsilon} \dot{\lambda}")
    wf, wb, uf, ub = sp.symbols(r"\omega_f \omega_b u_f u_b")

    x = sp.Matrix([phi, eps, lamb, dphi, deps, dlamb, wf, wb])
    u = sp.Matrix([uf, ub])

    dx = system_f(x, u, ModelType.NO_SIMPLIFICATIONS, True)

    A_symbolic = dx.jacobian(x)
    B_symbolic = dx.jacobian(u)

    A_lambda = sp.lambdify([phi, eps, lamb, dphi, deps, dlamb, wf, wb, uf, ub], A_symbolic)
    B_lambda = sp.lambdify([phi, eps, lamb, dphi, deps, dlamb, wf, wb, uf, ub], B_symbolic)

    return A_lambda, B_lambda


A_lambda, B_lambda = generate_A_and_B_functions()


def compute_linear_ss_full(x_op, u_op):
    """
    Computes linear state space matrices A and B for the full model (without any simplifications) at operating point
    x_op. This state space model contains all 8 (!) state variables.

    :return: A, B
    """
    A = A_lambda(x_op[0], x_op[1], x_op[2], x_op[3], x_op[4], x_op[5], x_op[6], x_op[7], u_op[0], u_op[1])
    B = B_lambda(x_op[0], x_op[1], x_op[2], x_op[3], x_op[4], x_op[5], x_op[6], x_op[7], u_op[0], u_op[1])

    return A, B


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


def compute_pitch_and_inputs_flatness_centripetal(eps_derivs, lamb_derivs):
    eps = eps_derivs[0]
    deps1 = eps_derivs[1]
    deps2 = eps_derivs[2]
    deps3 = eps_derivs[3]
    deps4 = eps_derivs[4]

    lamb = lamb_derivs[0]
    dlamb1 = lamb_derivs[1]
    dlamb2 = lamb_derivs[2]
    dlamb3 = lamb_derivs[3]
    dlamb4 = lamb_derivs[4]

    p_phi_1 = mc.m_h * mc.d_h**2 + mc.m_h * mc.l_p**2
    p_phi_2 = -mc.g*mc.d_h*mc.m_h
    p_eps_1 = mc.m_h*(mc.l_h**2+mc.d_h**2) + mc.m_c * (mc.l_c**2 + mc.d_c**2)
    p_eps_2 = mc.g*(mc.m_c*mc.d_c - mc.m_h * mc.d_h)
    p_eps_3 = mc.g*(mc.m_h*mc.l_h - mc.m_c*mc.l_c)
    p_lamb_1 = mc.m_h*(mc.l_h**2+mc.l_p**2) + mc.m_c*mc.l_c**2

    A = p_eps_1*deps2 + mc.mu_eps*deps1+p_eps_2*sin(eps)+p_eps_3*cos(eps)
    B = p_lamb_1 * dlamb2 + mc.mu_lamb*dlamb1

    dA1 = p_eps_1*deps3+mc.mu_eps*deps2+p_eps_2*deps1*cos(eps)-p_eps_3*deps1*sin(eps)
    dB1 = p_lamb_1*dlamb3+mc.mu_lamb*dlamb2

    dA2 = p_eps_1*deps4+mc.mu_eps*deps3+p_eps_2*(deps2*cos(eps)-deps1**2*sin(eps))-p_eps_3*(deps2*sin(eps)+deps1**2*cos(eps))
    dB2 = p_lamb_1*dlamb4+mc.mu_lamb*dlamb3

    D = dB1*A*cos(eps)-B*(dA1*cos(eps)-A*sin(eps)*deps1)
    E = (A*cos(eps))**2

    dD1 = dB2*A*cos(eps)-B*(dA2*cos(eps)-2*dA1*sin(eps)*deps1-A*(deps2*sin(eps)+deps1**2*cos(eps)))
    dE1 = 2*A*cos(eps)*(dA1*cos(eps)-A*deps1*sin(eps))

    C = B/(A*cos(eps))
    dC1 = D/E
    dC2 = (dD1*E-D*dE1)/E**2

    phi = arctan(C)
    dphi1 = dC1/(1+C**2)
    dphi2 = (dC2*(1+C**2)-2*C*dC1**2)/(1+C**2)**2

    Fs = 1/mc.l_h * sqrt(A**2 + (B/cos(eps))**2)
    Fd = 1/mc.l_p * (p_phi_1*dphi2 + p_phi_2*sin(phi)+mc.mu_phi*dphi1)

    Ff = (Fs + Fd) / 2
    Fb = (Fs - Fd) / 2

    uf = Fr_inverse(Ff)
    ub = Fr_inverse(Fb)

    return np.array([phi, dphi1, dphi2]), np.array([uf, ub])
