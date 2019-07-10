import numpy as np
import abc # abstract base class
from ModelConstants import ModelType
import scipy.linalg
from numpy.ma import cos, sin
from observer import InaccurateModelConstants as mc
from observer.HeliKalmanSimulation import HeliKalmanSimulation

L1 = mc.l_p
L2 = mc.g * (mc.l_c * mc.m_c - mc.l_h * mc.m_h)
L3 = mc.l_h
L4 = mc.l_h

Jp_static = 2 * mc.m_h * mc.l_p ** 2
Je_static = mc.m_c * mc.l_c ** 2 + 2 * mc.m_h * mc.l_h ** 2
Jl_static = mc.m_c * mc.l_c ** 2 + 2 * mc.m_h * (mc.l_h ** 2 + mc.l_p ** 2)


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
                      [-L3 * Vs_op * sin(p_op) / Je_static, -L2 * sin(e_op) / Je_static, 0, 0, 0, 0],
                      [L4 * Vs_op * cos(p_op) * cos(e_op) / Jl_static, 0, 0, 0, 0, 0]])
    elif model_type == ModelType.FRICTION:
        A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, 0, -mc.d_p / Jp_static, 0, 0],
                      [-L3 * Vs_op * sin(p_op) / Je_static, -L2 * sin(e_op) / Je_static, 0, 0, -mc.d_e / Je_static, 0],
                      [L4 * Vs_op * cos(p_op) * cos(e_op) / Jl_static, -L4 * Vs_op * sin(p_op) * sin(e_op) / Jl_static, 0, 0, 0, -mc.d_l / Jl_static]])
    elif model_type == ModelType.CENTRIPETAL:
        A = np.array([[0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1],
                      [-(de_op ** 2 - dlamb_op ** 2 * cos(e_op) ** 2) * sin(p_op) ** 2 + (de_op ** 2 - dlamb_op ** 2 * cos(e_op) ** 2) * cos(p_op) ** 2, 2 * dlamb_op ** 2 * sin(p_op) * sin(e_op) * cos(p_op) * cos(e_op), 0, -mc.d_p / Jp_static, 2 * de_op * sin(p_op) * cos(p_op), -2 * dlamb_op * sin(p_op) * cos(p_op) * cos(e_op) ** 2],
                      [-L3 * Vs_op * sin(p_op) / Je_static, dlamb_op ** 2 * sin(e_op) ** 2 - dlamb_op ** 2 * cos(e_op) ** 2 - L2 * sin(e_op) / Je_static, 0, 0, -mc.d_e / Je_static, -2 * dlamb_op * sin(e_op) * cos(e_op)],
                      [L4 * Vs_op * cos(p_op) * cos(e_op) / Jl_static, -L4 * Vs_op * sin(p_op) * sin(e_op) / Jl_static, 0, 0, 0, -mc.d_l / Jl_static]])

    B = np.array([[0, 0],
                  [0, 0],
                  [0, 0],
                  [L1 / Jp_static, -L1 / Jp_static],
                  [L3 / Je_static * cos(p_op), L3 / Je_static * cos(p_op)],
                  [L4 * sin(p_op) * cos(e_op) / Jl_static, L4 * sin(p_op) * cos(e_op) / Jl_static]])

    return A, B, Vf_op, Vb_op


def get_gyro_matrices(operating_point, v_f, v_b, dynamic_inertia):
    """":arg operating point: 8-element-vector
    :arg v_f, v_b: operating point input values
    :arg dynamic_inertia: False ==> static inertia, True ==> dynamic inertia
    :return A, B"""
    p, e, lamb, dp, de, dlamb, f_speed, b_speed = operating_point

    if dynamic_inertia:
        A = np.array([[0,0,0,1,0,0,0,0],
                     [0,0,0,0,1,0,0,0],
                     [0,0,0,0,0,1,0,0],
                     [-(mc.J_m*de*np.sin(p)*(b_speed-f_speed)+2*mc.l_p**2*mc.m_p*np.cos(p)**2*(dlamb**2*np.cos(e)**2-de**2)-2*mc.l_p**2*mc.m_p*np.sin(p)**2*(dlamb**2*np.cos(e)**2-de**2)+mc.J_m*dlamb*np.cos(e)*np.cos(p)*(b_speed-f_speed))/(2*mc.l_p**2*mc.m_p),(4*mc.m_p*np.cos(e)*np.cos(p)*np.sin(e)*np.sin(p)*dlamb**2*mc.l_p**2+mc.J_m*np.sin(e)*np.sin(p)*(b_speed-f_speed)*dlamb)/(2*mc.l_p**2*mc.m_p),0,-mc.d_p/(2*mc.l_p**2*mc.m_p),(4*de*mc.m_p*np.cos(p)*np.sin(p)*mc.l_p**2+mc.J_m*np.cos(p)*(b_speed-f_speed))/(2*mc.l_p**2*mc.m_p),-(mc.J_m*np.cos(e)*np.sin(p)*(b_speed-f_speed)+4*dlamb*mc.l_p**2*mc.m_p*np.cos(e)**2*np.cos(p)*np.sin(p))/(2*mc.l_p**2*mc.m_p),(mc.K*L1-mc.J_m*de*np.cos(p)+mc.J_m*dlamb*np.cos(e)*np.sin(p))/(2*mc.l_p**2*mc.m_p),-(mc.K*L1-mc.J_m*de*np.cos(p)+mc.J_m*dlamb*np.cos(e)*np.sin(p))/(2*mc.l_p**2*mc.m_p)],
                     [(4*mc.l_p**2*mc.m_p*np.cos(p)*np.sin(p)*(np.cos(e)*np.sin(e)*(mc.m_c*mc.l_c**2+2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2))*dlamb**2-mc.J_m*np.cos(p)*np.sin(e)*(b_speed-f_speed)*dlamb+mc.d_e*de-L2*np.cos(e)+mc.K_m*np.sin(p)*(b_speed-f_speed)+mc.J_m*dp*np.cos(p)*(b_speed-f_speed)-mc.K*L3*np.cos(p)*(b_speed+f_speed)))/(mc.m_c*mc.l_c**2+2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2))**2-(4*mc.m_p*np.cos(e)*np.cos(p)*np.sin(e)*np.sin(p)*dlamb**2*mc.l_p**2+mc.J_m*np.sin(e)*np.sin(p)*(b_speed-f_speed)*dlamb+mc.K_m*np.cos(p)*(b_speed-f_speed)+mc.K*L3*np.sin(p)*(b_speed+f_speed)-mc.J_m*dp*np.sin(p)*(b_speed-f_speed))/(2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2)+mc.l_c**2*mc.m_c),-(L2*np.sin(e)+dlamb**2*np.cos(e)**2*(2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2)+mc.l_c**2*mc.m_c)-dlamb**2*np.sin(e)**2*(2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2)+mc.l_c**2*mc.m_c)-mc.J_m*dlamb*np.cos(e)*np.cos(p)*(b_speed-f_speed))/(2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2)+mc.l_c**2*mc.m_c),0,-(mc.J_m*np.cos(p)*(b_speed-f_speed))/(2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2)+mc.l_c**2*mc.m_c),-mc.d_e/(2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2)+mc.l_c**2*mc.m_c),-(2*dlamb*np.cos(e)*np.sin(e)*(2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2)+mc.l_c**2*mc.m_c)-mc.J_m*np.cos(p)*np.sin(e)*(b_speed-f_speed))/(2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2)+mc.l_c**2*mc.m_c),(mc.K_m*np.sin(p)+mc.K*L3*np.cos(p)+mc.J_m*dp*np.cos(p)-mc.J_m*dlamb*np.cos(p)*np.sin(e))/(2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2)+mc.l_c**2*mc.m_c),-(mc.K_m*np.sin(p)-mc.K*L3*np.cos(p)+mc.J_m*dp*np.cos(p)-mc.J_m*dlamb*np.cos(p)*np.sin(e))/(2*mc.m_p*(mc.l_h**2+mc.l_p**2*np.sin(p)**2)+mc.l_c**2*mc.m_c)],
                     [-(mc.K_m*np.cos(e)*np.sin(p)*(b_speed-f_speed)+mc.J_m*dlamb*np.cos(e)*np.cos(p)*(b_speed-f_speed)+mc.J_m*dp*np.cos(e)*np.cos(p)*(b_speed-f_speed)-mc.K*L4*np.cos(e)*np.cos(p)*(b_speed+f_speed))/(2*mc.m_p*(mc.l_h**2*np.cos(e)**2+mc.l_p**2*np.cos(e)**2*np.sin(p)**2+mc.l_p**2*np.cos(p)**2)+mc.l_c**2*mc.m_c*np.cos(e)**2)-(2*mc.m_p*(2*mc.l_p**2*np.cos(p)*np.sin(p)-2*mc.l_p**2*np.cos(e)**2*np.cos(p)*np.sin(p))*(mc.d_l*dlamb-mc.K_m*np.cos(e)*np.cos(p)*(b_speed-f_speed)+mc.J_m*dlamb*np.cos(e)*np.sin(p)*(b_speed-f_speed)+mc.J_m*dp*np.cos(e)*np.sin(p)*(b_speed-f_speed)-mc.K*L4*np.cos(e)*np.sin(p)*(b_speed+f_speed)))/(2*mc.m_p*(mc.l_h**2*np.cos(e)**2+mc.l_p**2*np.cos(e)**2*np.sin(p)**2+mc.l_p**2*np.cos(p)**2)+mc.l_c**2*mc.m_c*np.cos(e)**2)**2,-(mc.K_m*np.cos(p)*np.sin(e)*(b_speed-f_speed)-mc.J_m*dlamb*np.sin(e)*np.sin(p)*(b_speed-f_speed)-mc.J_m*dp*np.sin(e)*np.sin(p)*(b_speed-f_speed)+mc.K*L4*np.sin(e)*np.sin(p)*(b_speed+f_speed))/(2*mc.m_p*(mc.l_h**2*np.cos(e)**2+mc.l_p**2*np.cos(e)**2*np.sin(p)**2+mc.l_p**2*np.cos(p)**2)+mc.l_c**2*mc.m_c*np.cos(e)**2)-((2*mc.m_c*np.cos(e)*np.sin(e)*mc.l_c**2+2*mc.m_p*(2*np.cos(e)*np.sin(e)*mc.l_h**2+2*np.cos(e)*np.sin(e)*mc.l_p**2*np.sin(p)**2))*(mc.d_l*dlamb-mc.K_m*np.cos(e)*np.cos(p)*(b_speed-f_speed)+mc.J_m*dlamb*np.cos(e)*np.sin(p)*(b_speed-f_speed)+mc.J_m*dp*np.cos(e)*np.sin(p)*(b_speed-f_speed)-mc.K*L4*np.cos(e)*np.sin(p)*(b_speed+f_speed)))/(2*mc.m_p*(mc.l_h**2*np.cos(e)**2+mc.l_p**2*np.cos(e)**2*np.sin(p)**2+mc.l_p**2*np.cos(p)**2)+mc.l_c**2*mc.m_c*np.cos(e)**2)**2,0,-(mc.J_m*np.cos(e)*np.sin(p)*(b_speed-f_speed))/(2*mc.m_p*(mc.l_h**2*np.cos(e)**2+mc.l_p**2*np.cos(e)**2*np.sin(p)**2+mc.l_p**2*np.cos(p)**2)+mc.l_c**2*mc.m_c*np.cos(e)**2),0,-(mc.d_l+mc.J_m*np.cos(e)*np.sin(p)*(b_speed-f_speed))/(2*mc.m_p*(mc.l_h**2*np.cos(e)**2+mc.l_p**2*np.cos(e)**2*np.sin(p)**2+mc.l_p**2*np.cos(p)**2)+mc.l_c**2*mc.m_c*np.cos(e)**2),(mc.K*L4*np.cos(e)*np.sin(p)-mc.K_m*np.cos(e)*np.cos(p)+mc.J_m*dlamb*np.cos(e)*np.sin(p)+mc.J_m*dp*np.cos(e)*np.sin(p))/(2*mc.m_p*(mc.l_h**2*np.cos(e)**2+mc.l_p**2*np.cos(e)**2*np.sin(p)**2+mc.l_p**2*np.cos(p)**2)+mc.l_c**2*mc.m_c*np.cos(e)**2),(mc.K_m*np.cos(e)*np.cos(p)+mc.K*L4*np.cos(e)*np.sin(p)-mc.J_m*dlamb*np.cos(e)*np.sin(p)-mc.J_m*dp*np.cos(e)*np.sin(p))/(2*mc.m_p*(mc.l_h**2*np.cos(e)**2+mc.l_p**2*np.cos(e)**2*np.sin(p)**2+mc.l_p**2*np.cos(p)**2)+mc.l_c**2*mc.m_c*np.cos(e)**2)],
                     [0,0,0,0,0,0,-1/mc.T_f,0],
                     [0,0,0,0,0,0,0,-1/mc.T_b]])
    else:
        A = np.array([[0, 0, 0, 1, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0],
                      [(-mc.J_m*de*(b_speed - f_speed)*np.sin(p) + mc.J_m*dlamb*(-b_speed + f_speed)*np.cos(e)*np.cos(p) - Jp_static*(de**2 - dlamb**2*np.cos(e)**2)*np.sin(p)**2 + Jp_static*(de**2 - dlamb**2*np.cos(e)**2)*np.cos(p)**2)/Jp_static, (-mc.J_m*dlamb*(-b_speed + f_speed)*np.sin(e)*np.sin(p) + 2*Jp_static*dlamb**2*np.sin(e)*np.sin(p)*np.cos(e)*np.cos(p))/Jp_static, 0, -mc.d_p/Jp_static, (mc.J_m*(b_speed - f_speed)*np.cos(p) + 2*Jp_static*de*np.sin(p)*np.cos(p))/Jp_static, (mc.J_m*(-b_speed + f_speed)*np.sin(p)*np.cos(e) - 2*Jp_static*dlamb*np.sin(p)*np.cos(e)**2*np.cos(p))/Jp_static, (-mc.J_m*de*np.cos(p) + mc.J_m*dlamb*np.sin(p)*np.cos(e) + mc.K*L1)/Jp_static, (mc.J_m*de*np.cos(p) - mc.J_m*dlamb*np.sin(p)*np.cos(e) - mc.K*L1)/Jp_static],
                      [(-mc.J_m*dlamb*(b_speed - f_speed)*np.sin(e)*np.sin(p) - mc.J_m*dp*(-b_speed + f_speed)*np.sin(p) - mc.K*L3*(b_speed + f_speed)*np.sin(p) + mc.K_m*(-b_speed + f_speed)*np.cos(p))/Je_static, (Je_static*dlamb**2*np.sin(e)**2 - Je_static*dlamb**2*np.cos(e)**2 + mc.J_m*dlamb*(b_speed - f_speed)*np.cos(e)*np.cos(p) - L2*np.sin(e))/Je_static, 0, mc.J_m*(-b_speed + f_speed)*np.cos(p)/Je_static, -mc.d_e/Je_static, (-2*Je_static*dlamb*np.sin(e)*np.cos(e) + mc.J_m*(b_speed - f_speed)*np.sin(e)*np.cos(p))/Je_static, (-mc.J_m*dlamb*np.sin(e)*np.cos(p) + mc.J_m*dp*np.cos(p) + mc.K*L3*np.cos(p) + mc.K_m*np.sin(p))/Je_static, (mc.J_m*dlamb*np.sin(e)*np.cos(p) - mc.J_m*dp*np.cos(p) + mc.K*L3*np.cos(p) - mc.K_m*np.sin(p))/Je_static],
                      [(mc.J_m*dlamb*(-b_speed + f_speed)*np.cos(e)*np.cos(p) + mc.J_m*dp*(-b_speed + f_speed)*np.cos(e)*np.cos(p) + mc.K*L4*(b_speed + f_speed)*np.cos(e)*np.cos(p) - mc.K_m*(b_speed - f_speed)*np.sin(p)*np.cos(e))/Jl_static, (-mc.J_m*dlamb*(-b_speed + f_speed)*np.sin(e)*np.sin(p) - mc.J_m*dp*(-b_speed + f_speed)*np.sin(e)*np.sin(p) - mc.K*L4*(b_speed + f_speed)*np.sin(e)*np.sin(p) - mc.K_m*(b_speed - f_speed)*np.sin(e)*np.cos(p))/Jl_static, 0, mc.J_m*(-b_speed + f_speed)*np.sin(p)*np.cos(e)/Jl_static, 0, (mc.J_m*(-b_speed + f_speed)*np.sin(p)*np.cos(e) - mc.d_l)/Jl_static, (mc.J_m*dlamb*np.sin(p)*np.cos(e) + mc.J_m*dp*np.sin(p)*np.cos(e) + mc.K*L4*np.sin(p)*np.cos(e) - mc.K_m*np.cos(e)*np.cos(p))/Jl_static, (-mc.J_m*dlamb*np.sin(p)*np.cos(e) - mc.J_m*dp*np.sin(p)*np.cos(e) + mc.K*L4*np.sin(p)*np.cos(e) + mc.K_m*np.cos(e)*np.cos(p))/Jl_static],
                      [0, 0, 0, 0, 0, 0, -1/mc.T_f, 0],
                      [0, 0, 0, 0, 0, 0, 0, -1/mc.T_b]])
    B = np.array([[0, 0],
                  [0, 0],
                  [0, 0],
                  [0, 0],
                  [0, 0],
                  [0, 0],
                  [mc.K_f/mc.T_f, 0],
                  [0, mc.K_b/mc.T_b]])
    return A, B


def get_operating_point_inputs(e_op, model_type: ModelType):
    ''':return vf_op, vb_op: steady-state inputs'''
    if model_type == ModelType.GYROMOMENT:
        f_plus_b = - (L2/L3) * np.cos(e_op)
        f = f_plus_b / 2
        b = f_plus_b / 2

        vf_op = f / mc.K_f
        vb_op = b / mc.K_b
    elif model_type == ModelType.EASY:
        v_s = - L2 / L3 * cos(e_op)
        vf_op = v_s / 2
        vb_op = v_s / 2
    else:
        raise NotImplementedError("[ERROR] At the moment only EASY and Gyromoment are implemented")

    return vf_op, vb_op


def compute_exp_matrix_intergration(A,T,nbins=100):
    """https://math.stackexchange.com/questions/658276/integral-of-matrix-exponential"""
    f = lambda x: scipy.linalg.expm(A*x)
    xv = np.linspace(0, T, nbins)
    result = np.apply_along_axis(f, 0, xv.reshape(1, -1))
    return np.trapz(result, xv)


def discretize_linear_state_space(At, Bt, Ct, Dt, T):
    """Calculates the discrete state space matrices Ak, Bk, Ck and Dk"""
    n = At.shape[0] # it is assumed that At is quadratic
    Ak = scipy.linalg.expm(At * T)
    if np.linalg.det(At) != 0:
        Bk = np.linalg.inv(At) * (Ak - np.eye(n)) * Bt
    else:
        Bk = compute_exp_matrix_intergration(At, T, nbins=5) @ Bt
    Ck = Ct
    Dk = Dt
    return Ak, Bk, Ck, Dk


class Observer(object):
    """Base class for observer"""

    def __init__(self, init_state):
        self.nStateVariables = np.size(init_state)
        self.x_estimated_state = np.resize(np.array(init_state), (np.size(init_state), 1))

    @abc.abstractmethod
    def calc_observation(self, t, x, u):
        """Estimates the system state dependent on the output of the system.
                    Args:
                        t: current simulation time
                        x: current system state. x = [p, e, lambda, dp/dt, de/dt, dlambda/dt, f, b]
                        u: current controller output (u[0]: Vf, u[1]: Vb)
                    Returns:
                        x_hat: estimated state of system
                        noisy_input: input signal with noise
                        noisy_output; output signal with noise
                        cov_matrix: current covariance matrix (8x8)"""
        return

    def set_estimated_state(self, x_estimated_state):
        self.x_estimated_state = np.resize(np.array(x_estimated_state), (self.nStateVariables, 1))

    def get_estimated_state(self):
        return np.transpose(self.x_estimated_state)[0]

    def set_dynamic_inertia(self, dynamic_inertia):
        self.dynamic_inertia = dynamic_inertia
        print("Dynamic Inertia was set to " + str(self.dynamic_inertia))

    def set_should_limit(self, should_check_limits):
        self.should_check_limits = should_check_limits


class KalmanFilterBase(Observer):
    """Base class for Kalman Filter. Takes care of adding noise to input and output signals.
    Uses the variable naming conventions of the lecture 'Steuerung mobiler Roboter' of Prof. Janschek"""

    def __init__(self, init_state, init_cov_matrix, model_type: ModelType, nOutputs, input_variance = (0.25/50) ** 2,
                 output_variance = (0.5 / 180 * np.pi) ** 2, input_noise_variance = (0.25/50) ** 2,
                 output_noise_variance = (0.5 / 180 * np.pi) ** 2):
        ''':arg input_variance: the value of the main diagonal of the input noise matrix, NOT THE ACTUAL NOISE
        :arg: output_variance: the value of the main diagonal of the output noise matrix, NOT THE ACUTAL NOISE
        :arg input_noise_variance: the variance of the input noise
        :arg output_noise_variance: the variance of the output noise'''
        if model_type == ModelType.EASY:
            # delete the last two states because these are not present in the model
            init_state = init_state[0:6]
            init_cov_matrix = init_cov_matrix[0:6, 0:6]

        super().__init__(init_state)
        self.cov_matrix = init_cov_matrix
        self.model_type = model_type

        self.bInputNoise = True
        self.bOutputNoise = True
        # Backup, if noise is disabled and then again enabled
        self.input_noise_variance = input_noise_variance
        self.output_noise_variance = output_noise_variance
        self.no_output_noise_variance = (0.00001 / 180 * np.pi) ** 2

        self.input_variance = input_variance
        self.output_variance = output_variance
        self.nOutputs = nOutputs

        # set system noise parameters
        if self.bInputNoise:
            self.vf_var = input_noise_variance
            self.vb_var = input_noise_variance
        else:
            self.vf_var = 0
            self.vb_var = 0
        if self.bOutputNoise:
            self.p_var = output_noise_variance
            self.e_var = output_noise_variance
            self.lamb_var = output_noise_variance
            self.f_var = output_noise_variance
            self.b_var = output_noise_variance
        else:
            # if W is singular, then the matrix to be inversed can happen to be the 0-Matrix
            self.p_var = self.no_output_noise_variance
            self.e_var = self.no_output_noise_variance
            self.lamb_var = self.no_output_noise_variance
            self.f_var = self.no_output_noise_variance
            self.b_var = self.no_output_noise_variance
        # N is the covariance matrix of the input signals. 2 inputs ==> N is a 2x2 matrix
        # Assuming white noise
        self.N = np.diag([input_variance, input_variance])
        if nOutputs == 3:
            # W is the covariance matrix of the output signals. 3 outputs ==> W is a 3x3 matrix
            self.W = np.diag([output_variance, output_variance, output_variance])
        elif nOutputs == 5:
            # 5 Output signals ==> W is a 5x5 matrix
            self.W = np.diag([output_variance, output_variance, output_variance, output_variance, output_variance])
        # Assuming NO PROCESS NOISE

        return

    def toggle_input_variance(self, toggle):
        if toggle:
            self.N = np.diag([self.input_variance, self.input_variance])
        else:
            self.N = np.diag([0, 0])

    def toggle_output_variance(self, toggle):
        if toggle:
            if self.nOutputs == 3:
                # W is the covariance matrix of the output signals. 3 outputs ==> W is a 3x3 matrix
                self.W = np.diag([self.output_variance, self.output_variance, self.output_variance])
            elif self.nOutputs == 5:
                # 5 Output signals ==> W is a 5x5 matrix
                self.W = np.diag([self.output_variance, self.output_variance, self.output_variance,
                                  self.output_variance, self.output_variance])
        else:
            if self.nOutputs == 3:
                # W is the covariance matrix of the output signals. 3 outputs ==> W is a 3x3 matrix
                self.W = np.diag([self.no_output_noise_variance, self.no_output_noise_variance, self.no_output_noise_variance])
            elif self.nOutputs == 5:
                # 5 Output signals ==> W is a 5x5 matrix
                self.W = np.diag([self.no_output_noise_variance, self.no_output_noise_variance, self.no_output_noise_variance,
                                  self.no_output_noise_variance, self.no_output_noise_variance])


    def toggle_input_noise(self, bInputNoise):
        '''Dis-/Enables the input noise. Doesn't modify the N-Matrix'''
        self.bInputNoise = bInputNoise
        if self.bInputNoise:
            self.vf_var = self.input_noise_variance
            self.vb_var = self.input_noise_variance
        else:
            self.vf_var = 0
            self.vb_var = 0

    def toggle_output_noise(self, bOutputNoise):
        '''Dis-/Enables the output noise. Doesn't modify the W-Matrix'''
        self.bOutputNoise = bOutputNoise
        if self.bOutputNoise:
            self.p_var = self.output_noise_variance
            self.e_var = self.output_noise_variance
            self.lamb_var = self.output_noise_variance
            self.f_var = self.output_noise_variance
            self.b_var = self.output_noise_variance
        else:
            # if W is singular, then the matrix to be inversed can happen to be the 0-Matrix
            self.p_var = self.no_output_noise_variance
            self.e_var = self.no_output_noise_variance
            self.lamb_var = self.no_output_noise_variance
            self.f_var = self.no_output_noise_variance
            self.b_var = self.no_output_noise_variance

    def get_noisy_output_of_system(self, y_without_noise):
        """Adds gaussian (white) noise to the output signals
        :arg y_without_noise: n-d-array with 3 elements (p, e, lambda). Dimension: 1x3"""
        # There were some problems with copying the array data so I just wrote a copy command for every single line
        if self.bOutputNoise:
            if np.size(y_without_noise, 0) == 3:
                y_with_noise = np.zeros(3)
                y_with_noise[0] = y_without_noise[0] + np.random.normal(0, np.sqrt(self.p_var), 1)[0]
                y_with_noise[1] = y_without_noise[1] + np.random.normal(0, np.sqrt(self.e_var), 1)[0]
                y_with_noise[2] = y_without_noise[2] + np.random.normal(0, np.sqrt(self.lamb_var), 1)[0]
            elif np.size(y_without_noise, 0) == 5:
                y_with_noise = np.zeros(5)
                y_with_noise[0] = y_without_noise[0] + np.random.normal(0, np.sqrt(self.p_var), 1)[0]
                y_with_noise[1] = y_without_noise[1] + np.random.normal(0, np.sqrt(self.e_var), 1)[0]
                y_with_noise[2] = y_without_noise[2] + np.random.normal(0, np.sqrt(self.lamb_var), 1)[0]
                y_with_noise[3] = y_without_noise[3] + np.random.normal(0, np.sqrt(self.f_var), 1)[0]
                y_with_noise[4] = y_without_noise[4] + np.random.normal(0, np.sqrt(self.b_var), 1)[0]
        else:
            y_with_noise = y_without_noise
        return y_with_noise

    def get_noisy_input_of_system(self, u_without_noise):
        """Adds gaussian (white) noise to the input signals
        :arg u_without_noise: n-d-array with 2 elements (Vf, Vb). Dimension: 2x1"""
        u_with_noise = np.zeros(2)
        if self.bInputNoise:
            u_with_noise[0] = u_without_noise[0] + np.random.normal(0, np.sqrt(self.vf_var), 1)[0]
            u_with_noise[1] = u_without_noise[1] + np.random.normal(0, np.sqrt(self.vb_var), 1)[0]
        else:
            u_with_noise = u_without_noise
        return u_with_noise

    @abc.abstractmethod
    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        """Sets the current model"""
        return


    @abc.abstractmethod
    def calc_observation(self, t, x, u):
        """Estimates the system state dependent on the output of the system.
                    Args:
                        t: current simulation time
                        x: current system state. x = [p, e, lambda, dp/dt, de/dt, dlambda/dt]
                        u: current controller output
                    Returns:
                        x_hat: estimated state of system
                        noisy_input: input signal with noise
                        noisy_output; output signal with noise"""
        return


class NoKalmanFilter(KalmanFilterBase):

    def __init__(self, init_state, init_cov_matrix):
        super().__init__(init_state, init_cov_matrix, ModelType.EASY, 3)

    def calc_observation(self, t, x, u):
        # add noise to input and output
        u_with_noise = self.get_noisy_input_of_system(u)
        y_with_noise = self.get_noisy_output_of_system(x[0:3])
        y_with_noise = np.pad(y_with_noise, (0, 2), "constant")
        return x, u_with_noise, y_with_noise, np.zeros((8, 8))

    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        """nothing here"""
        return


class LinearKalmanFilterWithLimits(KalmanFilterBase):
    """This kalman filter linearizes the model in a fixed operating point which is defined at initialisation."""

    def __init__(self, init_state, init_cov_matrix, model_type : ModelType, e_op, nOutputs, stepSize,
                 input_variance=(0.25/50) ** 2, output_variance=(0.5 / 180 * np.pi) ** 2,
                 input_noise_variance=(0.25/50) ** 2, output_noise_variance=(0.5 / 180 * np.pi) ** 2):
        ''':arg operating_point: 8-element-vector'''
        super().__init__(init_state, init_cov_matrix, model_type, nOutputs, input_variance, output_variance,
                         input_noise_variance, output_noise_variance)
        if np.size(init_cov_matrix, 0) != np.size(init_state, 0):
            raise Exception("dim(init_x) = " + str(np.size(init_state)) + " dim(init_cov_matrix) = " +
                            str(np.size(init_cov_matrix)))
        self.timeStep = stepSize
        vf_op, vb_op = get_operating_point_inputs(e_op, model_type)
        # the lambda op value has no influence on the linear matrices.
        # because of it wasn't my intention to give the possibility to enter a really flexible operating point
        # i will just set it to 0 and remove it from the gui
        lamb_op = 0
        self.operating_point = np.array([0, e_op, lamb_op, 0, 0, 0, vf_op * mc.K_f, vb_op * mc.K_b])
        self.model_type = model_type
        self.nOutputs = nOutputs
        self.heliSim = HeliKalmanSimulation(0, 0, 0, stepSize)
        self.heliSim.set_model_type(ModelType.EASY)
        if model_type == ModelType.EASY:
            self.nStateVars = 6
            self.heliSim.set_current_state_and_time(np.concatenate((init_state, np.zeros(2))), self.heliSim.get_current_time())
        elif model_type == ModelType.GYROMOMENT:
            self.nStateVars = 8
            self.heliSim.set_current_state_and_time(init_state, self.heliSim.get_current_time())

        # Linearize and discretize the system in the operating point because it is already fixed
        vf_op, vb_op = get_operating_point_inputs(e_op, self.model_type)
        self.u_op = np.array([[vf_op], [vb_op]])
        self.Ak, self.Bk, self.Ck, self.Dk = self.get_linear_discrete_matrices()

    def set_estimated_state(self, x_estimated_state):
        x_estimated_state = x_estimated_state[0:self.nStateVars]
        if self.model_type == ModelType.EASY:
            self.heliSim.set_current_state_and_time(np.concatenate((x_estimated_state, np.zeros(2))),
                                                    self.heliSim.get_current_time())
        elif self.model_type == ModelType.GYROMOMENT:
            self.heliSim.set_current_state_and_time(x_estimated_state, self.heliSim.get_current_time())
        super().set_estimated_state(x_estimated_state)

    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        '''The model_type can't be set by this function and the parent function should be changed.'''
        self.timeStep = stepSize
        return

    def get_linear_discrete_matrices(self):
        """Linearizes and discretizes the current system model at the operating point
        and saves it for calc_observation"""

        vf_op, vb_op = get_operating_point_inputs(self.operating_point[1], self.model_type)
        if self.model_type == ModelType.EASY:
            At, Bt, Vf_op, Vd_op = getLinearizedMatrices(self.model_type, self.operating_point[0:6], vf_op, vb_op)
        elif self.model_type == ModelType.GYROMOMENT:
            At, Bt = get_gyro_matrices(self.operating_point, self.operating_point[6] / mc.K_f, vf_op, vb_op)
        if self.model_type == ModelType.EASY:
            Ct = np.array([[1, 0, 0, 0, 0, 0],
                          [0, 1, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0]])
            Dt = np.array([[0, 0],
                           [0, 0],
                           [0, 0]])
        elif self.model_type == ModelType.GYROMOMENT:
            if self.nOutputs == 3:
                Ct = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0, 0, 0],
                               [0, 0, 1, 0, 0, 0, 0, 0]])
                Dt = np.array([[0, 0],
                               [0, 0],
                               [0, 0],
                               [0, 0],
                               [0, 0]])
            elif self.nOutputs == 5:
                Ct = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0, 0, 0],
                               [0, 0, 1, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0, 1, 0],
                               [0, 0, 0, 0, 0, 0, 0, 1]])
                Dt = np.array([[0, 0],
                               [0, 0],
                               [0, 0],
                               [0, 0],
                               [0, 0]])

        Ak, Bk, Ck, Dk = discretize_linear_state_space(At, Bt, Ct, Dt, self.timeStep)
        return Ak, Bk, Ck, Dk

    def kf_algorithm(self, u, y):
        """Computes the estimated state using the ekf algorithm.
        :arg u: input signal with noise (Dimension: 2x1)
        :arg y: output signal with noise (Dimension: self.nOutputsx1)"""
        # For the linear filter, x_estimated_state is the difference to the operating point
        cov_matrix_before = self.cov_matrix
        # 0. Calculate difference to operating point
        u = u - self.u_op
        if self.model_type == ModelType.EASY:
            x_est_before = self.x_estimated_state - self.operating_point[0:6].reshape((6, 1))
            # x_est_before.reshape((6, 1))
        else:
            x_est_before = self.x_estimated_state - self.operating_point.reshape((8, 1))
            # x_est_before.reshape((8, 1))
        if self.nOutputs == 3:
            y = y - self.operating_point[0:3].reshape(3, 1)
        elif self.nOutputs == 5:
            y = y - np.concatenate((self.operating_point[0:3], self.operating_point[6:8])).reshape(5, 1)
        # x_est_before = self.x_estimated_state - self.operating_point
        #     1. Prediction
        # predict the state by using the linearized system at the fixed operating point
        v_s = u[0][0] + u[1][0]
        v_d = u[0][0] - u[1][0]
        x_est_predict = self.Ak @ x_est_before + self.Bk @ u
        # predict the new covariance
        cov_matrix_predict = (self.Ak @ cov_matrix_before @ np.transpose(self.Ak)
                              + self.Bk @ self.N @ np.transpose(self.Bk))
        #     2. Update
        # compute kalman gain
        Kl = (cov_matrix_predict @ np.transpose(self.Ck) @
              np.linalg.inv(self.Ck @ cov_matrix_predict @ np.transpose(self.Ck) + self.W))
        # update state
        if self.nOutputs == 3:
            y_est = x_est_predict[0:3,]
        elif self.nOutputs == 5:
            y_est = np.concatenate((x_est_predict[0:3], x_est_predict[6:8]))
        x_est_update = x_est_predict + Kl @ (y - y_est)
        # update covariance matrix (identity matrix must have as many lines as the Kalman gain
        cov_matrix_update = (np.eye(np.size(Kl, 0)) - Kl @ self.Ck) @ cov_matrix_predict
        # add again the operating point
        if self.model_type == ModelType.EASY:
            x_estimated_state = x_est_update + self.operating_point[0:6].reshape((6, 1))
            # self.x_estimated_state = x_estimated_state.reshape((1, 6))[0]
        else:
            x_estimated_state = x_est_update + self.operating_point.reshape((8, 1))
            # self.x_estimated_state = x_estimated_state.reshape((1, 8))[0]

        if self.should_check_limits:
            # check if the update step state needs to be changed because of limit crossing
            # if that is the case, correct the state and set the state of the simulation accordingly
            corrected_state = self.heliSim.get_limited_state_and_change_state_without_sim(np.transpose(x_estimated_state)[0],
                                                                              self.model_type)
            x_estimated_state = np.resize(corrected_state, (self.nStateVars, 1))
        self.x_estimated_state = x_estimated_state
        self.cov_matrix = cov_matrix_update
        # print("------")
        # print(cov_matrix_predict)
        return x_estimated_state

    def calc_observation(self, t, x, u):
        # 1. add noise to input and output
        u_with_noise = self.get_noisy_input_of_system(u)
        if self.nOutputs == 5:
            y_with_noise = self.get_noisy_output_of_system(np.concatenate((x[0:3], x[6:8])))
        elif self.nOutputs == 3:
            y_with_noise = self.get_noisy_output_of_system(x[0:3])
        # 2. execute ekf algorithm
        x_estimated_state = self.kf_algorithm(np.resize(u_with_noise, (2, 1)), np.resize(y_with_noise,
                                                                                          (self.nOutputs, 1)))
        # 3. add two zero elements at the end of the state vector
        if self.model_type == ModelType.EASY:
            x_estimated_state = np.resize(x_estimated_state, (1, 6))[0]
            x_estimated_state = np.pad(x_estimated_state, (0, 2), "constant")
            y_with_noise = np.pad(y_with_noise, (0, 2), "constant")
        elif self.model_type == ModelType.GYROMOMENT:
            x_estimated_state = np.resize(x_estimated_state, (1, 8))[0]
            if self.nOutputs == 3:
                y_with_noise = np.pad(y_with_noise, (0, 2), "constant")
        # 4. pad covariance matrix with zeros so that it has the shape 8x8
        if self.nStateVars == 6:
            cov_matrix_padded = np.pad(self.cov_matrix, ((0, 2), (0, 2)), "constant")
        elif self.nStateVars == 8:
            cov_matrix_padded = self.cov_matrix

        return x_estimated_state, u_with_noise, y_with_noise, cov_matrix_padded


class ExtKalmanFilterEasyModelLimits(KalmanFilterBase):
    """Kalman filter for the simple model, using p, e and lambda as output.
    This class takes care of the angle limits of the model."""

    def __init__(self, init_state, init_cov_matrix, stepSize, input_variance=(0.25/50) ** 2,
                 output_variance=(0.5 / 180 * np.pi) ** 2, input_noise_variance=(0.25/50) ** 2,
                 output_noise_variance=(0.5 / 180 * np.pi) ** 2):
        super().__init__(init_state, init_cov_matrix, ModelType.EASY, 3, input_variance, output_variance,
                         input_noise_variance, output_noise_variance)
        self.heliSim = HeliKalmanSimulation(0, 0, 0, stepSize)
        self.heliSim.set_current_state_and_time(init_state, self.heliSim.get_current_time())
        self.heliSim.set_model_type(ModelType.EASY)
        self.timeStep = stepSize
        self.no_disturbance_eval = np.zeros(5)

    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        # self.heliSim.set_model_type(model_type)
        return

    def set_dynamic_inertia(self, dynamic_inertia):
        self.dynamic_inertia = dynamic_inertia
        self.heliSim.set_dynamic_inertia_torque(dynamic_inertia)

    def set_should_limit(self, should_check_limits):
        self.should_check_limits = should_check_limits
        self.heliSim.set_should_limit(should_check_limits)

    def set_estimated_state(self, x_estimated_state):
        '''Overload this method in order to update the simulation object as well.'''
        # ToDo: check if this really works as expected
        super().set_estimated_state(x_estimated_state)
        self.heliSim.set_current_state_and_time(np.concatenate((x_estimated_state, np.zeros(2))), self.heliSim.get_current_time())

    def get_linear_discrete_matrices(self, operating_point, v_f, v_b):
        """Linearizes and discretizes the current system model at the operating point
        and saves it for calc_observation
        :arg operating_point: 6-element-state vector"""

        At, Bt, Vf_op, Vd_op = getLinearizedMatrices(ModelType.EASY, operating_point, v_f, v_b)
        Ct = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0]])
        Dt = np.array([[0, 0],
                      [0, 0],
                      [0, 0]])

        Ak, Bk, Ck, Dk = discretize_linear_state_space(At, Bt, Ct, Dt, self.timeStep)
        return Ak, Bk, Ck, Dk

    def ekf_algorithm(self, u, y):
        """Computes the estimated state using the ekf algorithm.
        :arg u: input signal with noise (Dimension: 2x1)
        :arg y: output signal with noise (Dimension: 3x1)"""
        x_est_before = self.x_estimated_state
        cov_matrix_before = self.cov_matrix
        #     1. Prediction
        # predict the state by integrate the time continuous system numerically
        sim_state_predict = self.heliSim.calc_step(u[0][0], u[1][0], self.no_disturbance_eval)
        x_est_predict = np.resize(sim_state_predict, (6, 1))
        # predict the new covariance by linearizing and discretizing the model
        Ak, Bk, Ck, Dk = self.get_linear_discrete_matrices(x_est_before, u[0][0], u[1][0])
        cov_matrix_predict = Ak @ cov_matrix_before @ np.transpose(Ak) + Bk @ self.N @ np.transpose(Bk)
        #     2. Update
        # compute kalman gain
        Kl = cov_matrix_predict @ np.transpose(Ck) @ np.linalg.inv(Ck @ cov_matrix_predict @ np.transpose(Ck) + self.W)
        # update state
        y_est = x_est_predict[0:3, ]
        x_est_update = x_est_predict + Kl @ (y - y_est)
        # update covariance matrix (identity matrix must have as many lines as the Kalman gain
        cov_matrix_update = (np.eye(np.size(Kl, 0)) - Kl @ Ck) @ cov_matrix_predict

        # check if the update step state needs to be changed because of limit crossing
        # if that is the case, correct the state and set the state of the simulation accordingly
        # heliSim_state = np.resize(x_est_update, (1, 6))[0]
        # heliSim_state = np.pad(heliSim_state, (0, 2), "constant")
        corrected_state = self.heliSim.get_limited_state_and_change_state(x_est_update.reshape((1, 6))[0],
                                                                          ModelType.EASY)
        x_est_update = np.resize(corrected_state, (6, 1))

        self.x_estimated_state = x_est_update
        self.cov_matrix = cov_matrix_update
        return x_est_update

    def calc_observation(self, t, x, u):
        # 1. add noise to input and output
        u_with_noise = self.get_noisy_input_of_system(u)
        y_with_noise = self.get_noisy_output_of_system(x[0:3])
        # 2. execute ekf algorithm
        x_estimated_state = self.ekf_algorithm(np.resize(u_with_noise, (2, 1)), np.resize(y_with_noise, (3, 1)))
        # 3. add two zero elements at the end of the state vector
        x_estimated_state = np.resize(x_estimated_state, (1, 6))[0]
        x_estimated_state = np.pad(x_estimated_state, (0, 2), "constant")
        y_with_noise = np.pad(y_with_noise, (0, 2), "constant")
        # 4. pad covariance matrix with zeros so that it has the shape 8x8
        cov_matrix_padded = np.pad(self.cov_matrix, ((0, 2), (0, 2)), "constant")
        return x_estimated_state, u_with_noise, y_with_noise, cov_matrix_padded


class ExtKalmanFilterGyroModelLimits(KalmanFilterBase):
    """Kalman filter for the gyromoment model, using p, e, lambda, f and b as output.
    This class takes care of the angle limits of the model."""

    def __init__(self, init_state, init_cov_matrix, stepSize, input_variance=(0.25/50) ** 2,
                 output_variance=(0.5 / 180 * np.pi) ** 2, input_noise_variance=(0.25/50) ** 2,
                 output_noise_variance=(0.5 / 180 * np.pi) ** 2):
        super().__init__(init_state, init_cov_matrix, ModelType.GYROMOMENT, 5, input_variance, output_variance,
                         input_noise_variance, output_noise_variance)
        self.heliSim = HeliKalmanSimulation(0, 0, 0, stepSize)
        self.heliSim.set_current_state_and_time(init_state, self.heliSim.get_current_time())
        self.heliSim.set_model_type(ModelType.GYROMOMENT)
        self.timeStep = stepSize
        self.no_disturbance_eval = np.zeros(5)

    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        # ToDo: This class is only for the gyro model, it's not necessary to set this model
        #   in this function
        # self.heliSim.set_model_type(model_type)
        return

    def set_should_limit(self, should_check_limits):
        self.should_check_limits = should_check_limits
        self.heliSim.set_should_limit(should_check_limits)

    def set_dynamic_inertia(self, dynamic_inertia):
        self.dynamic_inertia = dynamic_inertia
        self.heliSim.set_dynamic_inertia_torque(dynamic_inertia)

    def set_estimated_state(self, x_estimated_state):
        '''Overload this method in order update the simulation object as well.'''
        self.x_estimated_state = np.resize(np.array(x_estimated_state), (8, 1))
        self.heliSim.set_current_state_and_time(x_estimated_state, self.heliSim.get_current_time())

    def get_linear_discrete_matrices(self, operating_point, v_f, v_b):
        """Linearizes and discretizes the current system model at the operating point
        and saves it for calc_observation
        :arg operating_point: 8-element-state vector"""

        At, Bt = get_gyro_matrices(operating_point, v_f, v_b, self.dynamic_inertia)
        Ct = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1]])
        Dt = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0]])

        Ak, Bk, Ck, Dk = discretize_linear_state_space(At, Bt, Ct, Dt, self.timeStep)
        return Ak, Bk, Ck, Dk

    def ekf_algorithm(self, u, y):
        """Computes the estimated state using the ekf algorithm.
        :arg u: input signal with noise (Dimension: 2x1)
        :arg y: output signal with noise (Dimension: 3x1)"""
        x_est_before = self.x_estimated_state
        cov_matrix_before = self.cov_matrix
        #     1. Prediction
        # predict the state by integrate the time continuous system numerically
        sim_state_predict = self.heliSim.calc_step(u[0][0], u[1][0], self.no_disturbance_eval)
        x_est_predict = np.resize(sim_state_predict, (8, 1))
        # predict the new covariance by linearizing and discretizing the model
        Ak, Bk, Ck, Dk = self.get_linear_discrete_matrices(x_est_before, u[0][0], u[1][0])
        cov_matrix_predict = Ak @ cov_matrix_before @ np.transpose(Ak) + Bk @ self.N @ np.transpose(Bk)
        #     2. Update
        # compute kalman gain
        Kl = cov_matrix_predict @ np.transpose(Ck) @ np.linalg.inv(Ck @ cov_matrix_predict @ np.transpose(Ck) + self.W)
        # update state
        y_est = np.concatenate((x_est_predict[0:3], x_est_predict[6:8]))
        x_est_update = x_est_predict + Kl @ (y - y_est)
        # update covariance matrix (identity matrix must have as many lines as the Kalman gain
        cov_matrix_update = (np.eye(np.size(Kl, 0)) - Kl @ Ck) @ cov_matrix_predict

        corrected_state = self.heliSim.get_limited_state_and_change_state(x_est_update.reshape((1, 8))[0],
                                                                           ModelType.GYROMOMENT)
        # corrected_state = x_est_update
        x_est_update = np.resize(corrected_state, (8, 1))

        self.x_estimated_state = x_est_update
        self.cov_matrix = cov_matrix_update
        # print("------")
        # print(cov_matrix_predict)
        return x_est_update

    def calc_observation(self, t, x, u):
        # 1. add noise to input and output
        u_with_noise = self.get_noisy_input_of_system(u)
        y_with_noise = self.get_noisy_output_of_system(np.concatenate((x[0:3], x[6:8])))
        # 2. execute ekf algorithm
        x_estimated_state = self.ekf_algorithm(np.resize(u_with_noise, (2, 1)), np.resize(y_with_noise, (5, 1)))
        # 3. add two zero elements at the end of the state vector and at the end of the y_with_noise_vector
        x_estimated_state = np.resize(x_estimated_state, (1, 8))[0]
        return x_estimated_state, u_with_noise, y_with_noise, self.cov_matrix


class ExtKalmanFilterGyroModelLimitsOnly3(KalmanFilterBase):
    """Kalman filter for the gyromoment model, using p, e, lambda, f and b as output.
    This class takes care of the angle limits of the model."""

    def __init__(self, init_state, init_cov_matrix, stepSize, input_variance=(0.25/50) ** 2,
                 output_variance=(0.5 / 180 * np.pi) ** 2, input_noise_variance=(0.25/50) ** 2,
                 output_noise_variance=(0.5 / 180 * np.pi) ** 2):
        super().__init__(init_state, init_cov_matrix, ModelType.GYROMOMENT, 3, input_variance, output_variance,
                         input_noise_variance, output_noise_variance)
        self.heliSim = HeliKalmanSimulation(0, 0, 0, stepSize)
        self.heliSim.set_current_state_and_time(init_state, self.heliSim.get_current_time())
        self.heliSim.set_model_type(ModelType.GYROMOMENT)
        self.timeStep = stepSize
        self.no_disturbance_eval = np.zeros(5)

    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        # ToDo: hardcode this model. This class is only for the gyro model, it's not necessary to set this model
        #   in this function
        # self.heliSim.set_model_type(model_type)
        return

    def set_should_limit(self, should_check_limits):
        self.should_check_limits = should_check_limits
        self.heliSim.set_should_limit(should_check_limits)

    def set_dynamic_inertia(self, dynamic_inertia):
        self.dynamic_inertia = dynamic_inertia
        self.heliSim.set_dynamic_inertia_torque(dynamic_inertia)

    def set_estimated_state(self, x_estimated_state):
        '''Overload this method in order update the simulation object as well.'''
        self.x_estimated_state = np.resize(np.array(x_estimated_state), (8, 1))
        self.heliSim.set_current_state_and_time(x_estimated_state, self.heliSim.get_current_time())

    def get_linear_discrete_matrices(self, operating_point, v_f, v_b):
        """Linearizes and discretizes the current system model at the operating point
        and saves it for calc_observation
        :arg operating_point: 8-element-state vector"""

        At, Bt = get_gyro_matrices(operating_point, v_f, v_b, self.dynamic_inertia)
        Ct = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0, 0, 0]])
        Dt = np.array([[0, 0],
                      [0, 0],
                      [0, 0]])

        Ak, Bk, Ck, Dk = discretize_linear_state_space(At, Bt, Ct, Dt, self.timeStep)
        return Ak, Bk, Ck, Dk

    def ekf_algorithm(self, u, y):
        """Computes the estimated state using the ekf algorithm.
        :arg u: input signal with noise (Dimension: 2x1)
        :arg y: output signal with noise (Dimension: 3x1)"""
        x_est_before = self.x_estimated_state
        cov_matrix_before = self.cov_matrix
        #     1. Prediction
        # predict the state by integrate the time continuous system numerically
        sim_state_predict = self.heliSim.calc_step(u[0][0], u[1][0], self.no_disturbance_eval)
        x_est_predict = np.resize(sim_state_predict, (8, 1))
        # predict the new covariance by linearizing and discretizing the model
        Ak, Bk, Ck, Dk = self.get_linear_discrete_matrices(x_est_before, u[0][0], u[1][0])
        cov_matrix_predict = Ak @ cov_matrix_before @ np.transpose(Ak) + Bk @ self.N @ np.transpose(Bk)
        #     2. Update
        # compute kalman gain
        Kl = cov_matrix_predict @ np.transpose(Ck) @ np.linalg.inv(Ck @ cov_matrix_predict @ np.transpose(Ck) + self.W)
        # update state
        y_est = x_est_predict[0:3]
        x_est_update = x_est_predict + Kl @ (y - y_est)
        # update covariance matrix (identity matrix must have as many lines as the Kalman gain
        cov_matrix_update = (np.eye(np.size(Kl, 0)) - Kl @ Ck) @ cov_matrix_predict

        # check if the update step state needs to be changed because of limit crossing
        # if that is the case, correct the state and set the state of the simulation accordingly
        corrected_state = self.heliSim.get_limited_state_and_change_state(x_est_update.reshape((1, 8))[0],
                                                                          ModelType.GYROMOMENT)
        x_est_update = np.resize(corrected_state, (8, 1))

        self.x_estimated_state = x_est_update
        self.cov_matrix = cov_matrix_update
        # print("------")
        # print(cov_matrix_predict)
        return x_est_update

    def calc_observation(self, t, x, u):
        # 1. add noise to input and output
        u_with_noise = self.get_noisy_input_of_system(u)
        y_with_noise = self.get_noisy_output_of_system(x[0:3])
        # 2. execute ekf algorithm
        x_estimated_state = self.ekf_algorithm(np.resize(u_with_noise, (2, 1)), np.resize(y_with_noise, (3, 1)))
        # 3. add two zero elements at the end of the state vector and at the end of the y_with_noise_vector
        x_estimated_state = np.resize(x_estimated_state, (1, 6))[0]
        x_estimated_state = np.pad(x_estimated_state, (0, 2), "constant")
        y_with_noise = np.pad(y_with_noise, (0, 2), "constant")
        # 4. pad covariance matrix with zeros so that it has the shape 8x8
        cov_matrix_padded = np.pad(self.cov_matrix, ((0, 2), (0, 2)), "constant")
        return x_estimated_state, u_with_noise, y_with_noise, self.cov_matrix