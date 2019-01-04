from abc import ABC
from enum import Enum

import numpy as np
import abc # abstract base class
import math
from ModelConstants import ModelType
from helicontrollers.util import compute_linear_ss
import scipy.linalg
from numpy.ma import cos, sin
import ModelConstants as mc

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
        Bk = compute_exp_matrix_intergration(At, T) @ Bt
    Ck = Ct
    Dk = Dt
    return Ak, Bk, Ck, Dk


class Observer(object):
    """Base class for observer"""

    def __init__(self, init_state):
        self.x_estimated_state = np.array(init_state)

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
                        noisy_output; output signal with noise"""
        return

    def set_estimated_state(self, x_estimated_state):
        self.x_estimated_state = np.array(x_estimated_state)


class KalmanFilterBase(Observer):
    """Base class for Kalman Filter. Takes care of adding noise to input and output signals.
    Uses the variable naming conventions of the lecture 'Steuerung mobiler Roboter' of Prof. Janschek"""

    def __init__(self, init_state, init_cov_matrix):
        super().__init__(init_state)
        self.cov_matrix = init_cov_matrix

        # set system noise parameters
        vf_var = (0.25/20) ** 2
        vb_var = (0.25/20) ** 2
        p_var = (1 / 180 * np.pi) ** 2
        e_var = (1 / 180 * np.pi) ** 2
        lamb_var = (1 / 180 * np.pi) ** 2
        # N is the covariance matrix of the input signals. 2 inputs ==> N is a 2x2 matrix
        # Assuming white noise
        self.N = np.diag([vf_var, vb_var])
        # W is the covariance matrix of the output signals. 3 outputs ==> W is a 3x3 matrix
        self.W = np.diag([p_var, e_var, lamb_var])
        # Assuming NO PROCESS NOISE

        self.bNoise = True
        return

    def get_noisy_output_of_system(self, y_without_noise):
        """Adds gaussian (white) noise to the output signals
        :arg y_without_noise: n-d-array with 3 elements (p, e, lambda). Dimension: 1x3"""
        y_with_noise = np.zeros(3)
        if self.bNoise:
            y_with_noise[0] = y_without_noise[0] + np.random.normal(0, np.sqrt(self.W[0][0]), 1)[0]
            y_with_noise[1] = y_without_noise[1] + np.random.normal(0, np.sqrt(self.W[1][1]), 1)[0]
            y_with_noise[2] = y_without_noise[2] + np.random.normal(0, np.sqrt(self.W[2][2]), 1)[0]
        else:
            y_with_noise = y_without_noise
        return y_with_noise

    def get_noisy_input_of_system(self, u_without_noise):
        """Adds gaussian (white) noise to the input signals
        :arg u_without_noise: n-d-array with 2 elements (Vf, Vb). Dimension: 2x1"""
        u_with_noise = np.zeros(2)
        if self.bNoise:
            u_with_noise[0] = u_without_noise[0] + np.random.normal(0, np.sqrt(self.N[0][0]), 1)[0]
            u_with_noise[1] = u_without_noise[1] + np.random.normal(0, np.sqrt(self.N[1][1]), 1)[0]
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


class TestKalmanFilter(KalmanFilterBase):
    """just for testing the base class"""

    def calc_observation(self, t, x, u):
        x_estimated = x
        u_with_noise = self.get_noisy_input_of_system(u)
        y_with_noise = self.get_noisy_output_of_system(x[0:3])
        return x_estimated, u_with_noise, y_with_noise

    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        return


class ExtKalmanFilterEasyModel(KalmanFilterBase):
    """Extended Kalman filter for the easy model without angle limiting"""

    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        self.timeStep = stepSize
        return

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

    def rhs_time_continuous(self, t, x, v_s, v_d):
        p, e, lamb, dp, de, dlamb = x
        ddp = (L1 / Jp) * v_d
        dde = (L2 / Je) * np.cos(e) + (L3 / Je) * np.cos(p) * v_s
        ddlamb = (L4 / Jl) * np.cos(e) * np.sin(p) * v_s
        return np.array([dp, de, dlamb, ddp, dde, ddlamb])

    def ekf_algorithm(self, u, y):
        """Computes the estimated state using the ekf algorithm.
        :arg u: input signal with noise (Dimension: 2x1)
        :arg y: output signal with noise (Dimension: 3x1)"""
        x_est_before = self.x_estimated_state
        cov_matrix_before = self.cov_matrix
        #     1. Prediction
        # predict the state by integrate the time continuous system numerically
        v_s = u[0][0] + u[1][0]
        v_d = u[0][0] - u[1][0] # order correct?
        tt = np.linspace(0, self.timeStep, 2)
        sol = scipy.integrate.solve_ivp(lambda t, x: self.rhs_time_continuous(t, x, v_s, v_d),
                                        (0, self.timeStep), np.resize(x_est_before, (1, 6))[0],
                                        method='RK45', t_eval=tt, rtol=1e-6, atol=1e-9)
        x_est_predict = np.resize(sol.y[:, -1], (6, 1))
        # predict the new covariance by linearizing and discretizing the model
        Ak, Bk, Ck, Dk = self.get_linear_discrete_matrices(x_est_before, u[0][0], u[1][0])
        cov_matrix_predict = Ak @ cov_matrix_before @ np.transpose(Ak) + Bk @ self.N @ np.transpose(Bk)

        #     2. Update
        # compute kalman gain
        Kl = cov_matrix_predict @ np.transpose(Ck) @ np.linalg.inv(Ck @ cov_matrix_predict @ np.transpose(Ck) + self.W)
        # update state
        y_est = x_est_predict[0:3,]
        x_est_update = x_est_predict + Kl @ (y - y_est)
        # update covariance matrix (identity matrix must have as many lines as the Kalman gain
        cov_matrix_update = (np.eye(np.size(Kl, 0)) - Kl @ Ck) @ cov_matrix_predict
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
        # 3. add two zero elements at the end of the state vector
        x_estimated_state = np.resize(x_estimated_state, (1, 6))[0]
        x_estimated_state = np.pad(x_estimated_state, (0, 2), "constant")
        return x_estimated_state, u_with_noise, y_with_noise






class LinearKalmanFilter(Observer):
    """Linear Kalman Filter with static operating point. This class linearizes the current system at the
    given operating point."""

    def __init__(self, init_state, init_cov_matrix, operating_point):
        super().__init__(init_state)
        self.cov_matrix = init_cov_matrix
        self.operating_point = operating_point
        self.Ak = np.zeros((6, 6))
        self.Bk = np.zeros((6, 2))
        self.Ck = np.zeros((3, 6))
        self.Dk = np.zeros((3, 2))

        # set system noise parameters
        # these variables contain the variance of the noise
        self.p_noise = (1 / 180 * np.pi) ** 2
        self.e_noise = (1 / 180 * np.pi) ** 2
        self.lamb_noise = (1 / 180 * np.pi) ** 2
        self.output_covariance_R = np.diagflat([self.p_noise, self.e_noise, self.lamb_noise])
        # self.process_noise_Q = np.zeros((6, 6))
        self.process_noise_Q = np.diag([self.e_noise, self.e_noise, self.e_noise, self.e_noise, self.e_noise,
                                        self.e_noise])

    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        """Linearizes and discretizes the current system model at the operating point
        and saves it for calc_observation"""

        At, Bt = compute_linear_ss(model_type, self.operating_point[1])
        Ct = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0]])
        Dt = np.array([[0, 0],
                      [0, 0],
                      [0, 0]])

        self.Ak, self.Bk, self.Ck, self.Dk = discretize_linear_state_space(At, Bt, Ct, Dt, stepSize)
        return

    def get_output_of_system(self, x):
        """Calculates the output from the current state and the Ck-Matrix + adds gaussian noise"""
        y = self.Ck @ x
        # ToDo: Werte anzeigen
        y[0] += np.random.normal(0, np.sqrt(self.p_noise), 1)[0]
        y[1] += np.random.normal(0, np.sqrt(self.e_noise), 1)[0]
        y[2] += np.random.normal(0, np.sqrt(self.lamb_noise), 1)[0]
        return y

    def calc_observation(self, t, x, u):
        # delete the last two elements
        x = x[0:-2]
        # get output of system from real matrix
        y = self.get_output_of_system(x)

        # 0. Initialization
        x_old = self.estimated_state
        P_old = self.cov_matrix
        # 1. Prediction
        x_predict = self.Ak @ x_old + self.Bk @ u
        P_predict = self.Ak @ P_old @ self.Ak.T + self.process_noise_Q

        # 2. Update
        innovation = y - self.Ck @ x_predict
        innovation_covariance = self.output_covariance_R + self.Ck @ P_predict @ self.Ck.T
        kalman_gain = P_predict @ self.Ck.T @ np.linalg.inv(innovation_covariance)
        x_update = x_predict + kalman_gain @ innovation
        P_update = ((np.eye(6) - kalman_gain @ self.Ck) @ P_predict
                    @ np.transpose(np.eye(6) - kalman_gain @ self.Ck)
                    + kalman_gain @ self.output_covariance_R @ kalman_gain.T)

        self.estimated_state = x_update
        self.cov_matrix = P_update

        x_predict = np.pad(x_predict, (0, 2), "constant")

        return x_predict, [0, 0], y

