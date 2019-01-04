from abc import ABC
from enum import Enum

import numpy as np
import abc # abstract base class
import math
from ModelConstants import ModelType
from helicontrollers.util import compute_linear_ss
import scipy.linalg
import logger


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
        self.estimated_state = init_state

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


class KalmanFilterBase(Observer):
    """Base class for Kalman Filter. Takes care of adding noise to input and output signals.
    Uses the variable naming conventions of the lecture 'Steuerung mobiler Roboter' of Prof. Janschek"""

    def __init__(self, init_state, init_cov_matrix):
        super().__init__(init_state)
        self.cov_matrix = init_cov_matrix

        # set system noise parameters
        vf_var = (1 / 180 * np.pi) ** 2
        vb_var = (3 / 180 * np.pi) ** 2
        p_var = (1 / 180 * np.pi) ** 2
        e_var = (3 / 180 * np.pi) ** 2
        lamb_var = (5 / 180 * np.pi) ** 2
        # N is the covariance matrix of the input signals. 2 inputs ==> N is a 2x2 matrix
        # Assuming white noise
        self.N = np.diag([vf_var, vb_var])
        # W is the covariance matrix of the output signals. 3 outputs ==> W is a 3x3 matrix
        self.W = np.diag([p_var, e_var, lamb_var])
        # Assuming NO PROCESS NOISE
        # these variables contain the variance of the noise
        return

    def get_noisy_output_of_system(self, y_without_noise):
        """Adds gaussian (white) noise to the output signals
        :arg y_without_noise: n-d-array with 3 elements (p, e, lambda). Dimension: 3x1"""
        y_with_noise = np.zeros((3, 1))
        y_with_noise[0][0] = y_without_noise[0][0] + np.random.normal(0, np.sqrt(self.W[0][0]), 1)[0]
        y_with_noise[1][0] = y_without_noise[1][0] + np.random.normal(0, np.sqrt(self.W[1][1]), 1)[0]
        y_with_noise[2][0] = y_without_noise[2][0] + np.random.normal(0, np.sqrt(self.W[2][2]), 1)[0]
        return y_with_noise

    def get_noisy_input_of_system(self, u_without_noise):
        """Adds gaussian (white) noise to the input signals
        :arg u_without_noise: n-d-array with 2 elements (Vf, Vb). Dimension: 2x1"""
        u_with_noise = np.zeros((2, 1))
        u_with_noise[0][0] = u_without_noise[0][0] + np.random.normal(0, np.sqrt(self.N[0][0]), 1)[0]
        u_with_noise[1][0] = u_without_noise[1][0] + np.random.normal(0, np.sqrt(self.N[1][1]), 1)[0]
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
        u_with_noise = self.get_noisy_input_of_system(np.resize(np.array(u), (2, 1)))
        y_with_noise = self.get_noisy_output_of_system(np.resize(np.array(x[0:3]), (3, 1)))
        return x_estimated, np.resize(u_with_noise, (1, 2)), np.resize(y_with_noise, (1, 3))

    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        return



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
        Dt = self.Dk

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

