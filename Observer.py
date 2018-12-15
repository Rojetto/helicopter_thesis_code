from abc import ABC
from enum import Enum

import numpy as np
import abc # abstract base class
import math
from ModelConstants import ModelType
from helicontrollers.util import compute_linear_ss
import scipy.linalg


def discretize_linear_state_space(At, Bt, Ct, Dt, T):
    """Calculates the discrete state space matrices Ak, Bk, Ck and Dk"""
    n = At.shape[0] # it is assumed that At is quadratic
    Ak = scipy.linalg.expm(At * T)
    Bk = np.inv(At) * (Ak - np.eye(n)) * Bt
    Ck = Ct
    Dk = Dt
    return Ak, Bk, Ck, Dk



class Observer(object):
    """Base class for observer"""

    def __init__(self, init_state):
        self.init_state = init_state

    @abc.abstractmethod
    def calc_observation(self, t, x, u):
        """Estimates the system state dependent on the output of the system.
                    Args:
                        t: current simulation time
                        x: current system state. x = [p, e, lambda, dp/dt, de/dt, dlambda/dt]
                        u: current controller output
                    Returns:
                        x_hat: estimated state of system"""
        return


class LinearKalmanFilter(Observer):
    """Linear Kalman Filter with static operating point. This class linearizes the current system at the
    given operating point."""

    def __init__(self, init_state, init_cov_matrix, operating_point):
        super().__init__(init_state)
        self.init_cov_matrix = init_cov_matrix
        self.operating_point = operating_point
        self.Ak = np.zeros((6, 6))
        self.Bk = np.zeros((6, 2))
        self.Ck = np.zeros(6).reshape(1, 6)
        self.Dk = np.zeros(2).reshape(1, 2)

    def set_system_model_and_step_size(self, model_type: ModelType, stepSize):
        """Linearizes and discretizes the current system model at the operating point
        and saves it for calc_observation"""

        At, Bt = compute_linear_ss(model_type, self.operating_point[1])
        Ct = np.array([1, 1, 1, 0, 0, 0]).reshape((1, 6))
        Dt = np.zeros(2).reshape(1, 2)

        self.Ak, self.Bk, self.Ck, self.Dk = discretize_linear_state_space(At, Bt, Ct, Dt, stepSize)
        return

    def calc_observation(self, t, x, u):
        # print("Kalman Filter is called!")
        return np.zeros(6)

