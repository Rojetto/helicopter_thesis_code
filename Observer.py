from abc import ABC
from enum import Enum

import numpy as np
import abc # abstract base class
import math
import scipy as sp
from scipy import special


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

    def calc_observation(self, t, x, u):
        # print("Kalman Filter ist called!")

        return np.zeros(6)

