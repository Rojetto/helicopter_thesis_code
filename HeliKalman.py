import numpy as np
import ModelConstants as mc

class HeliKalmanFilter(object):
    def __init__(self):
        pass

    def kalman_compute(self, t, x, u):
        """Estimates the system state dependent on the output of the system.
            Args:
                t: current simulation time
                x: current system state. x = [p, e, lambda, dp/dt, de/dt, dlambda/dt]
                u: current controller output
            Returns:
                None"""
        return None
