import numpy as np
from numpy.ma import cos, sin
import control as ctr

import ModelConstants as mc


class HeliControl(object):
    def __init__(self):
        self.operatingPoint = np.array([0, 0, 0])
        self.Vf_op = 0
        self.Vb_op = 0
        self.state_feedback_gain = self.compute_stabilizing_state_feedback(0, 0)

    def compute_stabilizing_state_feedback(self, lambda_op, e_op):
        L1 = mc.l_p
        L2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
        L3 = mc.l_h
        L4 = mc.l_h

        Jp = 2 * mc.m_p * mc.l_p**2
        Je = mc.m_c * mc.l_c**2 + 2 * mc.m_p * mc.l_h**2
        Jl = mc.m_c * mc.l_c**2 + 2 * mc.m_p * (mc.l_h ** 2 + mc.l_p**2)

        Vs_op = - L2/L3 * cos(e_op)
        Vd_op = 0

        self.Vf_op = (Vs_op + Vd_op) / 2
        self.Vb_op = (Vs_op - Vd_op) / 2

        A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, 0, 0, 0, 0],
                      [0, -L2*sin(e_op)/Je, 0, 0, 0, 0],
                      [L4*Vs_op*cos(e_op)/Jl, 0, 0, 0, 0, 0]])

        B = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [L1/Jp, -L1/Jp],
                      [L3/Je, L3/Je],
                      [0, 0]])

        K = ctr.place(A, B, [-1, -2, -3, -4, -5, -6])
        print(K)

        return K

    def control(self, t, x):
        """Is called by the main loop in order to get the current controller output.
            Args:
                t: current simulation time
                x: current system state. x = [p, e, lambda, dp/dt, de/dt, dlambda/dt]
            Returns:
                u: control output. u = [Vf, Vb]"""
        u_op = np.array([self.Vf_op, self.Vb_op])
        x_op = np.array([self.operatingPoint[0], self.operatingPoint[1], self.operatingPoint[2], 0, 0, 0])
        return u_op - self.state_feedback_gain @ (x - x_op)

    def setOperatingPoint(self, point):
        """This function is called in order to set the current operating point of the controller.
        point: list of coordinates
        point[0]: Pitch
        point[1]: Elevation
        point[2]: Lambda """
        self.operatingPoint = np.array(point)
        self.state_feedback_gain = self.compute_stabilizing_state_feedback(point[2], point[1])
        print("Operating Point was set to " + str(self.operatingPoint))
