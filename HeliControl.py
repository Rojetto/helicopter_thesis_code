import numpy as np
from numpy.ma import cos, sin
import control as ctr

from enum import Enum

import ModelConstants as mc


class ControlMethod(Enum):
    POLES = 1
    LQR = 2
    PID = 3


class HeliControl(object):
    def __init__(self):
        self.operatingPoint = np.array([0, 0, 0])
        self.control_method = ControlMethod.POLES
        self.feedback_poles = [-1, -2, -3, -4, -5, -6]
        self.Vf_op = 0
        self.Vb_op = 0
        self.state_feedback_gain = np.zeros((2, 6))

        self.update_stabilizing_state_feedback()

    def update_stabilizing_state_feedback(self):
        e_op = self.operatingPoint[1]

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

        try:
            K = ctr.place(A, B, self.feedback_poles)
            self.state_feedback_gain = K
        except:
            print("Error during pole placement")

    def control(self, t, x):
        """Is called by the main loop in order to get the current controller output.
            Args:
                t: current simulation time
                x: current system state. x = [p, e, lambda, dp/dt, de/dt, dlambda/dt]
            Returns:
                u: control output. u = [Vf, Vb]"""
        u_op = np.array([self.Vf_op, self.Vb_op])
        x_op = np.array([self.operatingPoint[0], self.operatingPoint[1], self.operatingPoint[2], 0, 0, 0])

        u = np.zeros(2)

        if self.control_method == ControlMethod.POLES:
            u = u_op - self.state_feedback_gain @ (x - x_op)

        return u

    def setControlMethod(self, method):
        self.control_method = method

    def setOperatingPoint(self, point):
        """This function is called in order to set the current operating point of the controller.
        point: list of coordinates
        point[0]: Pitch
        point[1]: Elevation
        point[2]: Lambda """
        self.operatingPoint = np.array(point)
        self.update_stabilizing_state_feedback()

    def setFeedbackPoles(self, poles):
        self.feedback_poles = poles
        self.update_stabilizing_state_feedback()
