import numpy as np
from numpy.ma import cos, sin
from scipy.linalg import solve_continuous_are
import control as ctr
from enum import Enum

import ModelConstants as mc


class ControlMethod(Enum):
    POLES = 1
    LQR = 2
    PID_DIRECT = 3
    PID_CASCADE = 4

class ModelType(Enum):
    EASY = 1
    FRICTION = 2
    CENTRIPETAL = 3


class HeliControl(object):
    def __init__(self):
        self.operatingPoint = np.array([0, 0, 0])  # pitch, elevation, travel
        self.control_method = ControlMethod.POLES
        self.model_type = ModelType.CENTRIPETAL
        self.feedback_poles = [-1, -2, -3, -4, -5, -6]  # closed loop poles to be used for state feedback
        self.lqr_Q = [1, 1, 1, 1, 1, 1]  # diagonal of weighting matrix Q
        self.lqr_R = [1, 1]  # diagonal of weighting matrix R
        self.pid_elevation_gains = [10, 0, 5]  # Kp, Ki and Kd for elevation PID controller
        self.pid_travel_gains = [1, 0.1, 0.01]  # Kp, Ki and Kd for direct travel PID controller
        self.pid_travel_pitch_gains = [2, 0, 2]  # Kp, Ki and Kd for outer travel-pitch PID controller
        self.pid_pitch_vd_gains = [20, 0, 2]  # Kp, Ki and Kd for inner pitch-vd PID controller
        self.Vf_op = 0  # front rotor voltage to stay in equilibrium at the operating point
        self.Vb_op = 0  # back rotor voltage to stay in equilibrium at the operating point
        self.state_feedback_gain = np.zeros((2, 6))  # state feedback matrix, determined by pole placement
        self.lqr_gain = np.zeros((2, 6))  # state feedback matrix, determined by LQR design

        self.last_state = np.zeros(6)  # system state on last invocation of "control", used for PID
        self.last_t = 0  # system time on last invocation of "control", used for PID
        self.e_error_int = 0
        self.l_error_direct_int = 0
        self.l_error_cascade_int = 0
        self.p_error_cascade_int = 0

        self.update_controller_algorithms()

    def update_controller_algorithms(self):
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

        if self.model_type == ModelType.EASY:
            A = np.array([[0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1],
                          [0, 0, 0, 0, 0, 0],
                          [0, -L2*sin(e_op)/Je, 0, 0, 0, 0],
                          [L4*Vs_op*cos(e_op)/Jl, 0, 0, 0, 0, 0]])

        if self.model_type == ModelType.FRICTION or self.model_type == ModelType.CENTRIPETAL:
            A = np.array([[0, 0,                  0,             1, 0, 0],
                          [0, 0,                  0,             0, 1, 0],
                          [0, 0,                  0,             0, 0, 1],
                          [0, 0,                  0, - mc.d_p / Jp, 0, 0],
                          [0, -L2*sin(e_op)/Je,   0, 0,  - mc.d_e / Je, 0],
                          [L4*Vs_op*cos(e_op)/Jl, 0, 0, 0, 0,  - mc.d_l / Jl]])

        B = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [L1/Jp, -L1/Jp],
                      [L3/Je, L3/Je],
                      [0, 0]])

        Q = np.diag(self.lqr_Q)
        R = np.diag(self.lqr_R)

        try:
            K = ctr.place(A, B, self.feedback_poles)
            self.state_feedback_gain = K
        except Exception as e:
            print("Error during pole placement: " + str(e))

        try:
            X = solve_continuous_are(A, B, Q, R)
            K = np.linalg.inv(R) @ B.T @ X
            self.lqr_gain = K
        except Exception as e:
            print("Error during LQR computation: " + str(e))

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
        elif self.control_method == ControlMethod.LQR:
            u = u_op - self.lqr_gain @ (x - x_op)
        elif self.control_method == ControlMethod.PID_DIRECT or self.control_method == ControlMethod.PID_CASCADE:
            if not(np.linalg.norm(x - self.last_state) > 0.1 or t - self.last_t > 0.1):
                # only calculate pid outputs when we didn't just jump by setting the state or switching the mode
                e_error = x[1] - x_op[1]
                self.e_error_int += e_error * (t - self.last_t)

                Vs_diff = -(self.pid_elevation_gains[0] * e_error + self.pid_elevation_gains[1] * self.e_error_int + self.pid_elevation_gains[2] * x[4])

                if self.control_method == ControlMethod.PID_DIRECT:
                    l_error = x[2] - x_op[2]
                    self.l_error_direct_int += l_error * (t - self.last_t)
                    Vd_diff = -(self.pid_travel_gains[0] * l_error + self.pid_travel_gains[1] * self.l_error_direct_int + self.pid_travel_gains[2] * x[5])
                else:
                    # outer loop: travel --> pitch
                    l_error = x[2] - x_op[2]
                    self.l_error_cascade_int += l_error * (t - self.last_t)
                    p_op = -(self.pid_travel_pitch_gains[0] * l_error + self.pid_travel_pitch_gains[1] * self.l_error_cascade_int + self.pid_travel_pitch_gains[2] * x[5])

                    # inner loop: pitch --> Vd
                    p_error = x[0] - p_op
                    self.p_error_cascade_int += p_error * (t - self.last_t)
                    Vd_diff = -(self.pid_pitch_vd_gains[0] * p_error + self.pid_pitch_vd_gains[1] * self.p_error_cascade_int + self.pid_pitch_vd_gains[2] * x[3])

                Vf_diff = (Vs_diff + Vd_diff) / 2
                Vb_diff = (Vs_diff - Vd_diff) / 2

                u = u_op + np.array([Vf_diff, Vb_diff])
            else:
                self.e_error_int = 0
                self.l_error_direct_int = 0
                self.l_error_cascade_int = 0
                self.p_error_cascade_int = 0

        self.last_t = t
        self.last_state = x

        return u

    def setControlMethod(self, method):
        self.control_method = method

    def setModelType(self, modelType):
        self.model_type = modelType

    def setOperatingPoint(self, point):
        """This function is called in order to set the current operating point of the controller.
        point: list of coordinates
        point[0]: Pitch
        point[1]: Elevation
        point[2]: Lambda """
        self.operatingPoint = np.array(point)
        self.update_controller_algorithms()

    def setFeedbackPoles(self, poles):
        self.feedback_poles = poles
        self.update_controller_algorithms()

    def setLqrQDiagonal(self, lqr_Q):
        self.lqr_Q = lqr_Q
        self.update_controller_algorithms()

    def setLqrRDiagonal(self, lqr_R):
        self.lqr_R = lqr_R
        self.update_controller_algorithms()

    def setElevationPidGains(self, gains):
        self.pid_elevation_gains = gains

    def setTravelPidGains(self, gains):
        self.pid_travel_gains = gains

    def setTravelPitchPidGains(self, gains):
        self.pid_travel_pitch_gains = gains

    def setPitchVdPidGains(self, gains):
        self.pid_pitch_vd_gains = gains
