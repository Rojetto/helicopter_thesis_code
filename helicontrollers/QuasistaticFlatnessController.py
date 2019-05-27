from helicontrollers.AbstractController import AbstractController
from helicontrollers.util import compute_feed_forward_flatness_simple
from gui.ParameterFrame import ParamFloatArray
from control import sample_system, place, ss, ssdata
import numpy as np


class QuasistaticFlatnessController(AbstractController):
    def __init__(self):
        self.k1 = [1, 4, 6, 4]
        self.k2 = [1, 2]
        self.luenberger_poles = [-6, -5, -4, -3, -2, -1]
        self.L = np.zeros((6, 4))
        self.luenberger_Ad = np.zeros((6, 6))
        self.luenberger_Bd = np.zeros((6, 6))
        self.luenberger_state = np.zeros(6)
        self.run_once = False

        super().__init__("Quasistatic Flatness Controller", {
            "k1": ParamFloatArray(self.k1),
            "k2": ParamFloatArray(self.k2),
            "Luenberger poles": ParamFloatArray(self.luenberger_poles)
        })

    def control(self, t, x, e_traj, lambda_traj):
        e = x[1]
        l = x[2]
        de1 = x[4]
        dl1 = x[5]
        m = np.array([e, de1, l, dl1])

        if not self.run_once:
            self.luenberger_state = np.array([e, de1, 0, 0, l, dl1])
            self.run_once = True

        v1 = e_traj[4] - self.k1[3]*(self.luenberger_state[3] - e_traj[3])\
             - self.k1[2]*(self.luenberger_state[2] - e_traj[2])\
             - self.k1[1]*(de1 - e_traj[1])\
             - self.k1[0]*(e - e_traj[0])

        v2 = lambda_traj[2] - self.k2[1]*(dl1 - lambda_traj[1]) - self.k2[0]*(l - lambda_traj[0])
        v = np.array([v1, v2])

        dv21 = lambda_traj[3] - self.k2[1]*(v2 - lambda_traj[2]) - self.k2[0]*(dl1 - lambda_traj[1])
        dv22 = lambda_traj[4] - self.k2[1]*(dv21 - lambda_traj[3]) - self.k2[0]*(v2 - lambda_traj[2])

        y1_and_derivatives = np.array([e, de1, self.luenberger_state[2], self.luenberger_state[3], v1])
        y2_and_derivatives = np.array([l, dl1, v2, dv21, dv22])
        Vf, Vb = compute_feed_forward_flatness_simple(y1_and_derivatives, y2_and_derivatives)

        self.luenberger_state = self.luenberger_Ad @ self.luenberger_state + self.luenberger_Bd @ np.concatenate((v, m))
        # The previous matrix multiplications resulted in a matrix object, so we need to flatten to a vector
        self.luenberger_state = self.luenberger_state.A1

        return Vf, Vb

    def initialize(self, param_value_dict):
        self.k1 = param_value_dict["k1"]
        self.k2 = param_value_dict["k2"]
        self.luenberger_poles = param_value_dict["Luenberger poles"]

        AB = np.array([[0, 1, 0, 0, 0, 0],
                       [0, 0, 1, 0, 0, 0],
                       [0, 0, 0, 1, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 1],
                       [0, 0, 0, 0, 0, 0]])

        BB = np.array([[0, 0],
                       [0, 0],
                       [0, 0],
                       [1, 0],
                       [0, 0],
                       [0, 1]])

        CB = np.array([[1, 0, 0, 0, 0, 0],
                       [0, 1, 0, 0, 0, 0],
                       [0, 0, 0, 0, 1, 0],
                       [0, 0, 0, 0, 0, 1]])

        L = place(AB.T, CB.T, self.luenberger_poles).T

        cont_observer_system = ss(AB - L @ CB, np.hstack((BB, L)), np.zeros((1, 6)), np.zeros((1, 6)))
        # TODO: Remove hardcoded sample time
        discrete_observer_system = sample_system(cont_observer_system, 1/60)
        self.luenberger_Ad, self.luenberger_Bd, _, _ = ssdata(discrete_observer_system)
        self.run_once = False
