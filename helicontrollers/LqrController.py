from scipy.linalg import solve_continuous_are

from helicontrollers.AbstractController import *
import numpy as np

from helicontrollers.util import ModelType, compute_linear_ss_and_op_output


class LqrController(AbstractController):
    def __init__(self):
        self.Q_diag = [1, 1, 1, 1, 1, 1]  # diagonal of weighting matrix Q
        self.R_diag = [1, 1]  # diagonal of weighting matrix R
        self.K = np.zeros((2, 6))
        self.Vf_op = 0  # front rotor voltage to stay in equilibrium at the operating point
        self.Vb_op = 0  # back rotor voltage to stay in equilibrium at the operating point
        self.operating_point = [0, 0]
        self.model_type = ModelType.CENTRIPETAL

        super().__init__("LQR", {
            "Model type": ParamEnum(["Simple", "Friction", "Centripetal"],
                                    [ModelType.EASY, ModelType.FRICTION, ModelType.CENTRIPETAL],
                                    self.model_type),
            "diag(Q)": ParamFloatArray([0, 0, 0, 0, 0, 0],
                                       [100, 100, 100, 100, 100, 100],
                                       self.Q_diag),
            "diag(R)": ParamFloatArray([0, 0],
                                       [100, 100],
                                       self.R_diag)
        })

    def control(self, t, x):
        u_op = np.array([self.Vf_op, self.Vb_op])
        x_op = np.array([0, self.operating_point[1], self.operating_point[0], 0, 0, 0])

        u = u_op - self.K @ (x - x_op)
        return u

    def initialize(self, operating_point, param_value_dict, planner_travel, planner_elevation):
        self.operating_point = operating_point
        self.model_type = param_value_dict["Model type"]
        self.Q_diag = param_value_dict["diag(Q)"]
        self.R_diag = param_value_dict["diag(R)"]
        Q = np.diag(self.Q_diag)
        R = np.diag(self.R_diag)

        A, B, self.Vf_op, self.Vb_op = compute_linear_ss_and_op_output(self.model_type, self.operating_point[1])

        try:
            X = solve_continuous_are(A, B, Q, R)
            self.K = np.linalg.inv(R) @ B.T @ X
        except Exception as e:
            print("Error during LQR computation: " + str(e))
