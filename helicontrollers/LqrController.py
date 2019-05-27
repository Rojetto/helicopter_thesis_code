from scipy.linalg import solve_continuous_are

from helicontrollers.AbstractController import AbstractController
from gui.ParameterFrame import ParamEnum, ParamBool, ParamFloatArray
import numpy as np

from helicontrollers.util import ModelType, compute_linear_ss, compute_linear_ss_full, compute_feed_forward_static


class LqrController(AbstractController):
    def __init__(self):
        self.Q = np.diag([2, 10, 4, 0.2, 0.2, 0.1, 1, 1])  # diagonal weighting matrix Q
        self.R = np.diag([0.1, 0.1])  # diagonal weighting matrix R
        self.K = np.zeros((2, 8))
        self.model_type = ModelType.NO_SIMPLIFICATIONS
        self.feedback_computed = False
        self.time_variant_feedback = False

        self.trajectory = None

        super().__init__("LQR", {
            "Model type": ParamEnum(["Simple", "Friction", "Centripetal", "Full"],
                                    [ModelType.EASY, ModelType.FRICTION, ModelType.CENTRIPETAL, ModelType.NO_SIMPLIFICATIONS],
                                    self.model_type),
            "Time variant feedback": ParamBool(self.time_variant_feedback),
            "diag(Q)": ParamFloatArray(np.diag(self.Q)),
            "diag(R)": ParamFloatArray(np.diag(self.R))
        })

    def control(self, t, x):
        traj_eval = self.trajectory.eval(t)

        if self.time_variant_feedback or not self.feedback_computed:
            self.K = self.compute_feedback_matrix(traj_eval.eps[0])
            self.feedback_computed = True

        Vf_op = traj_eval.vf[0]
        Vb_op = traj_eval.vb[0]
        u_op = np.array([Vf_op, Vb_op])
        x_op = np.array([0, traj_eval.eps[0], traj_eval.lamb[0], 0, 0, 0, Vf_op, Vb_op])

        u = u_op - self.K @ (x - x_op)
        return u

    def set_params(self, param_value_dict):
        self.model_type = param_value_dict["Model type"]
        self.Q = np.diag(param_value_dict["diag(Q)"])
        self.R = np.diag(param_value_dict["diag(R)"])
        self.time_variant_feedback = param_value_dict["Time variant feedback"]

    def initialize(self, trajectory):
        self.trajectory = trajectory
        self.feedback_computed = False

    def compute_feedback_matrix(self, e_op):
        if self.model_type == ModelType.NO_SIMPLIFICATIONS:
            Vf_op, Vb_op = compute_feed_forward_static([e_op, 0, 0, 0, 0], [0, 0, 0, 0, 0])
            u_op = np.array([Vf_op, Vb_op])
            x_op = np.array([0, e_op, 0, 0, 0, 0, Vf_op, Vb_op])
            A, B = compute_linear_ss_full(x_op, u_op)

            try:
                X = solve_continuous_are(A, B, self.Q, self.R)
                K = np.linalg.inv(self.R) @ B.T @ X

                return K
            except Exception as e:
                print("Error during LQR computation: " + str(e))
                return np.zeros((2, 8))
        else:
            A_reduced, B_reduced = compute_linear_ss(self.model_type, e_op)

            try:
                Q_reduced = self.Q[:6, :6]
                R_reduced = self.R[:6, :]
                X = solve_continuous_are(A_reduced, B_reduced, Q_reduced, R_reduced)
                K_reduced = np.linalg.inv(R_reduced) @ B_reduced.T @ X
                K = np.zeros((2, 8))
                K[:, :6] = K_reduced

                return K
            except Exception as e:
                print("Error during LQR computation: " + str(e))
                return np.zeros((2, 8))
