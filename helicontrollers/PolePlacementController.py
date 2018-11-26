from helicontrollers.AbstractController import AbstractController, ParamEnum, ParamBool, ParamFloatArray
from helicontrollers.util import ModelType, compute_linear_ss
import control as ctr
import numpy as np


class PolePlacementController(AbstractController):
    def __init__(self):
        self.model_type = ModelType.CENTRIPETAL
        self.poles = [-1, -2, -3, -4, -5, -6]
        self.K = np.zeros((2, 6))
        self.feedback_computed = False
        self.time_variant_feedback = False

        super().__init__("Pole placement", {
            "Model type": ParamEnum(["Simple", "Friction", "Centripetal"],
                                    [ModelType.EASY, ModelType.FRICTION, ModelType.CENTRIPETAL],
                                    self.model_type),
            "Time variant feedback": ParamBool(self.time_variant_feedback),
            "Poles": ParamFloatArray([-100, -100, -100, -100, -100, -100],
                                     [-0.01, -0.01, -0.01, -0.01, -0.01, -0.01],
                                     self.poles)
        })

    def control(self, t, x, e_traj, lambda_traj):
        # delete front and back speed because we dont need it here
        x = x[:6]
        if self.time_variant_feedback or not self.feedback_computed:
            self.K = self.compute_feedback_matrix(e_traj[0])
            self.feedback_computed = True

        # TODO: We could potentially also compute the trajectory for the pitch, should we do that? This would make the
        #       linearization a whole lot more complex though. Same for the derivatives of all states.
        x_op = np.array([0, e_traj[0], lambda_traj[0], 0, 0, 0])

        u = - self.K @ (x - x_op)
        return u

    def initialize(self, param_value_dict):
        self.model_type = param_value_dict["Model type"]
        self.poles = param_value_dict["Poles"]
        self.time_variant_feedback = param_value_dict["Time variant feedback"]
        self.feedback_computed = False

    def compute_feedback_matrix(self, e_op):
        A, B = compute_linear_ss(self.model_type, e_op)

        try:
            K = ctr.place(A, B, self.poles)
            return K
        except Exception as e:
            print("Error during pole placement: " + str(e))
