from helicontrollers.AbstractController import AbstractController, ParamEnum, ParamFloatArray
from helicontrollers.util import ModelType, compute_linear_ss_and_op_output
import control as ctr
import numpy as np


class PolePlacementController(AbstractController):
    def __init__(self):
        self.operating_point = [0, 0]  # travel, elevation
        self.model_type = ModelType.CENTRIPETAL
        self.poles = [-1, -2, -3, -4, -5, -6]
        self.Vf_op = 0
        self.Vb_op = 0
        self.K = np.zeros((2, 6))

        super().__init__("Pole placement", {
            "Model type": ParamEnum(["Simple", "Friction", "Centripetal"],
                                    [ModelType.EASY, ModelType.FRICTION, ModelType.CENTRIPETAL],
                                    self.model_type),
            "Poles": ParamFloatArray([-100, -100, -100, -100, -100, -100],
                                     [-0.01, -0.01, -0.01, -0.01, -0.01, -0.01],
                                     self.poles)
        })

    def control(self, t, x):
        u_op = np.array([self.Vf_op, self.Vb_op])
        x_op = np.array([0, self.operating_point[1], self.operating_point[0], 0, 0, 0])

        u = u_op - self.K @ (x - x_op)

        return u

    def initialize(self, operating_point, param_value_dict, planner_travel, planner_elevation):
        self.operating_point = operating_point
        self.model_type = param_value_dict["Model type"]
        self.poles = param_value_dict["Poles"]

        A, B, self.Vf_op, self.Vb_op = compute_linear_ss_and_op_output(self.model_type, self.operating_point[1])

        try:
            self.K = ctr.place(A, B, self.poles)
        except Exception as e:
            print("Error during pole placement: " + str(e))
