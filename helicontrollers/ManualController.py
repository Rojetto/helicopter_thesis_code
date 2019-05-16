from helicontrollers.AbstractController import *


class ManualController(AbstractController):
    def __init__(self):
        self.Vf = 0.25
        self.Vb = 0.25

        super().__init__("Manual control", {
            "Vf": ParamFloatArray([self.Vf]),
            "Vb": ParamFloatArray([self.Vb])
        })

    def control(self, t, x, e_traj, lambda_traj):
        return [self.Vf, self.Vb]

    def initialize(self, param_value_dict):
        self.Vf = param_value_dict["Vf"][0]
        self.Vb = param_value_dict["Vb"][0]
