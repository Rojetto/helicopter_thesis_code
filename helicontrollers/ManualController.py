from helicontrollers.AbstractController import *


class ManualController(AbstractController):
    def __init__(self):
        self.Vf = 0.25
        self.Vb = 0.25

        super().__init__("Manual control", {
            "Vf": ParamFloatArray([-10], [10], [self.Vf]),
            "Vb": ParamFloatArray([-10], [10], [self.Vb])
        })

    def control(self, t, x):
        return [self.Vf, self.Vb]

    def initialize(self, operating_point, param_value_dict, planner_travel, planner_elevation):
        self.Vf = param_value_dict["Vf"][0]
        self.Vb = param_value_dict["Vb"][0]
