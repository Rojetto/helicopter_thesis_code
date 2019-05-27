from helicontrollers.AbstractController import AbstractController
from gui.ParameterFrame import ParamFloatArray


class ManualController(AbstractController):
    def __init__(self):
        self.Vf = 0.25
        self.Vb = 0.25

        super().__init__("Manual control", {
            "Vf": ParamFloatArray([self.Vf]),
            "Vb": ParamFloatArray([self.Vb])
        })

    def control(self, t, x):
        return [self.Vf, self.Vb]

    def set_params(self, param_value_dict):
        self.Vf = param_value_dict["Vf"][0]
        self.Vb = param_value_dict["Vb"][0]

    def initialize(self, trajectory):
        pass
