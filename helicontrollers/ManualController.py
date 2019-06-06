from helicontrollers.AbstractController import AbstractController
from gui.ParameterFrame import ParamFloatArray, ParamBool


class ManualController(AbstractController):
    def __init__(self):
        self.Vf = 0.0
        self.Vb = 0.0
        self.use_feed_forward = True

        self.trajectory = None

        super().__init__("Manual control", {
            "Vf": ParamFloatArray([self.Vf]),
            "Vb": ParamFloatArray([self.Vb]),
            "Feed forward": ParamBool(self.use_feed_forward)
        })

    def control(self, t, x):
        traj_eval = self.trajectory.eval(t)

        uf = self.Vf
        ub = self.Vb

        if self.use_feed_forward:
            uf += traj_eval.vf[0]
            ub += traj_eval.vb[0]

        return [uf, ub]

    def set_params(self, param_value_dict):
        self.Vf = param_value_dict["Vf"][0]
        self.Vb = param_value_dict["Vb"][0]
        self.use_feed_forward = param_value_dict["Feed forward"]

    def initialize(self, trajectory):
        self.trajectory = trajectory
