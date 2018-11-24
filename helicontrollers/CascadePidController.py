from helicontrollers.AbstractController import *
from helicontrollers.util import PidAlgorithm, ModelType


class CascadePidController(AbstractController):
    def __init__(self):
        self.elevation_pid = PidAlgorithm([10, 0, 5])
        self.travel_pitch_pid = PidAlgorithm([2, 0, 2])
        self.pitch_vd_pid = PidAlgorithm([20, 0, 2])

        super().__init__("Cascade PID", {
            "Elevation PID": ParamFloatArray([0, 0, 0],
                                             [100, 100, 100],
                                             self.elevation_pid.gains),
            "Travel-Pitch PID": ParamFloatArray([0, 0, 0],
                                                [100, 100, 100],
                                                self.travel_pitch_pid.gains),
            "Pitch-Vd PID": ParamFloatArray([0, 0, 0],
                                            [100, 100, 100],
                                            self.pitch_vd_pid.gains)
        })

    def control(self, t, x, e_traj, lambda_traj):
        e_error = x[1] - e_traj[0]
        Vs = - self.elevation_pid.compute(t, e_error, x[4])

        # outer loop: travel --> pitch
        lambda_error = x[2] - lambda_traj[0]
        p_op = - self.travel_pitch_pid.compute(t, lambda_error, x[5])

        # inner loop: pitch --> Vd
        p_error = x[0] - p_op
        Vd = - self.pitch_vd_pid.compute(t, p_error, x[3])

        Vf = (Vs + Vd) / 2
        Vb = (Vs - Vd) / 2

        return [Vf, Vb]

    def initialize(self, param_value_dict):
        self.elevation_pid = PidAlgorithm(param_value_dict["Elevation PID"])
        self.travel_pitch_pid = PidAlgorithm(param_value_dict["Travel-Pitch PID"])
        self.pitch_vd_pid = PidAlgorithm(param_value_dict["Pitch-Vd PID"])
