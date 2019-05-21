from helicontrollers.AbstractController import AbstractController
from helicontrollers.util import PidAlgorithm
from ParameterFrame import ParamFloatArray


class DirectPidController(AbstractController):
    def __init__(self):
        self.elevation_pid = PidAlgorithm([10, 0, 5])
        self.travel_pid = PidAlgorithm([1, 0, 1])

        super().__init__("Direct PID", {
            "Elevation PID": ParamFloatArray(self.elevation_pid.gains),
            "Travel PID": ParamFloatArray(self.travel_pid.gains)
        })

    def control(self, t, x, e_traj, lambda_traj):
        e_error = x[1] - e_traj[0]
        Vs = - self.elevation_pid.compute(t, e_error, x[4])

        lambda_error = x[2] - lambda_traj[0]
        Vd = - self.travel_pid.compute(t, lambda_error, x[5])

        Vf = (Vs + Vd) / 2
        Vb = (Vs - Vd) / 2

        return [Vf, Vb]

    def initialize(self, param_value_dict):
        self.elevation_pid = PidAlgorithm(param_value_dict["Elevation PID"])
        self.travel_pid = PidAlgorithm(param_value_dict["Travel PID"])
