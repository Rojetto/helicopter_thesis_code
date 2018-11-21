from helicontrollers.AbstractController import *
from helicontrollers.util import PidAlgorithm, compute_linear_ss_and_op_output, ModelType


class DirectPidController(AbstractController):
    def __init__(self):
        self.elevation_pid = PidAlgorithm([10, 0, 5])
        self.travel_pid = PidAlgorithm([1, 0, 1])
        self.Vf_op = 0
        self.Vb_op = 0
        self.operating_point = [0, 0]

        super().__init__("Direct PID", {
            "Elevation PID": ParamFloatArray([0, 0, 0],
                                             [100, 100, 100],
                                             self.elevation_pid.gains),
            "Travel PID": ParamFloatArray([0, 0, 0],
                                          [100, 100, 100],
                                          self.travel_pid.gains)
        })

    def control(self, t, x):
        e_error = x[1] - self.operating_point[1]
        Vs_diff = - self.elevation_pid.compute(t, e_error, x[4])

        lambda_error = x[2] - self.operating_point[0]
        Vd_diff = - self.travel_pid.compute(t, lambda_error, x[5])

        Vf_diff = (Vs_diff + Vd_diff) / 2
        Vb_diff = (Vs_diff - Vd_diff) / 2

        return [self.Vf_op + Vf_diff, self.Vb_op + Vb_diff]

    def initialize(self, operating_point, param_value_dict, planner_travel, planner_elevation):
        # We only need the operating point voltages for the rotors
        _, _, self.Vf_op, self.Vb_op = compute_linear_ss_and_op_output(ModelType.EASY, operating_point[1])
        self.elevation_pid = PidAlgorithm(param_value_dict["Elevation PID"])
        self.travel_pid = PidAlgorithm(param_value_dict["Travel PID"])
        self.operating_point = operating_point
