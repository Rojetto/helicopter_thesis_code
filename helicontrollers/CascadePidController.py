from helicontrollers.AbstractController import *
from helicontrollers.util import PidAlgorithm, compute_linear_ss_and_op_output, ModelType


class CascadePidController(AbstractController):
    def __init__(self):
        self.elevation_pid = PidAlgorithm([10, 0, 5])
        self.travel_pitch_pid = PidAlgorithm([2, 0, 2])
        self.pitch_vd_pid = PidAlgorithm([20, 0, 2])
        self.Vf_op = 0
        self.Vb_op = 0
        self.operating_point = [0, 0]

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

    def control(self, t, x):
        e_error = x[1] - self.operating_point[1]
        Vs_diff = - self.elevation_pid.compute(t, e_error, x[4])

        # outer loop: travel --> pitch
        lambda_error = x[2] - self.operating_point[0]
        p_op = - self.travel_pitch_pid.compute(t, lambda_error, x[5])

        # inner loop: pitch --> Vd
        p_error = x[0] - p_op
        Vd_diff = - self.pitch_vd_pid.compute(t, p_error, x[3])

        Vf_diff = (Vs_diff + Vd_diff) / 2
        Vb_diff = (Vs_diff - Vd_diff) / 2

        return [self.Vf_op + Vf_diff, self.Vb_op + Vb_diff]

    def initialize(self, operating_point, param_value_dict, planner_travel, planner_elevation):
        # We only need the operating point voltages for the rotors
        _, _, self.Vf_op, self.Vb_op = compute_linear_ss_and_op_output(ModelType.EASY, operating_point[1])
        self.elevation_pid = PidAlgorithm(param_value_dict["Elevation PID"])
        self.travel_pitch_pid = PidAlgorithm(param_value_dict["Travel-Pitch PID"])
        self.pitch_vd_pid = PidAlgorithm(param_value_dict["Pitch-Vd PID"])
        self.operating_point = operating_point
