from helicontrollers.AbstractController import AbstractController
from helicontrollers.util import PidAlgorithm, ModelType, compute_feed_forward_flatness
from ParameterFrame import ParamFloatArray, ParamBool


class CascadePidController(AbstractController):
    def __init__(self):
        self.elevation_pid = PidAlgorithm([10, 0, 5])
        self.travel_pitch_pid = PidAlgorithm([1.5, 0, 1.5])
        self.pitch_vd_pid = PidAlgorithm([20, 0, 2.3])
        self.k_rotor = [5, 0]

        self.front_rotor_pid = None
        self.back_rotor_pid = None
        self.rotor_speed_control = True

        self.trajectory = None

        super().__init__("Cascade PID", {
            "Elevation PID": ParamFloatArray(self.elevation_pid.gains),
            "Travel-Pitch PID": ParamFloatArray(self.travel_pitch_pid.gains),
            "Pitch-Vd PID": ParamFloatArray(self.pitch_vd_pid.gains),
            "Rotor PD + internal FF": ParamBool(self.rotor_speed_control),
            "Rotor PD": ParamFloatArray(self.k_rotor)
        })

    def control(self, t, x):
        traj_eval = self.trajectory.eval(t)
        eps_traj = traj_eval.eps
        lamb_traj = traj_eval.lamb
        wf_d_ff = traj_eval.vf[0]
        wb_d_ff = traj_eval.vb[0]

        e_error = x[1] - eps_traj[0]
        delta_ws_d = - self.elevation_pid.compute(t, e_error, x[4])

        # outer loop: travel --> pitch
        lambda_error = x[2] - lamb_traj[0]
        p_op = - self.travel_pitch_pid.compute(t, lambda_error, x[5])

        # inner loop: pitch --> Vd
        p_error = x[0] - p_op
        delta_wd_d = - self.pitch_vd_pid.compute(t, p_error, x[3])

        delta_wf_d = (delta_ws_d + delta_wd_d) / 2
        delta_wb_d = (delta_ws_d - delta_wd_d) / 2

        wf_d = wf_d_ff + delta_wf_d
        wb_d = wb_d_ff + delta_wb_d

        # Rotor speed control
        if self.rotor_speed_control:
            Vf = wf_d + self.front_rotor_pid.compute(t, wf_d - x[6])
            Vb = wb_d + self.back_rotor_pid.compute(t, wb_d - x[7])
        else:
            Vf = wf_d
            Vb = wb_d

        return [Vf, Vb]

    def initialize(self, trajectory):
        self.trajectory = trajectory
        self.front_rotor_pid = PidAlgorithm([self.k_rotor[0], 0, self.k_rotor[1]])
        self.back_rotor_pid = PidAlgorithm([self.k_rotor[0], 0, self.k_rotor[1]])

    def set_params(self, param_value_dict):
        self.elevation_pid = PidAlgorithm(param_value_dict["Elevation PID"])
        self.travel_pitch_pid = PidAlgorithm(param_value_dict["Travel-Pitch PID"])
        self.pitch_vd_pid = PidAlgorithm(param_value_dict["Pitch-Vd PID"])
        self.rotor_speed_control = param_value_dict["Rotor PD + internal FF"]
        self.k_rotor = param_value_dict["Rotor PD"]
