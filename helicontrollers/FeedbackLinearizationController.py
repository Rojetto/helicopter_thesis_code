from helicontrollers.AbstractController import AbstractController, ParamBool, ParamFloatArray
from helicontrollers.util import L1, L2, L3, L4, Je, Jl, Jp, compute_pitch_flatness_simple
from ModelConstants import d_l as mul, d_e as mue, d_p as mup
from numpy import sin, cos, arctan, sqrt, sign
from helicontrollers.util import PidAlgorithm


class FeedbackLinearizationController(AbstractController):
    def __init__(self):
        self.pitch_derivative_feed_forward = False
        self.friction_centripetal = True
        self.rotor_speed_controller = True
        self.ke = [20, 6]
        self.kl = [4, 3]
        self.kp = [150, 20]
        self.k_rotor = [5, 0]

        self.front_rotor_pid = None
        self.back_rotor_pid = None

        super().__init__("Feedback linearization", {
            "Pitch derivative feed-forward": ParamBool(self.pitch_derivative_feed_forward),
            "Friction and centripetal": ParamBool(self.friction_centripetal),
            "k Elevation": ParamFloatArray([0, 0], [1000, 1000], self.ke),
            "k Travel": ParamFloatArray([0, 0], [1000, 1000], self.kl),
            "k Pitch": ParamFloatArray([0, 0], [1000, 1000], self.kp),
            "Rotor speed controller": ParamBool(self.rotor_speed_controller),
            "Rotor PD": ParamFloatArray([0, 0], [1000, 1000], self.k_rotor)
        })

    def control(self, t, x, e_traj, lambda_traj):
        p, e, l, dp, de, dl, wf, wb = x

        v1 = e_traj[2] - self.ke[1] * (de - e_traj[1]) - self.ke[0] * (e - e_traj[0])
        v2 = lambda_traj[2] - self.kl[1] * (dl - lambda_traj[1]) - self.kl[0] * (l - lambda_traj[0])

        if self.friction_centripetal:
            u1v = 1/L3 * (Je*v1 - L2*cos(e) + mue*de + Je*cos(e)*sin(e)*dl**2)
            u2v = 1/(L4*cos(e))*(Jl*v2 + mul*dl)
        else:
            u1v = 1/L3 * (- L2 * cos(e) + Je * v1)
            u2v = Jl / (L4 * cos(e)) * v2

        Vs = sqrt(u1v**2 + u2v**2)*sign(u1v)

        pd = arctan(u2v / u1v)

        if self.pitch_derivative_feed_forward:
            _, dpd, ddpd = compute_pitch_flatness_simple(e_traj, lambda_traj)
        else:
            dpd = 0
            ddpd = 0

        v3 = ddpd - self.kp[1] * (dp - dpd) - self.kp[0] * (p - pd)

        if self.friction_centripetal:
            Vd = 1/L1 * (Jp * v3 + mup * dp - Jp*cos(p)*sin(p)*(de**2-cos(e)**2*dl**2))
        else:
            Vd = Jp/L1 * v3

        Vf = (Vs + Vd) / 2
        Vb = (Vs - Vd) / 2

        if self.rotor_speed_controller:
            Vf = self.front_rotor_pid.compute(t, Vf - wf)
            Vb = self.back_rotor_pid.compute(t, Vb - wb)

        return Vf, Vb

    def initialize(self, param_value_dict):
        self.pitch_derivative_feed_forward = param_value_dict["Pitch derivative feed-forward"]
        self.friction_centripetal = param_value_dict["Friction and centripetal"]
        self.ke = param_value_dict["k Elevation"]
        self.kl = param_value_dict["k Travel"]
        self.kp = param_value_dict["k Pitch"]
        self.rotor_speed_controller = param_value_dict["Rotor speed controller"]
        self.k_rotor = param_value_dict["Rotor PD"]

        self.front_rotor_pid = PidAlgorithm([self.k_rotor[0], 0, self.k_rotor[1]])
        self.back_rotor_pid = PidAlgorithm([self.k_rotor[0], 0, self.k_rotor[1]])
