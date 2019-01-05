from enum import Enum

from helicontrollers.AbstractController import *
from helicontrollers.util import PidAlgorithm
from JoystickWidget import JoystickWidget
import numpy as np


class InteractiveMode(Enum):
    VD_VS = 0
    PHI_VS = 1
    DLAMBDA_DE = 2


class InteractiveController(AbstractController):
    VD_MAX = 0.5
    VS_MAX = 1
    PHI_MAX = 80 / 180 * np.pi
    DLAMBDA_MAX = 90 / 180 * np.pi
    DEPSILON_MAX = 90 / 180 * np.pi

    def __init__(self, joystick_widget: JoystickWidget):
        self.mode = InteractiveMode.VD_VS
        self.rotor_speed_pd = [5, 0]
        self.phi_pd_gains = [20, 2]

        self.front_rotor_pd = None
        self.back_rotor_pd = None
        self.phi_pd = None

        self.joystick_widget = joystick_widget
        self.joystick_widget.pos_changed.connect(self.on_pos_changed)

        self.x = 0
        self.y = 0

        super().__init__("Interactive control", {
            "Mode": ParamEnum(["Vd, Vs", "phi, Vs", "dlambda, depsilon"],
                              [InteractiveMode.VD_VS, InteractiveMode.PHI_VS, InteractiveMode.DLAMBDA_DE],
                              self.mode),
            "Rotor speed PD": ParamFloatArray([0, 0], [100, 100], self.rotor_speed_pd),
            "Phi PD": ParamFloatArray([0, 0], [100, 100], self.phi_pd_gains)
        })

    def on_pos_changed(self, x, y):
        self.x = x
        self.y = y

    def control(self, t, x, e_traj, lambda_traj):
        p, e, l, dp, de, dl, wf, wb = x

        Vf = 0
        Vb = 0

        if self.mode == InteractiveMode.VD_VS:
            Vd = self.x
            Vs = self.y

            Vf = (Vs + Vd) / 2
            Vb = (Vs - Vd) / 2
        elif self.mode == InteractiveMode.PHI_VS:
            Vs = self.y
            phi_d = self.x

            p_error = x[0] - phi_d
            Vd = - self.phi_pd.compute(t, p_error, dp)

            Vf = (Vs + Vd) / 2
            Vb = (Vs - Vd) / 2

        Vf = Vf + self.front_rotor_pd.compute(t, Vf - wf)
        Vb = Vb + self.back_rotor_pd.compute(t, Vb - wb)

        return [Vf, Vb]

    def initialize(self, param_value_dict):
        self.mode = param_value_dict["Mode"]
        self.rotor_speed_pd = param_value_dict["Rotor speed PD"]
        self.phi_pd_gains = param_value_dict["Phi PD"]

        self.front_rotor_pd = PidAlgorithm([self.rotor_speed_pd[0], 0, self.rotor_speed_pd[1]])
        self.back_rotor_pd = PidAlgorithm([self.rotor_speed_pd[0], 0, self.rotor_speed_pd[1]])
        self.phi_pd = PidAlgorithm([self.phi_pd_gains[0], 0, self.phi_pd_gains[1]])

        if self.mode == InteractiveMode.VD_VS:
            self.joystick_widget.real_x_min = -self.VD_MAX
            self.joystick_widget.real_x_max = self.VD_MAX
            self.joystick_widget.real_y_min = 0
            self.joystick_widget.real_y_max = self.VS_MAX
        elif self.mode == InteractiveMode.PHI_VS:
            self.joystick_widget.real_x_min = -self.PHI_MAX
            self.joystick_widget.real_x_max = self.PHI_MAX
            self.joystick_widget.real_y_min = 0
            self.joystick_widget.real_y_max = self.VS_MAX
        elif self.mode == InteractiveMode.DLAMBDA_DE:
            self.joystick_widget.real_x_min = -self.DLAMBDA_MAX
            self.joystick_widget.real_x_max = self.DLAMBDA_MAX
            self.joystick_widget.real_y_min = -self.DEPSILON_MAX
            self.joystick_widget.real_y_max = self.DEPSILON_MAX

        self.joystick_widget.reset_pos()
