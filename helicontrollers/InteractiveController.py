from enum import Enum

from helicontrollers.AbstractController import *
from helicontrollers.util import PidAlgorithm
from JoystickWidget import JoystickWidget
import numpy as np


class InteractiveMode(Enum):
    VD_VS = 0
    PHI_VS = 1
    DLAMBDA_DEPSILON = 2
    VF_VB = 3


class InteractiveController(AbstractController):
    VD_MAX = 0.5
    VS_MAX = 1.0
    VFB_MAX = 0.5
    PHI_MAX = 80
    DLAMBDA_MAX = 90
    DEPSILON_MAX = 90

    def __init__(self, joystick_widget: JoystickWidget):
        self.mode = InteractiveMode.VF_VB
        self.rotor_speed_pd = [5, 0]
        self.phi_pd_gains = [20, 2]
        self.dlambda_pd_gains = [0, 0.5]
        self.depsilon_pd_gains = [15, 5]

        self.front_rotor_pd = None
        self.back_rotor_pd = None
        self.phi_pd = None
        self.dlambda_pd = None
        self.depsilon_pd = None

        self.joystick_widget = joystick_widget
        self.joystick_widget.pos_changed.connect(self.on_pos_changed)

        self.x = 0
        self.y = 0

        self.target_lambda = 0
        self.target_epsilon = 0

        super().__init__("Interactive control", {
            "Mode": ParamEnum(["Vf, Vb", "Vd, Vs", "phi, Vs", "dlambda, depsilon"],
                              [InteractiveMode.VF_VB, InteractiveMode.VD_VS, InteractiveMode.PHI_VS, InteractiveMode.DLAMBDA_DEPSILON],
                              self.mode),
            "Rotor speed PD": ParamFloatArray([0, 0], [100, 100], self.rotor_speed_pd),
            "Phi PD": ParamFloatArray([0, 0], [100, 100], self.phi_pd_gains),
            "dlambda PD": ParamFloatArray([0, 0], [100, 100], self.dlambda_pd_gains),
            "depsilon PD": ParamFloatArray([0, 0], [100, 100], self.depsilon_pd_gains)
        })

    def on_pos_changed(self, x, y):
        self.x = x
        self.y = y

    def control(self, t, x, e_traj, lambda_traj):
        p, e, l, dp, de, dl, wf, wb = x

        Vf = 0
        Vb = 0

        if self.mode == InteractiveMode.VF_VB:
            Vf = self.x
            Vb = self.y
        elif self.mode == InteractiveMode.VD_VS:
            Vd = self.x
            Vs = self.y

            Vf = (Vs + Vd) / 2
            Vb = (Vs - Vd) / 2
        elif self.mode == InteractiveMode.PHI_VS:
            Vs = self.y
            phi_d = self.x / 180 * np.pi

            p_error = p - phi_d
            Vd = - self.phi_pd.compute(t, p_error, dp)

            Vf = (Vs + Vd) / 2
            Vb = (Vs - Vd) / 2
        elif self.mode == InteractiveMode.DLAMBDA_DEPSILON:
            dlambda_d = self.x / 180 * np.pi
            depsilon_d = self.y / 180 * np.pi

            self.target_epsilon += 1 / 60.0 * depsilon_d
            self.target_lambda += 1 / 60.0 * dlambda_d

            phi_d = self.dlambda_pd.compute(t, self.target_lambda - l, dlambda_d - dl)
            Vd = - self.phi_pd.compute(t, p - phi_d, dp)

            Vs = self.depsilon_pd.compute(t, self.target_epsilon - e, depsilon_d - de)

            Vf = (Vs + Vd) / 2
            Vb = (Vs - Vd) / 2

        Vf = Vf + self.front_rotor_pd.compute(t, Vf - wf)
        Vb = Vb + self.back_rotor_pd.compute(t, Vb - wb)

        return [Vf, Vb]

    def initialize(self, param_value_dict):
        self.mode = param_value_dict["Mode"]
        self.rotor_speed_pd = param_value_dict["Rotor speed PD"]
        self.phi_pd_gains = param_value_dict["Phi PD"]
        self.dlambda_pd_gains = param_value_dict["dlambda PD"]
        self.depsilon_pd_gains = param_value_dict["depsilon PD"]

        self.front_rotor_pd = PidAlgorithm([self.rotor_speed_pd[0], 0, self.rotor_speed_pd[1]])
        self.back_rotor_pd = PidAlgorithm([self.rotor_speed_pd[0], 0, self.rotor_speed_pd[1]])
        self.phi_pd = PidAlgorithm([self.phi_pd_gains[0], 0, self.phi_pd_gains[1]])
        self.dlambda_pd = PidAlgorithm([self.dlambda_pd_gains[0], 0, self.dlambda_pd_gains[1]])
        self.depsilon_pd = PidAlgorithm([self.depsilon_pd_gains[0], 0, self.depsilon_pd_gains[1]])

        if self.mode == InteractiveMode.VF_VB:
            self.joystick_widget.real_x_min = 0
            self.joystick_widget.real_x_max = self.VFB_MAX
            self.joystick_widget.real_y_min = 0
            self.joystick_widget.real_y_max = self.VFB_MAX
        elif self.mode == InteractiveMode.VD_VS:
            self.joystick_widget.real_x_min = -self.VD_MAX
            self.joystick_widget.real_x_max = self.VD_MAX
            self.joystick_widget.real_y_min = 0
            self.joystick_widget.real_y_max = self.VS_MAX
        elif self.mode == InteractiveMode.PHI_VS:
            self.joystick_widget.real_x_min = -self.PHI_MAX
            self.joystick_widget.real_x_max = self.PHI_MAX
            self.joystick_widget.real_y_min = 0
            self.joystick_widget.real_y_max = self.VS_MAX
        elif self.mode == InteractiveMode.DLAMBDA_DEPSILON:
            self.joystick_widget.real_x_min = -self.DLAMBDA_MAX
            self.joystick_widget.real_x_max = self.DLAMBDA_MAX
            self.joystick_widget.real_y_min = -self.DEPSILON_MAX
            self.joystick_widget.real_y_max = self.DEPSILON_MAX

        self.target_lambda = 0
        self.target_epsilon = 0
        self.joystick_widget.reset_pos()
