from ParameterFrame import ParameterFrame, ParameterizedClass, ParamFloatArray, ParamBool
from Trajectory import Trajectory
from numpy import array
from helicontrollers.util import compute_feed_forward_static
from abc import ABC


class TrajectoryFrame(ParameterFrame):
    def __init__(self):
        super().__init__([TypeOperatingPoint()])

    def get_trajectory(self):
        traj_type = self.get_selected_object()
        return traj_type.generate()


class TrajectoryType(ParameterizedClass, ABC):
    def generate(self):
        raise NotImplementedError


class TypeOperatingPoint(TrajectoryType):
    def __init__(self):
        self.travel = 30.0
        self.elevation = 20.0

        super().__init__("Constant Operating Point", {
            "Travel": ParamFloatArray([self.travel]),
            "Elevation": ParamFloatArray([self.elevation])
        })

    def set_params(self, param_value_dict):
        self.travel = param_value_dict["Travel"]
        self.elevation = param_value_dict["Elevation"]

    def generate(self):
        t = array([0.0])
        phi = array([0.0])
        eps = array([self.elevation])
        lamb = array([self.travel])
        vf, vb = compute_feed_forward_static(eps, lamb)
        vf = array([vf])
        vb = array([vb])

        return Trajectory(t, phi, eps, lamb, vf, vb)
