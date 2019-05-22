from ParameterFrame import ParameterFrame, ParameterizedClass, ParamFloatArray, ParamBool
from Trajectory import Trajectory
from numpy import array, linspace, zeros, empty, pi
from helicontrollers.util import compute_feed_forward_static, compute_pitch_and_inputs_flatness_centripetal
from abc import ABC
from Planner import PolynomialPlanner

deg = pi / 180


class TrajectoryFrame(ParameterFrame):
    def __init__(self):
        super().__init__([TypeOperatingPoint(), TypePolynomialFlatness()])

    def get_trajectory(self):
        traj_type = self.get_selected_object()
        return traj_type.generate()


class TrajectoryType(ParameterizedClass, ABC):
    def generate(self):
        raise NotImplementedError


class TypeOperatingPoint(TrajectoryType):
    def __init__(self):
        self.elevation = 20.0
        self.travel = 30.0

        super().__init__("Constant Operating Point", {
            "Elevation": ParamFloatArray([self.elevation]),
            "Travel": ParamFloatArray([self.travel])
        })

    def set_params(self, param_value_dict):
        self.travel = param_value_dict["Travel"][0]
        self.elevation = param_value_dict["Elevation"][0]

    def generate(self):
        t = array([0.0])
        phi = array([[0.0]])
        eps = array([[self.elevation * deg]])
        lamb = array([[self.travel * deg]])
        vf, vb = compute_feed_forward_static(eps[0], lamb[0])
        vf = array([[vf]])
        vb = array([[vb]])

        return Trajectory(t, phi, eps, lamb, vf, vb)


class TypePolynomialFlatness(TrajectoryType):
    def __init__(self):
        self.elevation_start = 0.0
        self.elevation_end = 30.0
        self.travel_start = 0.0
        self.travel_end = 30.0
        self.t_end = 8.0
        self.derivative_order = 4
        self.sample_rate = 60.0

        super().__init__("Polynomial using flatness", {
            "Elevation start-end": ParamFloatArray([self.elevation_start, self.elevation_end]),
            "Travel start-end": ParamFloatArray([self.travel_start, self.travel_end]),
            "End time": ParamFloatArray([self.t_end]),
            #"Derivatives": ParamFloatArray([self.derivative_order]),
            "Sample rate (Hz)": ParamFloatArray([self.sample_rate])
        })

    def set_params(self, param_value_dict):
        self.elevation_start, self.elevation_end = param_value_dict["Elevation start-end"]
        self.travel_start, self.travel_end = param_value_dict["Travel start-end"]
        self.t_end = param_value_dict["End time"][0]
        #self.derivative_order = int(param_value_dict["Derivatives"][0])
        self.sample_rate = param_value_dict["Sample rate (Hz)"][0]

    def generate(self):
        elevation_d_start = zeros(self.derivative_order + 1)
        elevation_d_start[0] = self.elevation_start * deg

        elevation_d_end = zeros(self.derivative_order + 1)
        elevation_d_end[0] = self.elevation_end * deg

        travel_d_start = zeros(self.derivative_order + 1)
        travel_d_start[0] = self.travel_start * deg

        travel_d_end = zeros(self.derivative_order + 1)
        travel_d_end[0] = self.travel_end * deg

        elevation_planner = PolynomialPlanner(elevation_d_start, elevation_d_end, 0.0, self.t_end, self.derivative_order)
        travel_planner = PolynomialPlanner(travel_d_start, travel_d_end, 0.0, self.t_end, self.derivative_order)

        num_samples = int(self.t_end * self.sample_rate)
        t = linspace(0.0, self.t_end, num_samples)
        eps = elevation_planner.eval_vec(t)
        lamb = travel_planner.eval_vec(t)

        phi = empty((num_samples, 3))
        vf = empty((num_samples, 1))
        vb = empty((num_samples, 1))

        for i in range(num_samples):
            phi_i, vf_vb_i = compute_pitch_and_inputs_flatness_centripetal(eps[i], lamb[i])
            phi[i] = phi_i
            vf[i, 0] = vf_vb_i[0]
            vb[i, 0] = vf_vb_i[1]

        return Trajectory(t, phi, eps, lamb, vf, vb)
