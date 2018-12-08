import abc
import numpy as np


class Disturbance(object):
    """Base class for arbitrary disturbance"""
    def __init__(self):
        self.idx_lookup = {'p': 0, 'e': 1, "lambda": 2, 'f': 3, 'b': 4}

    @abc.abstractmethod
    def eval(self, t):
        """:return ndarray (with 5 elements) describing the disturbance at t for p, e, d, f and b"""
        return


class DisturbanceStep(Disturbance):
    """Heaviside step"""
    def __init__(self, t_start, z0, zf, point_of_application):
        super().__init__()
        self.t_start = t_start
        self.z0 = z0
        self.zf = zf
        self.point_of_application = point_of_application

    def eval(self, t):
        ret = np.zeros(5)
        ret[self.idx_lookup[self.point_of_application]] = self.z0 + self.zf * np.heaviside(t - self.t_start, 0.5)
        return ret


class DisturbanceSinus(Disturbance):
    """Heaviside step"""
    def __init__(self, t_start, z_offset, z_amplitude, z_frequency, point_of_application):
        super().__init__()
        self.t_start = t_start
        self.z_offset = z_offset
        self.z_amplitude = z_amplitude
        self.z_frequency = z_frequency
        self.point_of_application = point_of_application

    def eval(self, t):
        ret = np.zeros(5)
        ret[self.idx_lookup[self.point_of_application]] = (np.heaviside(t - self.t_start, 0.5) * (self.z_offset +
                                                           self.z_amplitude * np.sin(2*np.pi*self.z_frequency * t)))
        return ret


class DisturbanceRect(Disturbance):
    """Impulse with specific length and height"""
    def __init__(self, t_start, t_length, zf, point_of_application):
        super().__init__()
        self.t_start = t_start
        self.t_length = t_length
        self.zf = zf
        self.point_of_application = point_of_application

    def eval(self, t):
        ret = np.zeros(5)
        ret[self.idx_lookup[self.point_of_application]] = self.zf if self.t_start <= t < self.t_start + self.t_length else 0
        return ret


class NoDisturbance(Disturbance):
    def __init__(self):
        super().__init__()

    def eval(self, t):
        return np.zeros(5)


