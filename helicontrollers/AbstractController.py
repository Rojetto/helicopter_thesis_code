from gui.ParameterFrame import ParameterizedClass
from abc import ABC


class AbstractController(ParameterizedClass, ABC):
    def initialize(self, trajectory):
        raise NotImplementedError

    def control(self, t, x):
        raise NotImplementedError
