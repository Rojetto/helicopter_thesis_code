from ParameterFrame import ParameterizedClass


class AbstractController(ParameterizedClass):
    def initialize(self, param_value_dict):
        raise NotImplementedError

    def control(self, t, x):
        raise NotImplementedError
