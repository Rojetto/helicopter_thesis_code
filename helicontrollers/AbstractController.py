class ParamBool:
    def __init__(self, init_value):
        self.init_value = init_value


class ParamFloatArray:
    def __init__(self, init_values):
        self.init_values = init_values


class ParamEnum:
    def __init__(self, option_labels, option_values, init_value):
        self.option_labels = option_labels
        self.option_values = option_values
        self.init_value = init_value


class AbstractController:
    def __init__(self, name, param_definition_dict):
        self.name = name
        self.param_definition_dict = param_definition_dict

    def control(self, t, x, e_traj, lambda_traj):
        raise NotImplementedError

    def initialize(self, param_value_dict):
        raise NotImplementedError
