from helicontrollers.AbstractController import AbstractController
from ParameterFrame import ParamFloatArray, ParamBool
import matlab.engine


class MatlabController(AbstractController):
    matlab_engine = None

    def __init__(self, matlab_class_name):
        self.matlab_class_name = matlab_class_name
        self.matlab_controller = None
        self.matlab_errored = False

        param_definition_dict = {}

        session_name = 'matlab_heli'

        if MatlabController.matlab_engine is None:
            try:
                MatlabController.matlab_engine = matlab.engine.connect_matlab(session_name)
                print(f"Successfully connected to MATLAB session '{session_name}'")
            except matlab.engine.EngineError:
                print(f"Unable to connect to MATLAB session '{session_name}'")

        if MatlabController.matlab_engine is not None:
            try:
                self.matlab_controller = getattr(MatlabController.matlab_engine, self.matlab_class_name)()
                names_values = MatlabController.matlab_engine.getParametersAndValues(self.matlab_controller)

                for i in range(0, len(names_values), 2):
                    name = names_values[i]
                    value = names_values[i+1]

                    if type(value) is float:
                        definition = ParamFloatArray([value])
                    elif type(value) is matlab.double:
                        definition = ParamFloatArray(list(value[0]))
                    elif type(value) is bool:
                        definition = ParamBool(value)
                    else:
                        raise RuntimeError(f"Unable to create control for parameter of type {type(value)}")

                    param_definition_dict[name] = definition
            except matlab.engine.EngineError as e:
                print(f"Unable to instantiate MATLAB controller")

        super().__init__(f"{matlab_class_name} (MATLAB)", param_definition_dict)

    def control(self, t, x):
        x = matlab.double(list(x))

        u = [0.0, 0.0]

        if self.matlab_controller is not None:
            try:
                matlab_result = MatlabController.matlab_engine.control(self.matlab_controller, t, x)
                Vf = matlab_result[0][0]
                Vb = matlab_result[1][0]
                u = [Vf, Vb]
            except matlab.engine.MatlabExecutionError as e:
                if not self.matlab_errored:
                    print(f"Error while executing MATLAB controller step code:\n{str(e)}")
                    self.matlab_errored = True

        return u

    def set_params(self, param_value_dict):
        if MatlabController.matlab_engine is not None:
            self.matlab_controller = getattr(MatlabController.matlab_engine, self.matlab_class_name)()

            flat = []
            for name, value in param_value_dict.items():
                if type(value) is list:
                    matlab_value = matlab.double(value)
                else:
                    matlab_value = value

                flat += [name, matlab_value]

            MatlabController.matlab_engine.setAllParameters(self.matlab_controller, flat, nargout=0)

    def initialize(self, trajectory):
        self.matlab_errored = False

        if MatlabController.matlab_engine is not None:
            t_d = matlab.double(trajectory.t.tolist())
            phi_d = matlab.double(trajectory.phi.tolist())
            eps_d = matlab.double(trajectory.eps.tolist())
            lamb_d = matlab.double(trajectory.lamb.tolist())
            vf_d = matlab.double(trajectory.vf.tolist())
            vb_d = matlab.double(trajectory.vb.tolist())

            try:
                MatlabController.matlab_engine.initializeWithIndividualArguments(self.matlab_controller, t_d, phi_d, eps_d, lamb_d, vf_d, vb_d, nargout=0)
            except matlab.engine.MatlabExecutionError as e:
                print(f"Error while executing MATLAB controller initialization code:\n{str(e)}")
