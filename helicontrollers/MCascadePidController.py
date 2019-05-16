from helicontrollers.AbstractController import *
import matlab.engine


class MCascadePidController(AbstractController):
    def __init__(self):
        self.matlab_engine = None
        self.matlab_controller = None

        param_definition_dict = {}

        session_name = 'matlab_heli'

        try:
            self.matlab_engine = matlab.engine.connect_matlab(session_name)
            print(f"Successfully connected to MATLAB session '{session_name}'")

            try:
                self.matlab_controller = self.matlab_engine.CascadePid()
                names_values = self.matlab_engine.getParametersAndValues(self.matlab_controller)

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
        except matlab.engine.EngineError:
            print(f"Unable to connect to MATLAB session '{session_name}'")

        super().__init__("Cascade PID (MATLAB)", param_definition_dict)

    def control(self, t, x, e_traj, lambda_traj):
        x = matlab.double(list(x))
        e_traj = matlab.double(list(e_traj))
        lambda_traj = matlab.double(list(lambda_traj))

        u = [0.0, 0.0]

        if self.matlab_controller is not None:
            matlab_result = self.matlab_engine.control(self.matlab_controller, t, x, e_traj, lambda_traj)
            Vf = matlab_result[0][0]
            Vb = matlab_result[1][0]
            u = [Vf, Vb]

        return u

    def initialize(self, param_value_dict):
        if self.matlab_engine is not None:
            self.matlab_controller = self.matlab_engine.CascadePid()

            flat = []
            for name, value in param_value_dict.items():
                if type(value) is list:
                    matlab_value = matlab.double(value)
                else:
                    matlab_value = value

                flat += [name, matlab_value]

            self.matlab_engine.setAllParameters(self.matlab_controller, flat, nargout=0)
            self.matlab_engine.initialize(self.matlab_controller, nargout=0)
