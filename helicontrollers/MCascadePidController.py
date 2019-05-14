from helicontrollers.AbstractController import *
import matlab.engine


class MCascadePidController(AbstractController):
    def __init__(self):
        self.elevation_pid_gains = [10, 0, 5]
        self.travel_pitch_pid_gains = [1.5, 0, 1.5]
        self.pitch_vd_pid_gains = [20, 0, 2.3]
        self.params = matlab.double(self.elevation_pid_gains + self.travel_pitch_pid_gains + self.pitch_vd_pid_gains)
        self.matlab_engine = None

        super().__init__("Cascade PID (MATLAB)", {
            "Elevation PID": ParamFloatArray([0, 0, 0],
                                             [100, 100, 100],
                                             self.elevation_pid_gains),
            "Travel-Pitch PID": ParamFloatArray([0, 0, 0],
                                                [100, 100, 100],
                                                self.travel_pitch_pid_gains),
            "Pitch-Vd PID": ParamFloatArray([0, 0, 0],
                                            [100, 100, 100],
                                            self.pitch_vd_pid_gains)
        })

    def control(self, t, x, e_traj, lambda_traj):
        x = matlab.double(list(x))
        e_traj = matlab.double(list(e_traj))
        lambda_traj = matlab.double(list(lambda_traj))

        u = [0.0, 0.0]

        if self.matlab_engine is not None:
            matlab_result = self.matlab_engine.cascade_pid(t, x, e_traj, lambda_traj, self.params)
            Vf = matlab_result[0][0]
            Vb = matlab_result[1][0]
            u = [Vf, Vb]

        return u

    def initialize(self, param_value_dict):
        self.elevation_pid_gains = param_value_dict["Elevation PID"]
        self.travel_pitch_pid_gains = param_value_dict["Travel-Pitch PID"]
        self.pitch_vd_pid_gains = param_value_dict["Pitch-Vd PID"]
        self.params = matlab.double(self.elevation_pid_gains + self.travel_pitch_pid_gains + self.pitch_vd_pid_gains)

        if self.matlab_engine is None:
            session_name = 'matlab_heli'

            try:
                self.matlab_engine = matlab.engine.connect_matlab(session_name)
                print(f"Successfully connected to MATLAB session '{session_name}'")
            except matlab.engine.EngineError:
                print(f"Unable to connect to MATLAB session '{session_name}'")

        if self.matlab_engine is not None:
            self.matlab_engine.clear_controllers(nargout=0)
