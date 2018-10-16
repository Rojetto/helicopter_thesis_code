import numpy as np
import ModelConstants as mc

class HeliControl(object):
    def __init__(self):
        pass

    def control(self, t, x):
        """Is called by the main loop in order to get the current controller output.
            Args:
                t: current simulation time
                x: current system state. x = [p, e, lambda, dp/dt, de/dt, dlambda/dt]
            Returns:
                u: control output. u = [Vf, Vb]"""
        return [0, 0]