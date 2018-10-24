import numpy as np
import ModelConstants as mc

class HeliControl(object):
    def __init__(self):
        self.operatingPoint = np.array([0, 0, 0])
        pass

    def control(self, t, x):
        """Is called by the main loop in order to get the current controller output.
            Args:
                t: current simulation time
                x: current system state. x = [p, e, lambda, dp/dt, de/dt, dlambda/dt]
            Returns:
                u: control output. u = [Vf, Vb]"""
        return [0, 0]

    def setOperatingPoint(self, point):
        """This function is called in order to set the current operating point of the controller.
        point: list of coordinates
        point[0]: Pitch
        point[1]: Elevation
        point[2]: Lambda """
        self.operatingPoint = np.array(point)
        #print("Pitch = " + str(point[0]) + " Elevation = " + str(point[1]) + " Lambda = " + str(point[2]))
