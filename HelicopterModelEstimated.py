import vtk
import numpy as np
import JointAndConnectorEstimated as jc
from visualisation_util import *


def getDHBTable(theta1, theta2, theta3):
    dnh_table = np.array([[theta1, vtk_l_a, 0, -np.pi / 2],
                          [theta2 + np.pi / 2, 0, 0, np.pi / 2],
                          [theta3, vtk_l_h, 0, 0]])
    return dnh_table


def getDHBMatrix(theta, d, a, alpha):
    """Returns the homogenous i-1/i-Denavit-Hartenberg-Transformation-Matrix. Parameters have the index i"""
    c_t = np.cos(theta)
    s_t = np.sin(theta)
    c_a = np.cos(alpha)
    s_a = np.sin(alpha)
    m = np.array([[c_t, -s_t*c_a, s_t*s_a, a*c_t],
                  [s_t, c_t*c_a, -c_t*s_a, a*s_t],
                  [0, s_a, c_a, d],
                  [0, 0, 0, 1]])
    return m


class HelicopterModelEstimated(object):
    def __init__(self):
        self.state = [0, 0, 0] #theta1, theta2, theta3
        self.axesActor = vtk.vtkAxesActor()

        self.joint1 = jc.JointEstimated(1, 1, 0.25, T_j1)
        self.joint2 = jc.JointEstimated(2, 1, 0.25, T_j2)
        self.joint3 = jc.JointEstimated(3, 1, 0.25, T_j3)

        self.joint2.addEndeffector(T_e3)
        self.joint3.addEndeffector(T_e1)
        self.joint3.addEndeffector(T_e2)

    def addAllActors(self, renderer):
        """Must be called from main script in order to add all actors to the renderer"""
        self.joint1.addActor(renderer)
        self.joint2.addActor(renderer)
        self.joint3.addActor(renderer)
        renderer.AddActor(self.axesActor)

    def setState(self, theta1, theta2, theta3):
        """Updates the angles so that it looks different on the screen. ATTENTION: There was
        a bug related to the sign of the pitch angle, which was chosen differently in the exercise manual.
        Out of some reason I still do not understand completely, the elevation angle has a wrong sign as well.
        In this function this bug was simply fixed by multiplying the angles with -1."""
        theta2 = -theta2
        theta3 = -theta3
        self.state = [theta1, theta2, theta3]
        dhb_table = getDHBTable(theta1, theta2, theta3)
        T_0 = np.eye(4)
        T_1 = self.joint1.updatePosition(T_0, theta1, dhb_table[0])
        T_2 = self.joint2.updatePosition(T_1, theta2, dhb_table[1])
        T_3 = self.joint3.updatePosition(T_2, theta3, dhb_table[2])

    def getState(self):
        """get the state. change it later to read only"""
        return self.state

    def setVisibility(self, visibility):
        self.joint1.setVisibility(visibility)
        self.joint2.setVisibility(visibility)
        self.joint3.setVisibility(visibility)
