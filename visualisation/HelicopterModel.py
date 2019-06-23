from visualisation import JointAndConnector as jc
from visualisation.visualisation_util import *


def getDHBTable(theta1, theta2, theta3):
    dnh_table = np.array([[theta1, vtk_l_a, 0, -np.pi / 2],
                          [theta2 + np.pi / 2, 0, 0, np.pi / 2],
                          [theta3, vtk_l_h, 0, 0]])
    return dnh_table


class HelicopterModel(object):
    def __init__(self):
        self.state = [0, 0, 0]  # theta1, theta2, theta3
        self.axesActor = vtk.vtkAxesActor()

        self.joint_travel = jc.Joint(1, 0.1, 0.025, T_joint_travel)
        self.joint_elevation = jc.Joint(2, 0.1, 0.025, T_joint_elevation)
        self.joint_pitch = jc.Joint(3, 0.1, 0.025, T_joint_pitch)

        self.joint_elevation.addEndeffector(T_counter_weight)
        self.joint_pitch.addEndeffector(T_front_rotor)
        self.joint_pitch.addEndeffector(T_back_rotor)

    def addAllActors(self, renderer):
        """Must be called from main script in order to add all actors to the renderer"""
        self.joint_travel.addActor(renderer)
        self.joint_elevation.addActor(renderer)
        self.joint_pitch.addActor(renderer)
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
        T_1 = self.joint_travel.updatePosition(T_0, theta1, dhb_table[0])
        T_2 = self.joint_elevation.updatePosition(T_1, theta2, dhb_table[1])
        T_3 = self.joint_pitch.updatePosition(T_2, theta3, dhb_table[2])

    def getState(self):
        """get the state. change it later to read only"""
        return self.state
