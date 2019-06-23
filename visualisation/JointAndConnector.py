import vtk
import numpy as np
from visualisation.visualisation_util import setPokeMatrixWithMatr, drawLineFromMatrixToMatr


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


class Joint(object):
    def __init__(self, index, height, radius, T_joint):
        self.index = index
        self.height = height
        self.T_joint = T_joint
        self.endeff_list = []

        # Create Source
        self.source = vtk.vtkCylinderSource()
        self.source.SetRadius(radius)
        self.source.SetHeight(height)

        self.lineSource = vtk.vtkLineSource()
        self.lineSource.SetPoint1(0, 0, 0)
        self.lineSource.SetPoint2(0, 0, 1)
        self.lineMapper = vtk.vtkPolyDataMapper()
        self.lineMapper.SetInputConnection(self.lineSource.GetOutputPort())
        self.lineActor = vtk.vtkActor()
        self.lineActor.SetMapper(self.lineMapper)

        # Create Mapper
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputConnection(self.source.GetOutputPort())

        # Create Actor
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)

    def addActor(self, renderer):
        renderer.AddActor(self.actor)
        renderer.AddActor(self.lineActor)
        for endeff in self.endeff_list:
            endeff.addActor(renderer)

    def addEndeffector(self, T_e):
        """All Endeffectors must be connected BEFORE the call of addActor"""
        # create new endeffector
        self.endeff_list.append(Endeffector(self, T_e))

    def updatePosition(self, T_i_minus_one, theta, row):
        """Computes from theta the current poke matrix and sends it to the actor"""
        # Compute matrix
        theta_t, d, a, alpha = row
        T = getDHBMatrix(theta_t, d, a, alpha)
        T_i = np.dot(T_i_minus_one, T)

        # Set it to actor
        T_effec = np.dot(T_i_minus_one, self.T_joint)
        setPokeMatrixWithMatr(self.actor, T_effec)

        # Draw Line from this joint to the next joint
        drawLineFromMatrixToMatr(self.lineSource, T_i_minus_one, T_i)

        # Update endeffectors connected to joint as well
        for endeff in self.endeff_list:
            endeff.updatePosition(T_i, T_effec)

        return T_i


class Endeffector(object):
    def __init__(self, parent, T_e):
        self.T_e = T_e

        # Create Cube
        self.source = vtk.vtkCubeSource()
        self.source.SetXLength(0.5)
        self.source.SetYLength(0.5)
        self.source.SetZLength(0.5)

        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputConnection(self.source.GetOutputPort())

        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)

        # Create Line
        self.lineSource = vtk.vtkLineSource()
        self.lineMapper = vtk.vtkPolyDataMapper()
        self.lineMapper.SetInputConnection(self.lineSource.GetOutputPort())
        self.lineActor = vtk.vtkActor()
        self.lineActor.SetMapper(self.lineMapper)

    def addActor(self, renderer):
        renderer.AddActor(self.actor)
        renderer.AddActor(self.lineActor)

    def updatePosition(self, T_i, T_lastJoint):
        """T_lastJoint: Matrix of last GEOMETRIC joint"""
        # calc position
        T = np.dot(T_i, self.T_e)
        setPokeMatrixWithMatr(self.actor, T)
        # Draw line from last joint to endeffector
        drawLineFromMatrixToMatr(self.lineSource, T_lastJoint, T)

