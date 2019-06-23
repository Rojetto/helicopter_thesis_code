from visualisation.visualisation_util import *
from numpy import array, sin, cos


def getDHBTable(travel, elevation, pitch):
    dnh_table = np.array([[travel, vtk_l_a, 0, -np.pi / 2],
                          [elevation + np.pi / 2, 0, 0, np.pi / 2],
                          [pitch, vtk_l_h, 0, 0]])
    return dnh_table


def make_cube(renderer):
    source = vtk.vtkCubeSource()
    source.SetXLength(0.05)
    source.SetYLength(0.05)
    source.SetZLength(0.05)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    renderer.AddActor(actor)

    return actor


def make_joint(renderer):
    source = vtk.vtkCylinderSource()
    source.SetRadius(0.025)
    source.SetHeight(0.1)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    renderer.AddActor(actor)

    return actor


def make_line(renderer):
    source = vtk.vtkLineSource()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    renderer.AddActor(actor)

    return source


class HelicopterModel(object):
    def __init__(self, renderer):
        self.axesActor = vtk.vtkAxesActor()

        self.j_pitch, self.j_elevation, self.j_travel = make_joint(renderer), make_joint(renderer), make_joint(renderer)
        self.c_weight, self.c_front, self.c_back = make_cube(renderer), make_cube(renderer), make_cube(renderer)
        self.l_base, self.l_l_c, self.l_d_c, = make_line(renderer), make_line(renderer), make_line(renderer)
        self.l_l_h, self.l_d_1, self.l_d_2, self.l_l_p_1, self.l_l_p_2 = make_line(renderer), make_line(renderer), make_line(renderer), make_line(renderer), make_line(renderer)

    def setState(self, travel, elevation, pitch):

        I = array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
        IA = array([[cos(travel), -sin(travel), 0, 0],
                    [sin(travel), cos(travel), 0, 0],
                    [0, 0, 1, 0.3],
                    [0, 0, 0, 1]])
        AB = array([[cos(elevation), 0, -sin(elevation), 0],
                    [0, 1, 0, 0],
                    [sin(elevation), 0, cos(elevation), 0],
                    [0, 0, 0, 1]])
        BC = array([[1, 0, 0, vtk_l_h],
                    [0, cos(pitch), sin(pitch), 0],
                    [0, -sin(pitch), cos(pitch), -vtk_d_1],
                    [0, 0, 0, 1]])
        CMF = array([[1, 0, 0, 0],
                     [0, 1, 0, -vtk_l_p],
                     [0, 0, 1, vtk_d_2],
                     [0, 0, 0, 1]])
        CMB = array([[1, 0, 0, 0],
                     [0, 1, 0, vtk_l_p],
                     [0, 0, 1, vtk_d_2],
                     [0, 0, 0, 1]])
        BMC = array([[1, 0, 0, -vtk_l_c],
                     [0, 1, 0, 0],
                     [0, 0, 1, vtk_d_c],
                     [0, 0, 0, 1]])

        IMC = IA @ AB @ BMC
        IMF = IA @ AB @ BC @ CMF
        IMB = IA @ AB @ BC @ CMB

        setPokeMatrixWithMatr(self.c_weight, IMC)
        setPokeMatrixWithMatr(self.c_front, IMF)
        setPokeMatrixWithMatr(self.c_back, IMB)

        rot_x = array([[1, 0, 0, 0],
                       [0, 0, -1, 0],
                       [0, 1, 0, 0],
                       [0, 0, 0, 1]])

        rot_z = array([[0, -1, 0, 0],
                       [1, 0, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        translate = lambda x, y, z: array([[1, 0, 0, x],
                                           [0, 1, 0, y],
                                           [0, 0, 1, z],
                                           [0, 0, 0, 1]])

        setPokeMatrixWithMatr(self.j_travel, I@rot_x)
        setPokeMatrixWithMatr(self.j_elevation, I@IA)
        setPokeMatrixWithMatr(self.j_pitch, I@IA@AB@BC@rot_z)

        drawLineFromMatrixToMatr(self.l_base, I, I@IA)
        drawLineFromMatrixToMatr(self.l_l_c, I@IA@AB, I@IA@AB@translate(-vtk_l_c, 0, 0))
        drawLineFromMatrixToMatr(self.l_d_c, I@IA@AB@translate(-vtk_l_c, 0, 0), IMC)
        drawLineFromMatrixToMatr(self.l_l_h, I@IA@AB, I@IA@AB@translate(vtk_l_h, 0, 0))
        drawLineFromMatrixToMatr(self.l_d_1, I@IA@AB@translate(vtk_l_h, 0, 0), I@IA@AB@BC)
        drawLineFromMatrixToMatr(self.l_d_2, I@IA@AB@BC, I@IA@AB@BC@translate(0, 0, vtk_d_2))
        drawLineFromMatrixToMatr(self.l_l_p_1, I@IA@AB@BC@translate(0, 0, vtk_d_2), IMF)
        drawLineFromMatrixToMatr(self.l_l_p_2, I@IA@AB@BC@translate(0, 0, vtk_d_2), IMB)
