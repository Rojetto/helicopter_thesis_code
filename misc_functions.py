import vtk
import numpy as np
from ModelConstants import *

#set T_vtk to identity matrix for no artificial coordinates
T_vtk = np.array([[1, 0, 0, -(vtk_l_h - vtk_l_c)/2],
                  [0, 1, 0, 0],
                  [0, 0, 1, -vtk_l_a/2],
                  [0, 0, 0, 1]])

def drawLineFromMatrixToMatr(lineSource, T1, T2):
    """Draws a line between the position vectors of two transformation matrices.
    Furthermore it transforms the points so that the center of the model is in the origin"""
    T1_ = np.dot(T_vtk, T1)
    T2_ = np.dot(T_vtk, T2)
    lineSource.SetPoint1(T1_[0][3], T1_[1][3], T1_[2][3])
    lineSource.SetPoint2(T2_[0][3], T2_[1][3], T2_[2][3])

def setPokeMatrixWithMatr(actor, T):
    #set the origin in the center of the model
    T = np.dot(T_vtk, T)
    # leere Matrix anlegen (ist eine 4x4 Einheitsmatrix)
    poke = vtk.vtkMatrix4x4()

    # Matrix elementweise befuellen
    for row in range(4):
        for col in range(4):
            # Orientierung
            poke.SetElement(row, col, T[row, col])

    # Matrix an Actor übergeben
    actor.PokeMatrix(poke)

def setPokeMatrix(actor, r, T):
    '''
    Die Lage von 3D-Objekten in vtk wird mit einer sog. Poke-Matrix definiert.
    Sie ist eine 4x4-Matrix mit folgender Gestalt:

    [          ]
    [   T     r]
    [          ]
    [0  0  0  1]

    Soll die Lage eines Koerpers aktualisiert werden, muessen die Werte der
    Poke-Matrix geaendert werden. Das geschieht mit Hilfe dieser Funktion.
    '''

    # leere Matrix anlegen (ist eine 4x4 Einheitsmatrix)
    poke = vtk.vtkMatrix4x4()

    # Matrix elementweise befuellen
    for row in range(3):
        for col in range(3):

            # Orientierung
            poke.SetElement(row,col, T[row,col])

        # Positionsvektor
        poke.SetElement(row,3, r[row])


    # Matrix an Actor übergeben
    actor.PokeMatrix(poke)

def calcPosition(t):
    T = vtk.vtkMatrix4x4()
    #calc position
    x = np.sin((2*np.pi)/1 * t)
    r = np.array([x, 0, 0])
    return r