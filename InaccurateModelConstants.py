import numpy as np

from enum import Enum
import ModelConstants as real_mc
import random

# this should be implemented by the ModelConstants.py. otherwise you cant compare both enum classes
# class ModelType(Enum):
#     EASY = 1
#     FRICTION = 2
#     CENTRIPETAL = 3
#     ROTORSPEED = 4
#     GYROMOMENT = 5
#     NO_SIMPLIFICATIONS = 6


def scrambleParameters(lower_boundary, upper_boundary):
    '''Randomly alters the parameters.
    :arg lower_boundary: e.g. 0.8 ==> the lowest value of a variable will be 0.8 * real_value
    :arg upper_boundary: eg. 1.2 ==> the highest value of a variable will be 1.2 * real_value'''
    global l_p, l_h, l_c, m_p, m_c_max, m_c, m_m, r_m, J_m, d_p, d_e, d_l, T_f, T_b, K_f, K_b, K, K_m, g
    randNrs = [None] * 15
    for i in range(0, 15):
        randNrs[i] = random.uniform(lower_boundary, upper_boundary)

    l_p = randNrs[0] * real_mc.l_p
    l_h = randNrs[1] * real_mc.l_h
    l_c = randNrs[2] * real_mc.l_c
    m_p = randNrs[3] * real_mc.m_p

    m_c_max = 2 * l_h/l_c * m_p

    m_c = randNrs[4] * real_mc.m_c

    m_m = 0.2 * m_p
    r_m = l_p/8
    J_m = 0.5 * m_m * r_m**2

    d_p = randNrs[5] * real_mc.d_p
    d_e = randNrs[6] * real_mc.d_e
    d_l = randNrs[7] * real_mc.d_l

    T_f = randNrs[8] * real_mc.T_f
    T_b = randNrs[9] * real_mc.T_b
    K_f = randNrs[10] * real_mc.K_f
    K_b = randNrs[11] * real_mc.K_b
    K = randNrs[12] * real_mc.K
    K_m = randNrs[13] * real_mc.K_m

    g = randNrs[14] * real_mc.g
    return

# l_a = ? (not given in Fig. 7)
l_p = real_mc.l_p #m
l_h = real_mc.l_h
l_c = real_mc.l_c
m_p = real_mc.m_p #kg
m_c_max = 2 * l_h/l_c * m_p
m_c = real_mc.m_c  # arbitrary, but has to be smaller than m_c_max

# motor characteristics
m_m = 0.2 * m_p
r_m = l_p/8
J_m = 0.5 * m_m * r_m**2


d_p = real_mc.d_p  # dissipation coefficient in Nms
d_e = real_mc.d_e
d_l = real_mc.d_l

# PT1 charakteristics for ROTOR SPEED
T_f = real_mc.T_f
T_b = real_mc.T_b
K_f = real_mc.K_f
K_b = real_mc.K_b
K = real_mc.K
K_m = real_mc.K_m

g = real_mc.g  # m/s^2

# These values are not used by the kalman filter and also shouldn't be used by it
# vtk_l_a = 10*l_h
# vtk_l_h = 10*l_h
# vtk_l_p = 10*l_p
# vtk_l_c = 10*l_c
#
# #Static Theta values (for initialization)
# #s_theta1 = 0
# #s_theta2 = 0
# #s_theta3 = 0
#
# T_vtk = np.array([[1, 0, 0, 0],
#                   [0, 0, 1, 0],
#                   [0, -1, 0, 0],
#                   [0, 0, 0, 1]])
#
# #Alignment and position of geometrical joints relative to T_i_minus_one
# T_j1 = np.array([[1, 0, 0, 0],
#                  [0, 0, 1, 0],
#                  [0, -1, 0, 0],
#                  [0, 0, 0, 1]])
#
# T_j2 = np.array([[1, 0, 0, 0],
#                  [0, 0, -1, 0],
#                  [0, 1, 0, 0],
#                  [0, 0, 0, 1]])
#
# T_j3 = np.array([[1, 0, 0, 0],
#                  [0, 0, -1, 0],
#                  [0, 1, 0, vtk_l_h],
#                  [0, 0, 0, 1]])
#
# #endeffector alignment and position relative to T_i
# T_e1 = np.array([[1, 0, 0, 0],
#                  [0, 1, 0, -vtk_l_p],
#                  [0, 0, 1, 0],
#                  [0, 0, 0, 1]])
#
# T_e2 = np.array([[1, 0, 0, 0],
#                  [0, 1, 0, vtk_l_p],
#                  [0, 0, 1, 0],
#                  [0, 0, 0, 1]])
#
# T_e3 = np.array([[1, 0, 0, 0],
#                  [0, 1, 0, 0],
#                  [0, 0, 1, -vtk_l_c],
#                  [0, 0, 0, 1]])