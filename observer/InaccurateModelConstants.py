import numpy as np
from ModelConstants import OriginalConstants as real_mc
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


import numpy as np

from enum import Enum
# import ModelConstants as real_mc
# import random

# this should be implemented by the ModelConstants.py. otherwise you cant compare both enum classes
# class ModelType(Enum):
#     EASY = 1
#     FRICTION = 2
#     CENTRIPETAL = 3
#     ROTORSPEED = 4
#     GYROMOMENT = 5
#     NO_SIMPLIFICATIONS = 6


# def scrambleParameters(lower_boundary, upper_boundary):
#     '''Randomly alters the parameters.
#     :arg lower_boundary: e.g. 0.8 ==> the lowest value of a variable will be 0.8 * real_value
#     :arg upper_boundary: eg. 1.2 ==> the highest value of a variable will be 1.2 * real_value'''
#     return
#     global l_p, l_h, l_c, m_p, m_c_max, m_c, m_m, r_m, J_m, d_p, d_e, d_l, T_f, T_b, K_f, K_b, K, K_m, g
#     randNrs = [None] * 15
#     for i in range(0, 15):
#         randNrs[i] = random.uniform(lower_boundary, upper_boundary)
#
#     l_p = randNrs[0] * real_mc.l_p
#     l_h = randNrs[1] * real_mc.l_h
#     l_c = randNrs[2] * real_mc.l_c
#     m_p = randNrs[3] * real_mc.m_p
#
#     m_c_max = 2 * l_h/l_c * m_p
#
#     m_c = randNrs[4] * real_mc.m_c
#
#     m_m = 0.2 * m_p
#     r_m = l_p/8
#     J_m = 0.5 * m_m * r_m**2
#
#     d_p = randNrs[5] * real_mc.d_p
#     d_e = randNrs[6] * real_mc.d_e
#     d_l = randNrs[7] * real_mc.d_l
#
#     T_f = randNrs[8] * real_mc.T_f
#     T_b = randNrs[9] * real_mc.T_b
#     K_f = randNrs[10] * real_mc.K_f
#     K_b = randNrs[11] * real_mc.K_b
#     K = randNrs[12] * real_mc.K
#     K_m = randNrs[13] * real_mc.K_m
#
#     g = randNrs[14] * real_mc.g
#     return
#
# l_p = 0.20650923728210005
# l_h = 0.7091297324913599
# l_c = 0.5489289621561383
# m_p = 0.45485408907269087
# m_c_max = 1.1751996369066482
# m_c = 0.92383968135585
# m_m = 0.09097081781453818
# r_m = 0.025813654660262506
# J_m = 3.0308964196542922e-05
# d_p = 0.04807318348101174
# d_e = 0.06054766415052718
# d_l = 0.27903080557480076
# T_f = 0.0872521566921722
# T_b = 0.1000972064028679
# K_f = 0.9571671464313205
# K_b = 1.171400481309869
# K = 1.134136435229437
# K_m = 0.10789375878571401
# g = 8.474358413885424



