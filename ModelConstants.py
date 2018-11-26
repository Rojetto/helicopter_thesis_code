import numpy as np

# l_a = ? (not given in Fig. 7)
l_p = 0.18 #m
l_h = 0.65
l_c = 0.52
m_p = 0.5 * 0.771 #kg
m_c_max = 2 * l_h/l_c * m_p
m_c = 0.9  # arbitrary, but has to be smaller than m_c_max

d_p = 0.048  # dissipation coefficient in Nms
d_e = 0.053
d_l = 0.274

# PT1 charakteristics for ROTOR SPEED
T_f = 0.1
T_b = 0.1
K_f = 1
K_b = 1
K = 1

g = 9.81  # m/s^2

vtk_l_a = 10*l_h
vtk_l_h = 10*l_h
vtk_l_p = 10*l_p
vtk_l_c = 10*l_c

#Static Theta values (for initialization)
#s_theta1 = 0
#s_theta2 = 0
#s_theta3 = 0

T_vtk = np.array([[1, 0, 0, 0],
                  [0, 0, 1, 0],
                  [0, -1, 0, 0],
                  [0, 0, 0, 1]])

#Alignment and position of geometrical joints relative to T_i_minus_one
T_j1 = np.array([[1, 0, 0, 0],
                 [0, 0, 1, 0],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]])

T_j2 = np.array([[1, 0, 0, 0],
                 [0, 0, -1, 0],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]])

T_j3 = np.array([[1, 0, 0, 0],
                 [0, 0, -1, 0],
                 [0, 1, 0, vtk_l_h],
                 [0, 0, 0, 1]])

#endeffector alignment and position relative to T_i
T_e1 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, -vtk_l_p],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

T_e2 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, vtk_l_p],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

T_e3 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, -vtk_l_c],
                 [0, 0, 0, 1]])