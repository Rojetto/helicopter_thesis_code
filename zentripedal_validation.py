import numpy as np
import ModelConstants as mc
from HeliSimulation import getInertia
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm
validation_enabled = True
dl = 54
e = 25

def convV(x):
    vb = (x[0] - x[1]) / 2
    vf = (x[0] + x[1]) / 2
    return (vf, vb, x[2])


def calculateInputs_DL_E(dl, e, rad=True):
    """
    calculates inputs Vs, Vd, p for special case: ddlambda = 0, de = 0, dp = 0
    """

    if not rad:
        dl = dl * np.pi / 180
        e = e * np.pi / 180

    L1 = mc.l_p
    L2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
    L3 = mc.l_h
    L4 = mc.l_h
    J_p, J_e, J_l = getInertia([0, e, 0, 0, 0, 0, 0, 0], False)

    p = np.arctan(mc.d_l * dl * L3 / ((J_e * np.cos(e) * np.sin(e) - L2 * np.cos(e)) * L4 * np.cos(e)))

    Vd = J_p * np.cos(p) * np.sin(p) * np.cos(e)**2 * dl ** 2 / L1

    Vs = (J_e * np.cos(e) * np.sin(e) * dl ** 2 - L2 * np.cos(e)) / (L3 * np.cos(p))
    #Vs = mc.d_l * dl / (L4 * np.cos(e) * np.sin(p))



    #test this function:
    dp, de = 0,0
    ddp = (L1 / J_p) * Vd - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) * (de ** 2 - np.cos(e) ** 2 * dl ** 2)
    dde = (L2 / J_e) * np.cos(e) + (L3 / J_e) * np.cos(p) * Vs - (mc.d_e / J_e) * de - np.cos(e) * np.sin(e) * dl ** 2
    ddlamb = (L4 / J_l) * np.cos(e) * np.sin(p) * Vs - (mc.d_l / J_l) * dl

    max_error = 2 / 180 * np.pi # rad/s^2
    if (np.abs(ddp)>max_error) or (np.abs(dde)>max_error) or (np.abs(ddlamb)>max_error) :
        k = 180 / np.pi
        print(dl*k,p*k,ddp*k, dde*k, ddlamb*k) # should be zero
        return [np.nan]*3

    #k = 180 / np.pi
    #print(dl*k,p*k,ddp*k, dde*k, ddlamb*k) # should be zero


    if not rad:
        p = p * 180 / np.pi

    return [Vs, Vd, p]


def calculateInputs_E_P(e, p, rad=True):
    """
    calculates inputs Vs, Vd, dlambda for special case: ddlambda = 0, de = 0, dp = 0
    """

    if not rad:
        e = e * np.pi / 180
        p = p * np.pi / 180

    L1 = mc.l_p
    L2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
    L3 = mc.l_h
    L4 = mc.l_h
    J_p, J_e, J_l = getInertia([p, e, 0, 0, 0, 0, 0, 0], True)

    P = - L3 * np.cos(p) * mc.d_l ** 2 / (J_e * L4 ** 2 * np.cos(e) ** 2 * np.sin(e) * np.sin(p))
    Q = - L2 * mc.d_l ** 2 / (J_e * L4 ** 2 * np.cos(e) * np.sin(e) * np.sin(p))

    z = P ** 2 / 4 - Q

    if z < 0:
        return [(np.NaN, np.NaN, np.NaN), (np.NaN, np.NaN, np.NaN)]

    Vs1 = - P / 2 + np.sqrt(z)
    Vs2 = - P / 2 - np.sqrt(z)

    dl1 = Vs1 * L4 * np.cos(e) * np.sin(e) / mc.d_l
    dl2 = Vs2 * L4 * np.cos(e) * np.sin(e) / mc.d_l

    Vd1 = J_p * np.cos(p) * np.sin(p) * np.cos(e) * dl1 ** 2 / L1
    Vd2 = J_p * np.cos(p) * np.sin(p) * np.cos(e) * dl2 ** 2 / L1

    # return [(-0, -0, -0), (-0, -0, 0)]
    return [(Vs1, dl1, Vd1), (Vs2, dl2, Vd2)]


if __name__ == '__main__':

    steps = 100
    e = np.linspace(-90, 90, steps)
    dl = np.linspace(0, 90, steps)
    dlm, em = np.meshgrid(dl, e)
    Sol = np.empty((steps, steps, 3))
    for x in range(steps):
        for y in range(steps):
            Sol[x, y] = np.array([calculateInputs_DL_E(dl[y], e[x], rad=False)])

    print(Sol.shape)

    fig1 = plt.figure()
    ax = fig1.gca(projection='3d')
    surf = ax.plot_surface(em, dlm, Sol[:, :, 0])
    ax.set(title='Vs', xlabel='e', ylabel='dlamda', zlabel='Vs')

    fig2 = plt.figure()
    ax = fig2.gca(projection='3d')
    surf = ax.plot_surface(em, dlm, Sol[:, :, 1])
    ax.set(title='Vd', xlabel='e', ylabel='dlamda', zlabel='Vd')

    fig3 = plt.figure()
    ax = fig3.gca(projection='3d')
    surf = ax.plot_surface(em, dlm, Sol[:, :, 2])
    ax.set(title='p', xlabel='e', ylabel='dlamda', zlabel='p')

    res = calculateInputs_DL_E(dl=54, e=25, rad=False)
    print(convV(res)[0:2])

    plt.show()

    # steps = 100
    #     e = np.linspace(10, 80, steps)
    #     e = np.linspace(100, 170, steps)
    #     p = np.linspace(10, 80, steps)
    #     em, pm = np.meshgrid(p, e)
    #     Sol = np.empty((steps, steps, 2, 3))
    #     for x in range(steps):
    #         for y in range(steps):
    #             Sol[x, y] = np.array([calculateInputs_E_P(e[x], p[y], rad=False)])
    #
    #     print(e.shape, p.shape, Sol.shape)
    #
    #     for solnr in [0, 1]:
    #         fig1 = plt.figure()
    #         ax = fig1.gca(projection='3d')
    #         surf = ax.plot_surface(pm, em, Sol[:, :, solnr, 0])
    #         ax.set(title='Vs', xlabel='e', ylabel='p', zlabel='Vs')
    #
    #         fig2 = plt.figure()
    #         ax = fig2.gca(projection='3d')
    #         surf = ax.plot_surface(pm, em, Sol[:, :, solnr, 2])
    #         ax.set(title='Vd', xlabel='e', ylabel='p', zlabel='Vd')
    #
    #         fig3 = plt.figure()
    #         ax = fig3.gca(projection='3d')
    #         surf = ax.plot_surface(pm, em, Sol[:, :, solnr, 1])
    #         ax.set(title='velocity of lambda', xlabel='e', ylabel='p', zlabel='dlambda')
    #
    #         plt.show()
