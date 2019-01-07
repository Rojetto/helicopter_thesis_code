import numpy as np
import ModelConstants as mc
from HeliSimulation import getInertia
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.optimize import fsolve

validation_enabled = False
dl = 100
e = 25




def n_solve(functions,variables):
    func = lambda x: [ f(*x) for f in functions]
    return fsolve(func, variables)

def convV(x):
    vb = (x[0] - x[1]) / 2
    vf = (x[0] + x[1]) / 2
    return (vf, vb, x[2])

def calcInputs_numeric(dl, e, rad=True):
    """
    calculates inputs Vs, Vd, p for special case: ddlambda = 0, de = 0, dp = 0 with an numeric algorithm
    """
    if not rad:
        dl = dl * np.pi / 180
        e = e * np.pi / 180

    L1 = mc.l_p
    L2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
    L3 = mc.l_h
    L4 = mc.l_h
    J_p, J_e, J_l = getInertia([0, e, 0, 0, 0, 0, 0, 0], False)

    p_ana = np.arctan(mc.d_l * dl * L3 / ((J_e * np.cos(e) * np.sin(e) - L2 * np.cos(e)) * L4 * np.cos(e)))
    Vd_ana = J_p * np.cos(p_ana) * np.sin(p_ana) * np.cos(e)**2 * dl ** 2 / L1
    Vs_ana = (J_e * np.cos(e) * np.sin(e) * dl ** 2 - L2 * np.cos(e)) / (L3 * np.cos(p_ana))

    dp, de = 0, 0
    ddp = lambda Vs,Vd,p : (L1 / J_p) * Vd - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) * (de ** 2 - np.cos(e) ** 2 * dl ** 2)
    dde = lambda Vs,Vd,p : (L2 / J_e) * np.cos(e) + (L3 / J_e) * np.cos(p) * Vs - (mc.d_e / J_e) * de - np.cos(e) * np.sin(e) * dl ** 2
    ddlamb = lambda Vs,Vd,p : (L4 / J_l) * np.cos(e) * np.sin(p) * Vs - (mc.d_l / J_l) * dl

    Vs, Vd, p = n_solve([ddp, dde, ddlamb], [Vs_ana,Vd_ana, p_ana])

    if not rad:
        p = p * 180 / np.pi

    return [Vs, Vd, p]



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
    Vs2 = mc.d_l * dl / (L4 * np.cos(e) * np.sin(p))



    #test this function:
    dp, de = 0,0
    ddp = (L1 / J_p) * Vd - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) * (de ** 2 - np.cos(e) ** 2 * dl ** 2)
    dde = (L2 / J_e) * np.cos(e) + (L3 / J_e) * np.cos(p) * Vs - (mc.d_e / J_e) * de - np.cos(e) * np.sin(e) * dl ** 2
    ddlamb = (L4 / J_l) * np.cos(e) * np.sin(p) * Vs - (mc.d_l / J_l) * dl

    max_error = 2 / 180 * np.pi # rad/s^2
    #if (np.abs(ddp)>max_error) or (np.abs(dde)>max_error) or (np.abs(ddlamb)>max_error) :
    #    k = 180 / np.pi
    #    print(dl*k,p*k,ddp*k, dde*k, ddlamb*k) # should be zero
    #    return [np.nan]*3



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

def calcdde(dl,e):
    L2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
    J_e = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * mc.l_h ** 2

    return L2 * np.cos(e)/J_e - np.cos(e)* np.sin(e) *dl**2

def calcddp(dl,p):
    return -np.cos(p)*np.sin(p)* dl**2

def printValInput():
    steps = 100
    e = np.linspace(-50, 50, steps)
    dl = np.linspace(0, 90, steps)
    dlm, em = np.meshgrid(dl, e)
    Sol = np.empty((steps, steps, 3))
    for x in range(steps):
        for y in range(steps):
            # Sol[x, y] = np.array([calculateInputs_DL_E(dl[y], e[x], rad=False)])
            Sol[x, y] = np.array([calcInputs_numeric(dl[y], e[x], rad=False)])

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

def printValnoInput():
    steps = 100

    dl = np.linspace(0, 120, steps)
    p = np.linspace(-80, 80, steps)
    dlm, pm = np.meshgrid(p, dl)
    dde = np.empty((steps, steps))
    ddp = np.empty((steps, steps))
    for x in range(steps):
        for y in range(steps):
            dde[x, y] = np.array([calcdde(dl[x]/180 *np.pi, p[y]/180 *np.pi)])
            ddp[x, y] = np.array([calcddp(dl[x]/180 *np.pi, p[y]/180 *np.pi)])

    ddp = ddp * 180 / np.pi
    dde = dde * 180 / np.pi


    fig1 = plt.figure()
    ax = fig1.gca(projection='3d')
    surf = ax.plot_surface(pm, dlm, dde)
    ax.set(title='dde', xlabel='dlambda', ylabel='e', zlabel='dde')

    fig2 = plt.figure()
    ax = fig2.gca(projection='3d')
    surf = ax.plot_surface(pm, dlm, ddp)
    ax.set(title='ddp', xlabel='dlambda', ylabel='p', zlabel='ddp')



if __name__ == '__main__':
    printValInput()
    printValnoInput()

    plt.show()

