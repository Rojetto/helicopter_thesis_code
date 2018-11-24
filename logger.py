import numpy as np
import matplotlib.pyplot as plt
import ModelConstants as mc

index = 0
chunk_size = 600  # 10 s with 60 FPS
current_size = chunk_size
ts_store = np.empty(current_size)
xs_store = np.empty((current_size, 6))
us_store = np.empty((current_size, 2))
e_traj_store = np.empty((current_size, 5))
lambda_traj_store = np.empty((current_size, 5))
planner_travel_g = 0
planner_elevation_g = 0

def add_planner(planner_travel, planner_elevation):
    global planner_travel_g, planner_elevation_g
    planner_travel_g = planner_travel
    planner_elevation_g = planner_elevation


def add_frame(t, x, u, e_traj_and_derivatives, lambda_traj_and_derivatives):
    global current_size, index

    if index == current_size:
        current_size += chunk_size
        ts_store.resize(current_size, refcheck=False)
        xs_store.resize((current_size, 6), refcheck=False)
        us_store.resize((current_size, 2), refcheck=False)
        e_traj_store.resize((current_size, 5), refcheck=False)
        lambda_traj_store.resize((current_size, 5), refcheck=False)

    ts_store[index] = t
    xs_store[index] = x
    us_store[index] = u
    e_traj_store[index] = e_traj_and_derivatives
    lambda_traj_store[index] = lambda_traj_and_derivatives

    index += 1


def finish():
    global index

    process(ts_store[:index], xs_store[:index], us_store[:index], e_traj_store[:index], lambda_traj_store[:index])

    index = 0


def process(ts, xs, us, e_traj_and_derivatives, lambda_traj_and_derivatives):
    # Your data processing code goes here

    # fig = plt.figure()
    # ax1 = fig.add_subplot(211)
    # ax2 = fig.add_subplot(212)
    # trav = planner_travel_g.eval_vec(ts)[:, 0]
    # elev = planner_elevation_g.eval_vec(ts)[:, 0]
    # ax1.plot(ts, trav)
    # ax2.plot(ts, elev)
    # ax1.grid()
    # plt.show()

    # plt.figure("Front and back rotor voltages")
    # plt.plot(ts, us[:, 0])
    # plt.plot(ts, us[:, 1])
    # plt.legend(['Vf','Vb'])
    # plt.grid()
    #
    plt.figure("Joint angles (deg)")
    plt.plot(ts, xs[:, 0] / np.pi * 180.0)
    plt.plot(ts, xs[:, 1] / np.pi * 180.0)
    plt.plot(ts, e_traj_and_derivatives[:, 0] / np.pi * 180.0)
    plt.plot(ts, xs[:, 2] / np.pi * 180.0)
    plt.plot(ts, lambda_traj_and_derivatives[:, 0] / np.pi * 180.0)
    plt.legend(['p', 'e', 'e_traj', 'lambda', 'lambda_traj'])
    plt.grid()
    #
    # plt.figure("Joint velocity (deg/s)")
    # plt.plot(ts, xs[:, 3] / np.pi * 180.0)
    # plt.plot(ts, xs[:, 4] / np.pi * 180.0)
    # plt.plot(ts, xs[:, 5] / np.pi * 180.0)
    # plt.legend(['dp','de','dlambda'])
    # plt.grid()
    #
    # # figures to show the influence of CENTRIPETAL forces
    #
    # L_1 = mc.l_p
    # L_2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
    # L_3 = mc.l_h
    # L_4 = mc.l_h
    #
    # J_p = 2 * mc.m_p * mc.l_p ** 2
    # J_e = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * mc.l_h ** 2
    # J_l = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * (mc.l_h ** 2 + mc.l_p ** 2)
    #
    #
    # V_s = us[:,0] + us[:,1]
    # V_d = us[:,0] - us[:,1]
    # p, e = xs[:,0], xs[:,1]
    # dp, de, dlamb = xs[:,3], xs[:,4], xs[:,5]
    #
    # p_cor = np.cos(p) * np.sin(p) * (de ** 2 - np.cos(e) ** 2 * dlamb ** 2) / np.pi * 180.0
    # p_input = (L_1 / J_p) * V_d / np.pi * 180.0
    # p_fric = -(mc.d_p / J_p) * dp / np.pi * 180.0
    # ddp = p_input + p_fric + p_cor
    #
    # e_cor = - np.cos(e) * np.sin(e) * dlamb ** 2 / np.pi * 180.0
    # e_input = (L_3 / J_e) * np.cos(p) * V_s / np.pi * 180.0
    # e_fric = - (mc.d_e / J_e) * de / np.pi * 180.0
    # e_grav = (L_2 / J_e) * np.cos(e) / np.pi * 180.0
    # dde = e_grav + e_input + e_fric + e_cor
    #
    # plt.figure("Influence of centripetal forces at ddp (deg/s^2)")
    # plt.plot(ts, ddp)
    # plt.plot(ts, p_cor)
    # plt.plot(ts, p_input)
    # plt.plot(ts, p_fric)
    # plt.legend(['sum','centripetal','input','friction'])
    # plt.grid()
    #
    # plt.figure("Influence of centripetal forces at dde (deg/s^2)")
    # plt.plot(ts, dde)
    # plt.plot(ts, e_cor)
    # plt.plot(ts, e_input)
    # plt.plot(ts, e_fric)
    # plt.plot(ts, e_grav)
    # plt.legend(['sum','centripetal','input','friction','gravitation'])
    # plt.grid()
    #
    plt.show()
