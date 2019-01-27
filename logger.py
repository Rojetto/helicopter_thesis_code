import numpy as np
import matplotlib.pyplot as plt
import ModelConstants as mc
from helicontrollers.TimeVariantController import get_p_and_first_derivative
from helicontrollers.util import ModelType
from HeliSimulation import getInertia
from PyQt5 import QtWidgets
import os.path
import pickle
from matplotlib.figure import Figure

last_storage_directory = None
index = 0
chunk_size = 600  # 10 s with 60 FPS
current_size = chunk_size
ts_store = np.empty(current_size)
xs_store = np.empty((current_size, 8))
xs_estimated_store = np.empty((current_size, 8))
ys_noisy_output_store = np.empty((current_size, 5))
us_noisy_input_store = np.empty((current_size, 2))
cov_matrix_store = np.empty((current_size, 8, 8))
us_ff_store = np.empty((current_size, 2))
us_controller_store = np.empty((current_size, 2))
e_traj_store = np.empty((current_size, 5))
lambda_traj_store = np.empty((current_size, 5))
planner_travel_g = 0
planner_elevation_g = 0


class LoggingDataV2:
    def __init__(self, ts, xs, us_ff, us_controller, e_traj_and_derivatives, lambda_traj_and_derivatives,
                 xs_estimated_state, us_noisy_input, ys_noisy_output, cov_matrix):
        self.ts = ts
        self.xs = xs
        self.us_ff = us_ff
        self.us_controller = us_controller
        self.e_traj_and_derivatives = e_traj_and_derivatives
        self.lambda_traj_and_derivatives = lambda_traj_and_derivatives
        self.xs_estimated_state = xs_estimated_state
        self.ys_noisy_output = ys_noisy_output
        self.us_noisy_input = us_noisy_input
        self.cov_matrix = cov_matrix


class LoggingDataV1:
    def __init__(self, ts, xs, us_ff, us_controller, e_traj_and_derivatives, lambda_traj_and_derivatives):
        self.ts = ts
        self.xs = xs
        self.us_ff = us_ff
        self.us_controller = us_controller
        self.e_traj_and_derivatives = e_traj_and_derivatives
        self.lambda_traj_and_derivatives = lambda_traj_and_derivatives


def bundle_data():
    global index

    bundle = LoggingDataV2(ts_store[:index], xs_store[:index], us_ff_store[:index], us_controller_store[:index],
                           e_traj_store[:index], lambda_traj_store[:index], xs_estimated_store[:index],
                           us_noisy_input_store[:index], ys_noisy_output_store[:index], cov_matrix_store[:index])

    return bundle


def open_dialog_and_store():
    global last_storage_directory
    path, _ = QtWidgets.QFileDialog.getSaveFileName(None, directory=last_storage_directory,
                                                    filter="HeliControl data (*.hc2)")

    # User might have pressed "Cancel"
    if not path:
        return

    selected_dir = os.path.dirname(path)
    last_storage_directory = selected_dir
    bundle = bundle_data()

    with open(path, "wb") as file:
        pickle.dump(bundle, file)


def load_bundle(path):
    with open(path, "rb") as file:
        return pickle.load(file)


def add_planner(planner_travel, planner_elevation):
    global planner_travel_g, planner_elevation_g
    planner_travel_g = planner_travel
    planner_elevation_g = planner_elevation


def add_frame(t, x, u_ff, u_controller, e_traj_and_derivatives, lambda_traj_and_derivatives, x_estimated_state,
              us_noisy_input, ys_noisy_output, cov_matrix):
    global current_size, index

    if index == current_size:
        current_size += chunk_size
        ts_store.resize(current_size, refcheck=False)
        xs_store.resize((current_size, 8), refcheck=False)
        xs_estimated_store.resize((current_size, 8), refcheck=False)
        us_ff_store.resize((current_size, 2), refcheck=False)
        us_controller_store.resize((current_size, 2), refcheck=False)
        e_traj_store.resize((current_size, 5), refcheck=False)
        lambda_traj_store.resize((current_size, 5), refcheck=False)
        ys_noisy_output_store.resize((current_size, 5), refcheck=False)
        us_noisy_input_store.resize((current_size, 2), refcheck=False)
        cov_matrix_store.resize((current_size, 8, 8), refcheck=False)

    ts_store[index] = t
    xs_store[index] = x
    xs_estimated_store[index] = x_estimated_state
    us_ff_store[index] = u_ff
    us_controller_store[index] = u_controller
    ys_noisy_output_store[index] = ys_noisy_output
    us_noisy_input_store[index] = us_noisy_input
    e_traj_store[index] = e_traj_and_derivatives
    lambda_traj_store[index] = lambda_traj_and_derivatives
    cov_matrix_store[index] = cov_matrix

    index += 1


def reset():
    global index

    index = 0


def show_plots():
    print("Start Plotting")
    bundle = bundle_data()
    process(bundle)


def process(bundle: LoggingDataV2):
    ts = bundle.ts
    xs = bundle.xs
    us_ff = bundle.us_ff
    us_controller = bundle.us_controller
    e_traj_and_derivatives = bundle.e_traj_and_derivatives
    lambda_traj_and_derivatives = bundle.lambda_traj_and_derivatives
    xs_estimated_state = bundle.xs_estimated_state
    us_noisy_input = bundle.us_noisy_input
    ys_noisy_output = bundle.ys_noisy_output
    cov_matrix = bundle.cov_matrix

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

    plotMoments(bundle)

    plotBasics(bundle)

    plotValidation(bundle)

    plotInputs(bundle)

    plotObserver(bundle)

    plt.show()


def custom_figure(num=None,  # autoincrement if None, else integer from 1-N
                  figsize=None,  # defaults to rc figure.figsize
                  dpi=None,  # defaults to rc figure.dpi
                  facecolor=None,  # defaults to rc figure.facecolor
                  edgecolor=None,  # defaults to rc figure.edgecolor
                  frameon=True,
                  FigureClass=Figure,
                  clear=False,
                  **kwargs):
    fig = plt.figure(**kwargs, num=num,figsize=figsize,dpi=dpi,facecolor=facecolor,edgecolor=edgecolor,frameon=frameon,FigureClass=FigureClass,clear=clear)
    plt.grid()
    fig.canvas.mpl_connect('close_event', handle_close)

    return fig


def handle_close(evt):
    plt.close("all")


def plotValidation(bundle):
    # figures to show the influence of CENTRIPETAL forces
    ts = bundle.ts
    xs = bundle.xs
    us_controller = bundle.us_controller

    Vf, Vb = us_controller[:, 0], us_controller[:, 1]
    v_s = Vf + Vb
    v_d = Vf - Vb

    p, e, lamb, dp, de, dlamb, f_speed, b_speed = xs[:, 0], xs[:, 1], xs[:, 2], \
                                                  xs[:, 3], xs[:, 4], xs[:, 5], \
                                                  xs[:, 6], xs[:, 7]

    L_1 = mc.l_p
    L_2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
    L_3 = mc.l_h
    L_4 = mc.l_h

    J_p = 2 * mc.m_p * mc.l_p ** 2
    J_e = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * mc.l_h ** 2
    J_l = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * (mc.l_h ** 2 + mc.l_p ** 2)

    ddp = (L_1 / J_p) * v_d - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) * (de ** 2 - np.cos(e) ** 2 * dlamb ** 2)
    dde = (L_2 / J_e) * np.cos(e) + (L_3 / J_e) * np.cos(p) * v_s - (mc.d_e / J_e) * de - np.cos(e) * np.sin(e) * dlamb ** 2
    ddlamb = (L_4 / J_l) * np.cos(e) * np.sin(p) * v_s - (mc.d_l / J_l) * dlamb

    fig = custom_figure("Joint acceleration (deg/s^2)")
    plt.plot(ts, ddp / np.pi * 180.0)
    plt.plot(ts, dde / np.pi * 180.0)
    plt.plot(ts, ddlamb / np.pi * 180.0)
    plt.legend(['ddp', 'dde', 'ddlamb'])

def plotMoments(bundle):
    # figures to show the influence of all moments
    ts = bundle.ts
    xs = bundle.xs

    L_1 = mc.l_p
    L_2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
    L_3 = mc.l_h
    L_4 = mc.l_h

    p, e, lamb, dp, de, dlamb, f_speed, b_speed = xs[:, 0], xs[:, 1], xs[:, 2], \
                                                  xs[:, 3], xs[:, 4], xs[:, 5], \
                                                  xs[:, 6], xs[:, 7]

    J_p = 2 * mc.m_p * mc.l_p ** 2
    J_e = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * (mc.l_h ** 2 + mc.l_p ** 2 * np.sin(p) ** 2)
    J_l = mc.m_c * mc.l_c ** 2 * np.cos(e) ** 2 + 2 * mc.m_p * (
            (mc.l_h * np.cos(e)) ** 2 + (mc.l_p * np.sin(p) * np.cos(e)) ** 2 + (mc.l_p * np.cos(p)) ** 2)

    p_gyro = (np.cos(p) * de * mc.J_m * (b_speed - f_speed) + np.sin(p) * np.cos(e) * mc.J_m * (
            f_speed - b_speed)) / J_p / np.pi * 180.0
    p_cor = np.cos(p) * np.sin(p) * (de ** 2 - np.cos(e) ** 2 * dlamb ** 2) / np.pi * 180.0
    p_input = (L_1 * mc.K / J_p) * (f_speed - b_speed) / np.pi * 180.0
    p_fric = -(mc.d_p / J_p) * dp / np.pi * 180.0
    ddp = p_input + p_fric + p_cor + p_gyro

    e_motor = np.sin(p) * mc.K_m * (f_speed - b_speed) / J_e / np.pi * 180.0
    e_gyro = (np.cos(p) * dp * mc.J_m * (f_speed - b_speed) + np.sin(e) * np.cos(p) * dlamb * mc.J_m * (
            b_speed - f_speed)) / J_e / np.pi * 180.0
    e_cor = - np.cos(e) * np.sin(e) * dlamb ** 2 / np.pi * 180.0
    e_input = L_3 * mc.K * np.cos(p) * (f_speed + b_speed) / J_e / np.pi * 180.0
    e_fric = - (mc.d_e / J_e) * de / np.pi * 180.0
    e_grav = (L_2 / J_e) * np.cos(e) / np.pi * 180.0
    dde = e_grav + e_input + e_fric + e_cor + e_gyro + e_motor

    ddlamb = 1 / J_l * (
            L_4 * mc.K * np.cos(e) * np.sin(p) * (f_speed + b_speed) - mc.d_l * dlamb + np.cos(e) * np.cos(
        p) * mc.K_m * (b_speed - f_speed) \
            + np.sin(p) * np.cos(e) * dp * mc.J_m * (f_speed - b_speed) + np.sin(p) * np.cos(
        e) * dlamb * mc.J_m * (f_speed - b_speed))

    l_imput = L_4 * mc.K * np.cos(e) * np.sin(p) * (f_speed + b_speed) / J_l / np.pi * 180.0
    l_fric = - mc.d_l * dlamb / J_l / np.pi * 180.0
    l_motor = np.cos(e) * np.cos(p) * mc.K_m * (b_speed - f_speed) / J_l / np.pi * 180.0
    l_gyro = (np.sin(p) * np.cos(e) * dp * mc.J_m * (f_speed - b_speed) + np.sin(p) * np.cos(e) * dlamb * mc.J_m * (
            f_speed - b_speed)) / J_l / np.pi * 180.0
    ddlamb = l_imput + l_fric + l_motor + l_gyro

    fig = custom_figure("Influence of torques at ddp (deg/s^2)")
    plt.plot(ts, ddp)
    plt.plot(ts, p_cor)
    plt.plot(ts, p_input)
    plt.plot(ts, p_fric)
    plt.plot(ts, p_gyro)
    plt.legend(['sum', 'centripetal', 'input', 'friction', 'gyroscope'])

    fig = custom_figure("Influence of torques at dde (deg/s^2)")
    plt.plot(ts, dde)
    plt.plot(ts, e_cor)
    plt.plot(ts, e_input)
    plt.plot(ts, e_fric)
    plt.plot(ts, e_grav)
    plt.plot(ts, e_gyro)
    plt.plot(ts, e_motor)
    plt.legend(['sum', 'centripetal', 'input', 'friction', 'gravitation', 'gyroscope', 'motor torque'])

    fig = custom_figure("Influence of torques at ddlambda (deg/s^2)")
    plt.plot(ts, ddlamb)
    plt.plot(ts, l_imput)
    plt.plot(ts, l_fric)
    plt.plot(ts, l_gyro)
    plt.plot(ts, l_motor)
    plt.legend(['sum', 'input', 'friction', 'gyroscope', 'motor torque'])


def plotInputs(bundle):
    ts = bundle.ts
    xs = bundle.xs
    us_ff = bundle.us_ff
    us_controller = bundle.us_controller
    e_traj_and_derivatives = bundle.e_traj_and_derivatives
    lambda_traj_and_derivatives = bundle.lambda_traj_and_derivatives

    fig = custom_figure("Total front and back rotor voltages")
    plt.plot(ts, us_ff[:, 0] + us_controller[:, 0])
    plt.plot(ts, us_ff[:, 1] + us_controller[:, 1])
    plt.legend(['Vf', 'Vb'])

    fig = custom_figure("Feed forward and controller output")
    plt.plot(ts, us_ff[:, 0])
    plt.plot(ts, us_controller[:, 0])
    plt.plot(ts, us_ff[:, 1])
    plt.plot(ts, us_controller[:, 1])
    plt.legend(['Vf feed-forward', 'Vf controller', 'Vb feed-forward', 'Vb controller'])

    fig = custom_figure("Rotorspeed")
    plt.plot(ts, xs[:, 6])
    plt.plot(ts, xs[:, 7])
    plt.legend(['f', 'b'])


def plotBasics(bundle):
    ts = bundle.ts
    xs = bundle.xs
    us_ff = bundle.us_ff
    us_controller = bundle.us_controller
    e_traj_and_derivatives = bundle.e_traj_and_derivatives
    lambda_traj_and_derivatives = bundle.lambda_traj_and_derivatives

    fig = custom_figure("Joint angles (deg)")
    p_traj = np.array([get_p_and_first_derivative(ModelType.CENTRIPETAL, e, l)[0] for (e, l) in
                       zip(e_traj_and_derivatives, lambda_traj_and_derivatives)])
    plt.plot(ts, xs[:, 0] / np.pi * 180.0, label="p")
    plt.plot(ts, p_traj / np.pi * 180.0, label="p_traj")
    plt.plot(ts, xs[:, 1] / np.pi * 180.0, label="e")
    plt.plot(ts, e_traj_and_derivatives[:, 0] / np.pi * 180.0, label="e_traj")
    plt.plot(ts, xs[:, 2] / np.pi * 180.0, label="lambda")
    plt.plot(ts, lambda_traj_and_derivatives[:, 0] / np.pi * 180.0, label="lambda_traj")
    plt.legend()

    fig = custom_figure("Joint velocity (deg/s)")
    plt.plot(ts, xs[:, 3] / np.pi * 180.0)
    plt.plot(ts, xs[:, 4] / np.pi * 180.0)
    plt.plot(ts, xs[:, 5] / np.pi * 180.0)
    plt.legend(['dp', 'de', 'dlambda'])


def plotObserver(bundle):
    ts = bundle.ts
    xs = bundle.xs
    us_ff = bundle.us_ff
    us_controller = bundle.us_controller
    e_traj_and_derivatives = bundle.e_traj_and_derivatives
    lambda_traj_and_derivatives = bundle.lambda_traj_and_derivatives
    xs_estimated_state = bundle.xs_estimated_state
    us_noisy_input = bundle.us_noisy_input
    ys_noisy_output = bundle.ys_noisy_output
    cov_matrix = bundle.cov_matrix

    def rad2deg(rad):
        return rad * 180 / np.pi

    def rad_squared2def_squared(rad_squared):
        return rad_squared * (180 / np.pi)**2

    fig = custom_figure("Covariance Matrix (var(p), var(e), var(lambda))")
    plt.plot(ts, rad_squared2def_squared(cov_matrix[:, 0, 0]), label=r"var($ \varphi $) (°^2)")
    plt.plot(ts, rad_squared2def_squared(cov_matrix[:, 1, 1]), label=r"var($ \varepsilon $) (°^2)")
    plt.plot(ts, rad_squared2def_squared(cov_matrix[:, 2, 2]), label=r"var($ \lambda $) (°^2)")
    plt.xlabel("Time (s)")
    plt.ylabel("Variance Angles")
    plt.legend()

    fig = custom_figure("Covariance Matrix (var(dp), var(de), var(dlambda))")
    plt.plot(ts, rad_squared2def_squared(cov_matrix[:, 3, 3]), label=r"var($ \dfrac{d \varphi }{d t} $) ((°/s)^2)")
    plt.plot(ts, rad_squared2def_squared(cov_matrix[:, 4, 4]), label=r"var($ \dfrac{d \varepsilon }{d t} $) ((°/s)^2)")
    plt.plot(ts, rad_squared2def_squared(cov_matrix[:, 5, 5]), label=r"var($ \dfrac{d \lambda }{d t} $) ((°/s)^2)")
    plt.xlabel("Time (s)")
    plt.ylabel("Variance Angle Velocities")
    plt.legend()

    fig = custom_figure("Covariance Matrix (var(f), var(b))")
    plt.plot(ts, cov_matrix[:, 6, 6], label=r"var(f) (1/s)^2")
    plt.plot(ts, cov_matrix[:, 7, 7], label=r"var(b) (1/s)^2")
    plt.xlabel("Time (s)")
    plt.ylabel("Variance of rotational frequencies")
    plt.legend()

    fig = custom_figure("Estimated state of observer (p, e, lambda)")
    plt.plot(ts, rad2deg(xs_estimated_state[:, 0]), label=r"$ \hat \varphi $  (°)")
    plt.plot(ts, rad2deg(xs_estimated_state[:, 1]), label=r"$ \hat \varepsilon $ (°)")
    plt.plot(ts, rad2deg(xs_estimated_state[:, 2]), label=r"$ \hat \lambda $ (°)")
    plt.plot(ts, rad2deg(xs[:, 0]), "-.", label=r"$ \varphi $ from simulation (°)")
    plt.plot(ts, rad2deg(xs[:, 1]), "-.", label=r"$ \varepsilon $ from simulation (°)")
    plt.plot(ts, rad2deg(xs[:, 2]), "-.", label=r"$ \lambda $ from simulation (°)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle variables")
    plt.legend()

    fig = custom_figure("Estimated state of observer (dp, de, dlambda)")
    plt.plot(ts, rad2deg(xs_estimated_state[:, 3]), label=r"$ \hat \dfrac{d \varphi }{d t} $ (°/s)")
    plt.plot(ts, rad2deg(xs_estimated_state[:, 4]), label=r"$ \hat \dfrac{d \varepsilon }{d t} $  (°/s)")
    plt.plot(ts, rad2deg(xs_estimated_state[:, 5]), label=r"$ \hat \dfrac{d \lambda }{d t} $  (°/s)")
    plt.plot(ts, rad2deg(xs[:, 3]), "-.", label=r"$ \dfrac{d \varphi }{d t} $ from simulation (°/s)")
    plt.plot(ts, rad2deg(xs[:, 4]), "-.", label=r"$ \dfrac{d \varepsilon }{d t} $ from simulation (°/s)")
    plt.plot(ts, rad2deg(xs[:, 5]), "-.", label=r"$ \dfrac{d \lambda }{d t} $ from simulation (°/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle Velocities")
    plt.legend()

    fig = custom_figure("Estimated state of observer (f, b)")
    plt.plot(ts, xs_estimated_state[:, 6], label=r"$ \hat f $ (1/s)")
    plt.plot(ts, xs_estimated_state[:, 7], label=r"$ \hat b $ (1/s)")
    plt.plot(ts, xs[:, 6], "-.", label=r"f from simulation (1/s)")
    plt.plot(ts, xs[:, 7], "-.", label=r"b from simulation (1/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Motor rotational frequencies")
    plt.legend()

    # plot difference between real and observed value
    # estimate error = real value - estimate value
    fig = custom_figure("Estimate error of observer (p, e, lambda)")
    plt.plot(ts, rad2deg(xs[:, 0] - xs_estimated_state[:, 0]), label=r"$ \varphi $ - $ \hat \varphi $ (°)")
    plt.plot(ts, rad2deg(xs[:, 1] - xs_estimated_state[:, 1]), label=r"$ \varepsilon $ - $ \hat  \varepsilon $ (°)")
    plt.plot(ts, rad2deg(xs[:, 2] - xs_estimated_state[:, 2]), label=r"$ \lambda $ - $ \hat  \lambda $ (°)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle estimate error")
    plt.legend()

    fig = custom_figure("Estimate error of observer (dp, de, dlambda)")
    plt.plot(ts, rad2deg(xs[:, 3] - xs_estimated_state[:, 3]), label=r"$ \dfrac{d \varphi }{d t} -  \hat  \dfrac{d \varphi }{d t} $ (°/s)")
    plt.plot(ts, rad2deg(xs[:, 4] - xs_estimated_state[:, 4]), label=r"$ \dfrac{d \varepsilon }{d t}  - \hat   \dfrac{d \varepsilon }{d t} $ (°/s)")
    plt.plot(ts, rad2deg(xs[:, 5] - xs_estimated_state[:, 5]), label=r"$ \dfrac{d \lambda }{d t}  - \hat   \dfrac{d \lambda }{d t} $ (°/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle velocity estimate error")
    plt.legend()

    fig = custom_figure("Estimated state error (f, b)")
    plt.plot(ts, xs[:, 6] - xs_estimated_state[:, 6], label=r"$ f - \hat f$ (1/s)")
    plt.plot(ts, xs[:, 7] - xs_estimated_state[:, 7], label=r"$ b - \hat b$ (1/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Rotational frequency estimate error")
    plt.legend()


    fig = custom_figure("Input of system with and without noise")
    ax1 = fig.add_subplot(111)
    ax1.plot(ts, us_noisy_input[:, 0], label=r"Vf with noise (V)")
    ax1.plot(ts, us_noisy_input[:, 1], label=r"Vb with noise (V)")
    ax1.plot(ts, us_ff[:, 0] + us_controller[:, 0], label=r"Vf without noise (V)")
    ax1.plot(ts, us_ff[:, 1] + us_controller[:, 1], label=r"Vf without noise (V)")
    plt.xlabel("Time (s)")
    plt.ylabel("Motor voltages")
    ax1.legend(loc=1)

    fig = custom_figure("Output of system with and without noise (p, e, lambda)")
    ax1 = fig.add_subplot(111)
    ax1.plot(ts, rad2deg(ys_noisy_output[:, 0]), label=r"$ \varphi $ with noise (°)")
    ax1.plot(ts, rad2deg(ys_noisy_output[:, 1]), label=r"$ \varepsilon $ with noise (°)")
    ax1.plot(ts, rad2deg(ys_noisy_output[:, 2]), label=r"$ \lambda $ with noise (°)")
    ax1.plot(ts, rad2deg(xs[:, 0]), label=r"$ \varphi $ without noise (°)")
    ax1.plot(ts, rad2deg(xs[:, 1]), label=r"$ \varepsilon $ without noise (°)")
    ax1.plot(ts, rad2deg(xs[:, 2]), label=r"$ \lambda $ without noise (°)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle output values")
    ax1.legend(loc=2)

    fig = custom_figure("Output of system with and without noise (f, b)")
    ax1 = fig.add_subplot(111)
    ax1.plot(ts, ys_noisy_output[:, 3], label=r"f with noise (1/s)")
    ax1.plot(ts, ys_noisy_output[:, 4], label=r"b with noise (1/s)")
    ax1.plot(ts, xs[:, 6], label=r"f without noise (1/s)")
    ax1.plot(ts, xs[:, 7], label=r"b without noise (1/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Motor rotational frequencies")
    ax1.legend(loc=2)

    # fig = custom_figure("Noise of observed signal")
    # plt.plot(ts, xs_estimated_state[:, 0] - xs[:, 0], label="p_est - p")
    # plt.plot(ts, xs_estimated_state[:, 1] - xs[:, 1], label="e_est - e")
    # plt.plot(ts, xs_estimated_state[:, 2] - xs[:, 2], label="lambda_est - lambda")
    # plt.legend()

    # Calculate the variance of the kalman filter signals for verifying correct noise generation
    vf_var = np.var(us_noisy_input[:, 0] - (us_ff[:, 0] + us_controller[:, 0]))
    vb_var = np.var(us_noisy_input[:, 1] - (us_ff[:, 1] + us_controller[:, 1]))
    p_var = np.var(ys_noisy_output[:, 0] - xs[:, 0])
    e_var = np.var(ys_noisy_output[:, 1] - xs[:, 1])
    lamb_var = np.var(ys_noisy_output[:, 2] - xs[:, 2])
    print("Variance of Vf is " + str(vf_var))
    print("   ... the standard deviation is " + str(np.sqrt(vf_var)))
    print("Variance of Vb is " + str(vb_var))
    print("   ... the standard deviation is " + str(np.sqrt(vb_var)))
    print("Variance of p is " + str(p_var))
    print("   ... in degree the standard deviation is " + str(np.sqrt(p_var) * 180 / np.pi))
    print("Variance of e is " + str(e_var))
    print("   ... in degree the standard deviation is " + str(np.sqrt(e_var) * 180 / np.pi))
    print("Variance of lambda is " + str(lamb_var))
    print("   ... in degree the standard deviation is " + str(np.sqrt(lamb_var) * 180 / np.pi))
