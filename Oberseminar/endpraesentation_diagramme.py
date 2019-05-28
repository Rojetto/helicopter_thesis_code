import matplotlib
matplotlib.use('Qt5Agg')
matplotlib.rcParams["figure.dpi"] = 96
matplotlib.rcParams["figure.figsize"] = (8, 4)
from matplotlib.pyplot import *
from logger import LoggingDataV2
import pickle
from numpy import pi
import ModelConstants as mc
from helicontrollers.util import L1, L2, L3, L4, Je, Jl, Jp


def load_bundle(file_name, end_time=None):
    full_path = f"C:\\dev\\HeliControl\\Daten\\Praesentation\\{file_name}.hc2"
    with open(full_path, "rb") as file:
        bundle: LoggingDataV2 = pickle.load(file)

        if end_time is not None:
            indices = bundle.ts <= end_time
            bundle.ts = bundle.ts[indices]
            bundle.xs = bundle.xs[indices, :]
            bundle.lambda_traj_and_derivatives = bundle.lambda_traj_and_derivatives[indices, :]
            bundle.e_traj_and_derivatives = bundle.e_traj_and_derivatives[indices, :]
            # And other entries, if required

        return bundle


step_responses = [
    ("LQR Sprung in $\\varepsilon$", load_bundle("lqr_step_e", 3)),
    ("LQR Sprung in $\\lambda$", load_bundle("lqr_step_lambda")),
    ("LQR große Distanz", load_bundle("lqr_big_movement", 6)),
    ("PID Sprung in $\\varepsilon$", load_bundle("pid_step_e", 2)),
    ("PID Sprung in $\\lambda$", load_bundle("pid_step_lambda", 6)),
    ("PID große Distanz", load_bundle("pid_big_movement", 6))
]

disturb_responses = [
    ("LQR Störung auf $\\varepsilon$", load_bundle("lqr_disturb_e", 6)),
    ("LQR Störung auf $\\lambda$", load_bundle("lqr_disturb_lambda", 8)),
    ("PID Störung auf $\\varepsilon$", load_bundle("pid_disturb_e")),
    ("PID Störung auf $\\lambda$", load_bundle("pid_disturb_lambda")),
]

for step_response in step_responses:
    title_str, bundle = step_response
    figure(title_str.replace("$", "").replace("\\", ""))
    title(title_str)
    xlabel("$t$ [s]")
    ylabel("Winkel [°]")
    plot(bundle.ts, 180 / pi * bundle.xs[:, 0], label="$\\varphi$")
    plot(bundle.ts, 180 / pi * bundle.xs[:, 1], label="$\\varepsilon$")
    plot(bundle.ts, 180 / pi * bundle.xs[:, 2], label="$\\lambda$")
    legend()
    grid()
    tight_layout()

for disturb_response in disturb_responses:
    title_str, bundle = disturb_response
    figure(title_str.replace("$", "").replace("\\", ""))
    title(title_str)
    xlabel("$t$ [s]")
    ylabel("Winkel [°]")
    plot(bundle.ts, 180 / pi * bundle.xs[:, 0], label="$\\varphi$")
    plot(bundle.ts, 180 / pi * bundle.xs[:, 1], label="$\\varepsilon$")
    plot(bundle.ts, 180 / pi * bundle.xs[:, 2], label="$\\lambda$")
    axvline(1, color="black", label="Zeit der Störung", linestyle="dotted")
    legend()
    grid()
    tight_layout()


def calc_pd(xs, e_traj, lambda_traj):
    p = xs[:, 0]
    e = xs[:, 1]
    l = xs[:, 2]
    dp = xs[:, 3]
    de = xs[:, 4]
    dl = xs[:, 5]
    wf = xs[:, 6]
    wb = xs[:, 7]

    ke = [20, 6]
    kl = [1, 0.8]
    kp = [150, 20]

    mue = mc.d_e
    mul = mc.d_l
    mup = mc.d_p

    v1 = e_traj[:, 2] - ke[1] * (de - e_traj[:, 1]) - ke[0] * (e - e_traj[:, 0])
    v2 = lambda_traj[:, 2] - kl[1] * (dl - lambda_traj[:, 1]) - kl[0] * (l - lambda_traj[:, 0])

    u1v = 1 / L3 * (Je * v1 - L2 * np.cos(e) + mue * de + Je * np.cos(e) * np.sin(e) * dl ** 2)
    u2v = 1 / (L4 * np.cos(e)) * (Jl * v2 + mul * dl)

    pd = np.arctan(u2v / u1v)

    return pd


b: LoggingDataV2 = load_bundle("trajectory_30deg_6s", 10)
figure("Folgeregler 6s")
title("Folgeregelung bei 6 s Trajektorie")
xlabel("$t$ [s]")
ylabel("Winkel [°]")
plot(b.ts, 180 / pi * b.xs[:, 0], label="$\\varphi$")
plot(b.ts, 180 / pi * b.xs[:, 1], label="$\\varepsilon$")
plot(b.ts, 180 / pi * b.xs[:, 2], label="$\\lambda$")
plot(b.ts, 180 / pi * calc_pd(b.xs, b.e_traj_and_derivatives, b.lambda_traj_and_derivatives), label="Reglervorgabe $\\varphi^*$", linestyle="dotted", color="black")
plot(b.ts, 180 / pi * b.e_traj_and_derivatives[:, 0], label="Trajektorie $\\varepsilon$ und $\\lambda$", linestyle="dashed", color="black")
legend(loc='lower right')
grid()
tight_layout()

figure("Folgefehler 6s")
title("Folgefehler bei 6 s Trajektorie")
xlabel("$t$ [s]")
ylabel("Winkel [°]")
plot(b.ts, 180 / pi * (b.xs[:, 1] - b.e_traj_and_derivatives[:, 0]), label="$\\varepsilon - \\varepsilon_d$")
plot(b.ts, 180 / pi * (b.xs[:, 2] - b.e_traj_and_derivatives[:, 0]), label="$\\lambda - \\lambda_d$")
legend()
grid()
tight_layout()

b: LoggingDataV2 = load_bundle("trajectory_30deg_4s", 10)
figure("Folgeregler 4s mit Reaktionsmoment")
title("Folgeregelung bei 4 s Trajektorie (mit Reaktionsmoment)")
xlabel("$t$ [s]")
ylabel("Winkel [°]")
plot(b.ts, 180 / pi * b.xs[:, 0], label="$\\varphi$")
plot(b.ts, 180 / pi * b.xs[:, 1], label="$\\varepsilon$")
plot(b.ts, 180 / pi * b.xs[:, 2], label="$\\lambda$")
plot(b.ts, 180 / pi * calc_pd(b.xs, b.e_traj_and_derivatives, b.lambda_traj_and_derivatives), label="Reglervorgabe $\\varphi^*$", linestyle="dotted", color="black")
plot(b.ts, 180 / pi * b.e_traj_and_derivatives[:, 0], label="Trajektorie $\\varepsilon$ und $\\lambda$", linestyle="dashed", color="black")
legend()
grid()
tight_layout()

figure("Folgefehler 4s mit Reaktionsmoment")
title("Folgefehler bei 4 s Trajektorie (mit Reaktionsmoment)")
xlabel("$t$ [s]")
ylabel("Winkel [°]")
plot(b.ts, 180 / pi * (b.xs[:, 1] - b.e_traj_and_derivatives[:, 0]), label="$\\varepsilon - \\varepsilon_d$")
plot(b.ts, 180 / pi * (b.xs[:, 2] - b.e_traj_and_derivatives[:, 0]), label="$\\lambda - \\lambda_d$")
legend()
grid()
tight_layout()


b: LoggingDataV2 = load_bundle("trajectory_30deg_4s_no_reaction", 10)
figure("Folgeregler 4s ohne Reaktionsmoment")
title("Folgeregelung bei 4 s Trajektorie (ohne Reaktionsmoment)")
xlabel("$t$ [s]")
ylabel("Winkel [°]")
plot(b.ts, 180 / pi * b.xs[:, 0], label="$\\varphi$")
plot(b.ts, 180 / pi * b.xs[:, 1], label="$\\varepsilon$")
plot(b.ts, 180 / pi * b.xs[:, 2], label="$\\lambda$")
plot(b.ts, 180 / pi * calc_pd(b.xs, b.e_traj_and_derivatives, b.lambda_traj_and_derivatives), label="Reglervorgabe $\\varphi^*$", linestyle="dotted", color="black")
plot(b.ts, 180 / pi * b.e_traj_and_derivatives[:, 0], label="Trajektorie $\\varepsilon$ und $\\lambda$", linestyle="dashed", color="black")
legend()
grid()
tight_layout()

figure("Folgefehler 4s ohne Reaktionsmoment")
title("Folgefehler bei 4 s Trajektorie (ohne Reaktionsmoment)")
xlabel("$t$ [s]")
ylabel("Winkel [°]")
plot(b.ts, 180 / pi * (b.xs[:, 1] - b.e_traj_and_derivatives[:, 0]), label="$\\varepsilon - \\varepsilon_d$")
plot(b.ts, 180 / pi * (b.xs[:, 2] - b.e_traj_and_derivatives[:, 0]), label="$\\lambda - \\lambda_d$")
legend()
grid()
tight_layout()


show()