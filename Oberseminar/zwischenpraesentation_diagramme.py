import matplotlib
matplotlib.use('Qt5Agg')
matplotlib.rcParams["figure.dpi"] = 96
matplotlib.rcParams["figure.figsize"] = (8, 4)
from matplotlib.pyplot import *
from logger import LoggingDataV1
import pickle
from numpy import pi


def load_bundle(file_name):
    full_path = f"C:\\dev\\HeliControl\\Daten\\Zwischenpraesentation\\{file_name}.hc1"
    bundle = None
    with open(full_path, "rb") as file:
        bundle = pickle.load(file)
    return bundle


step_responses = [
    ("LQR Sprung in $e$", load_bundle("lqr_step_e")),
    ("LQR Sprung in $\\lambda$", load_bundle("lqr_step_lambda")),
    ("PID Sprung in $e$", load_bundle("pid_step_e")),
    ("PID Sprung in $\\lambda$", load_bundle("pid_step_lambda"))
]

disturb_responses = [
    ("LQR Störung auf $e$", load_bundle("lqr_disturb_e")),
    ("LQR Störung auf $\\lambda$", load_bundle("lqr_disturb_lambda")),
    ("PID Störung auf $e$", load_bundle("pid_disturb_e")),
    ("PID Störung auf $\\lambda$", load_bundle("pid_disturb_lambda")),
]

for step_response in step_responses:
    title_str, bundle = step_response
    figure(title_str.replace("$", "").replace("\\", ""))
    title(title_str)
    xlabel("$t$ [s]")
    ylabel("Winkel [°]")
    plot(bundle.ts, 180 / pi * bundle.xs[:, 0], label="$p$")
    plot(bundle.ts, 180 / pi * bundle.xs[:, 1], label="$e$")
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
    plot(bundle.ts, 180 / pi * bundle.xs[:, 0], label="$p$")
    plot(bundle.ts, 180 / pi * bundle.xs[:, 1], label="$e$")
    plot(bundle.ts, 180 / pi * bundle.xs[:, 2], label="$\\lambda$")
    axvline(1, color="black", label="Zeit der Störung", linestyle="dotted")
    legend()
    grid()
    tight_layout()

b: LoggingDataV1 = load_bundle("trajectory_feedback_lin_gyro_model")
figure("Folgeregler")
title("Folgeregelung mittels partieller Linearisierung")
xlabel("$t$ [s]")
ylabel("Winkel [°]")
plot(b.ts, 180 / pi * b.xs[:, 0], label="$p$")
plot(b.ts, 180 / pi * b.xs[:, 1], label="$e$")
plot(b.ts, 180 / pi * b.xs[:, 2], label="$\\lambda$")
plot(b.ts, 180 / pi * b.e_traj_and_derivatives[:, 0], label="Trajektorie $e$ und $\\lambda$", linestyle="dashed", color="black")
legend()
grid()
tight_layout()


show()
