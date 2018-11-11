import numpy as np
import matplotlib.pyplot as plt

index = 0
chunk_size = 600  # 10 s with 60 FPS
current_size = chunk_size
ts_store = np.empty(current_size)
xs_store = np.empty((current_size, 6))
us_store = np.empty((current_size, 2))


def add_frame(t, x, u):
    global current_size, index

    if index == current_size:
        current_size += chunk_size
        ts_store.resize(current_size, refcheck=False)
        xs_store.resize((current_size, 6), refcheck=False)
        us_store.resize((current_size, 2), refcheck=False)

    ts_store[index] = t
    xs_store[index] = x
    us_store[index] = u

    index += 1


def finish():
    global index

    process(ts_store[:index], xs_store[:index], us_store[:index])

    index = 0


def process(ts, xs, us):
    # Your data processing code goes here

    plt.figure("Front and back rotor voltages")
    plt.plot(ts, us[:, 0])
    plt.plot(ts, us[:, 1])
    plt.grid()

    plt.figure("Joint angles (deg)")
    plt.plot(ts, xs[:, 0] / np.pi * 180.0)
    plt.plot(ts, xs[:, 1] / np.pi * 180.0)
    plt.plot(ts, xs[:, 2] / np.pi * 180.0)
    plt.grid()

    plt.show()
