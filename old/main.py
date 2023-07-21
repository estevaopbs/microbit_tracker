import matplotlib.pyplot as plt
import matplotlib.animation as animation
from kaspersmicrobit import KaspersMicrobit
import numpy as np

# Microbit 0
ADDRESS = "C3:B0:42:88:FE:07"

# Microbit 1
ADDRESS1 = "F6:8C:51:58:97:63"

# Plot
# x
COLOR_X = "red"
LABEL_X = "x"
STYLE_X = "dashed"
LINEWIDTH_X = 0.5
MARKERSIZE_X = 0.5

# y
COLOR_Y = "green"
LABEL_Y = "y"
STYLE_Y = "dashed"
LINEWIDTH_Y = 0.5
MARKERSIZE_Y = 0.5

# z
COLOR_Z = "blue"
LABEL_Z = "z"
STYLE_Z = "dashed"
LINEWIDTH_Z = 0.5
MARKERSIZE_Z = 0.5

# TIME
CLOCK = 0.01
EXHIBITION_TIME = 20
TIMEAXIS = np.arange(-EXHIBITION_TIME, 0, CLOCK)
ANIMATION_TIME = 15


class Acc:
    def __init__(self, microbit):
        self.microbit = microbit
        self.x = np.zeros(int(EXHIBITION_TIME / CLOCK))
        self.y = np.zeros(int(EXHIBITION_TIME / CLOCK))
        self.z = np.zeros(int(EXHIBITION_TIME / CLOCK))

    def update(self):
        data = self.microbit.accelerometer.read()
        self.x = np.roll(self.x, -1)
        self.y = np.roll(self.y, -1)
        self.z = np.roll(self.z, -1)
        self.x[-1] = data.x
        self.y[-1] = data.y
        self.z[-1] = data.z

    def __str__(self):
        return f"Acc(x={self.x}, y={self.y}, z={self.z})"


def update_array(data_array, value):
    data_array = np.roll(data_array, -1)
    data_array[-1] = value


def update_frame(frame):
    acc.update()
    ax.clear()
    ax1.clear()
    ax.plot(
        TIMEAXIS,
        acc.x,
        linestyle=STYLE_X,
        color=COLOR_X,
        linewidth=LINEWIDTH_X,
        markersize=MARKERSIZE_X,
        label=LABEL_X,
    )
    ax.plot(
        TIMEAXIS,
        acc.y,
        linestyle=STYLE_Y,
        color=COLOR_Y,
        linewidth=LINEWIDTH_Y,
        markersize=MARKERSIZE_Y,
        label=LABEL_Y,
    )
    ax.plot(
        TIMEAXIS,
        acc.z,
        linestyle=STYLE_Z,
        color=COLOR_Z,
        linewidth=LINEWIDTH_Z,
        markersize=MARKERSIZE_Z,
        label=LABEL_Z,
    )
    ax.legend()
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Aceleração Gravitacional (Gaus)")
    acc1.update()
    ax1.plot(
        TIMEAXIS,
        acc1.x,
        linestyle=STYLE_X,
        color=COLOR_X,
        linewidth=LINEWIDTH_X,
        markersize=MARKERSIZE_X,
        label=LABEL_X,
    )
    ax1.plot(
        TIMEAXIS,
        acc1.y,
        linestyle=STYLE_Y,
        color=COLOR_Y,
        linewidth=LINEWIDTH_Y,
        markersize=MARKERSIZE_Y,
        label=LABEL_Y,
    )
    ax1.plot(
        TIMEAXIS,
        acc1.z,
        linestyle=STYLE_Z,
        color=COLOR_Z,
        linewidth=LINEWIDTH_Z,
        markersize=MARKERSIZE_Z,
        label=LABEL_Z,
    )
    ax1.legend()
    ax1.set_xlabel("Tempo (s)")
    ax1.set_ylabel("Aceleração Gravitacional (Gaus)")
    return None


if __name__ == "__main__":
    fig, ax = plt.subplots(2, 1)
    ax = plt.subplot(211)
    ax1 = plt.subplot(212)
    microbit = KaspersMicrobit(ADDRESS)
    microbit.connect()
    acc = Acc(microbit)
    microbit1 = KaspersMicrobit(ADDRESS1)
    microbit1.connect()
    acc1 = Acc(microbit1)

    # MP4
    # ani = animation.FuncAnimation(
    #    fig, update_frame, interval=CLOCK, repeat=False,
    #    frames=int(ANIMATION_TIME/CLOCK))
    # plt.rcParams['animation.ffmpeg_path'] = 'C:\\FFmpeg\\bin\\ffmpeg.exe'
    # writer = animation.writers['ffmpeg'](fps=1/CLOCK, bitrate=1800)
    # ani.save('animation.mp4', writer=writer)

    # SHOW
    ani = animation.FuncAnimation(
        fig,
        update_frame,
        interval=CLOCK,
        repeat=False,
        frames=int(ANIMATION_TIME / CLOCK),
    )
    plt.show()

    microbit.disconnect()
    microbit1.disconnect()
