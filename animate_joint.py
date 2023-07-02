import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from joint_tracker import JointTracker
import matplotlib.patches as patches
from copy import deepcopy, copy
import time


SEGMENTS_COLOR = "#1F77B4"
SEGMENTS_LINEWIDTH = 2

ARTICULATION_EXTERNAL_EDGE_C0LOR = "#000000"
ARTICULATION_EXTERNAL_SHAPE_C0LOR = "#000000"
ARTICULATION_EXTERNAL_LINEWIDTH = 0.5

ARTICULATION_INTERNAL_SOLID_COLOR = "#000000"
ARTICULATION_INTERNAL_SOLID_RADIUS = 0.5

TIMESPAN = 10


def xyz_to_yz(vec):
    return np.array([vec[1], vec[2]])


def rotate_vector(vec, angle):
    rot = np.array(
        [
            [np.cos(angle), np.sin(angle)],
            [-np.sin(angle), np.cos(angle)],
        ]
    )
    return np.dot(rot, vec)


class JointAnimation:
    def __init__(self, joint_tracker: JointTracker, lenghts, frames, xyz_lim):
        self.joint_tracker = joint_tracker
        self.lengths = lenghts
        self.articulation_diameter = 0.3
        self.frames = frames
        self.frame = 0
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(121)
        self.ax_angles = self.fig.add_subplot(322)
        self.ax_speed = self.fig.add_subplot(324)
        self.ax_acc = self.fig.add_subplot(326)
        self.xyz_lim = xyz_lim
        self.states = []
        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle()
        self.ax.set_aspect("equal")
        plt.subplots_adjust(hspace=0.3)

        # self.text = self.ax.text2D(
        #    0.85,
        #    0,
        #    "",
        #    transform=self.ax.transAxes,
        #    fontsize=14,
        #    verticalalignment="top",
        # )

    def update(self, frame):
        self.frame = frame
        self.joint_tracker.update()
        self.ax.clear()
        self.ax.set_title("Manipulador Planar")
        self.ax_angles.clear()
        self.ax_angles.set_title("Ângulos")
        self.ax_speed.clear()
        self.ax_speed.set_title("Velocidades Angulares")
        self.ax_acc.clear()
        self.ax_acc.set_title("Acelerações Angulares")
        self.ax.set_xlim(self.xyz_lim[0])
        self.ax.set_ylim(self.xyz_lim[1])

        self.ax_angles.set_ylim([np.degrees(-6.5), np.degrees(6.5)])
        self.ax_angles.set_xlim([-TIMESPAN, 0])

        # self.ax_speed.set_ylim([-7.5, 7.5])
        self.ax_speed.set_xlim([-TIMESPAN, 0])

        # self.ax_acc.set_ylim([-15, 15])
        self.ax_acc.set_xlim([-TIMESPAN, 0])

        articulation_origin = np.zeros(2)
        for vector, angle, length, n in zip(
            self.joint_tracker.state.vectors,
            self.joint_tracker.state.angles,
            self.lengths,
            range(len(self.lengths)),
        ):
            norm_vec = xyz_to_yz(vector / np.linalg.norm(vector))
            norm_rot_vec = rotate_vector(norm_vec, -np.pi / 2)
            vertices = [
                (norm_rot_vec * self.articulation_diameter / 2 * 0.8)
                + articulation_origin
            ]
            line_dir = deepcopy(norm_vec)
            for i in range(3):
                _length = length if i % 2 == 0 else self.articulation_diameter * 0.8
                vertices.append(vertices[-1] + line_dir * _length)
                line_dir = rotate_vector(line_dir, np.pi / 2)
            for j in range(4):
                self.ax.plot(
                    [vertices[j][0], vertices[j - 1][0]],
                    [vertices[j][1], vertices[j - 1][1]],
                    linewidth=SEGMENTS_LINEWIDTH,
                    color=SEGMENTS_COLOR,
                )
            self.ax.add_patch(
                patches.Circle(
                    deepcopy(articulation_origin),
                    radius=self.articulation_diameter / 2,
                    linewidth=ARTICULATION_EXTERNAL_LINEWIDTH,
                    facecolor="#FFFFFF",
                    edgecolor=ARTICULATION_EXTERNAL_EDGE_C0LOR,
                    fill=True,
                    zorder=1000,
                )
            )
            self.ax.add_patch(
                patches.Circle(
                    deepcopy(articulation_origin),
                    radius=(self.articulation_diameter / 2)
                    * ARTICULATION_INTERNAL_SOLID_RADIUS,
                    color=ARTICULATION_INTERNAL_SOLID_COLOR,
                    zorder=1000,
                )
            )
            articulation_origin += norm_vec * length
        now = self.joint_tracker.state.upd_time
        for n, state in enumerate(self.states):
            if now - state.upd_time < TIMESPAN:
                self.states = self.states[n:]
                break
        self.states.append(self.joint_tracker.state)
        x = []
        y_angles = [[] for _ in self.lengths]
        y_speed = [[] for _ in self.lengths]
        y_acc = [[] for _ in self.lengths]
        for state in self.states:
            x.append(state.upd_time - now)
            for n, articulation in enumerate(y_angles):
                articulation.append(np.degrees(state.angles[n]))
            for n, articulation in enumerate(y_speed):
                articulation.append(state.speeds[n])
            for n, articulation in enumerate(y_acc):
                articulation.append(state.accs[n])
        for n, y in enumerate(y_angles):
            self.ax_angles.plot(x, y, label=f"Articulação {n}")
        for n, y in enumerate(y_speed):
            self.ax_speed.plot(x, y, label=f"Articulação {n}")
        for n, y in enumerate(y_acc):
            self.ax_acc.plot(x, y, label=f"Articulação {n}")
        self.ax_angles.legend(loc="upper right")
        self.ax_angles.set_xlabel("Tempo (s)")
        self.ax_angles.set_ylabel("Ângulo (Graus)")
        self.ax_acc.legend(loc="upper right")
        self.ax_acc.set_xlabel("Tempo (s)")
        self.ax_acc.set_ylabel("Aceleração Ângular (rad/s²)")
        self.ax_speed.legend(loc="upper right")
        self.ax_speed.set_xlabel("Tempo (s)")
        self.ax_speed.set_ylabel("Velocidade Ângular (rad/s)")

        return

    def animate(self):
        self.animation = animation.FuncAnimation(
            self.fig, self.update, repeat=False, frames=self.frames
        )
        plt.show()
