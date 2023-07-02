import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from joint_tracker import JointTracker
import matplotlib.patches as patches
from copy import deepcopy


SEGMENTS_COLOR = "#1F77B4"
SEGMENTS_LINEWIDTH = 2

ARTICULATION_EXTERNAL_C0LOR = "#FF7F0E"
ARTICULATION_EXTERNAL_LINEWIDTH = 2

ARTICULATION_INTERNAL_SOLID_COLOR = "#FF7F0E"
ARTICULATION_INTERNAL_SOLID_RADIUS = 0.4

ARTICULATION_INTERNAL_SHAPE_COLOR = "#FFFFFF"
ARTICULATION_INTERNAL_SHAPE_RADIUS = 0.2
ARTICULATION_INTERNAL_SHAPE_LINEWIDTH = 0.05


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
        self.ax = self.fig.add_subplot(111)
        self.xyz_lim = xyz_lim
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
        self.ax.set_xlim(self.xyz_lim[0])
        self.ax.set_ylim(self.xyz_lim[1])

        articulation_origin = np.zeros(2)
        for vector, angle, length, n in zip(
            self.joint_tracker.state.vectors,
            self.joint_tracker.state.angles,
            self.lengths,
            range(len(self.lengths)),
        ):
            self.ax.add_patch(
                patches.Circle(
                    deepcopy(articulation_origin),
                    radius=self.articulation_diameter / 2,
                    # animated=True,
                    linewidth=ARTICULATION_EXTERNAL_LINEWIDTH,
                    color=ARTICULATION_EXTERNAL_C0LOR,
                    edgecolor=ARTICULATION_EXTERNAL_C0LOR,
                    fill=False,
                )
            )
            # self.ax.add_patch(
            #    patches.Circle(
            #        deepcopy(articulation_origin),
            #        radius=(self.articulation_diameter / 2)
            #        * ARTICULATION_INTERNAL_SOLID_RADIUS,
            #        animated=True,
            #        color=ARTICULATION_INTERNAL_SOLID_COLOR,
            #    )
            # )
            # self.ax.add_patch(
            #    patches.Circle(
            #        deepcopy(articulation_origin),
            #        radius=self.articulation_diameter
            #        / 2
            #        * ARTICULATION_INTERNAL_SHAPE_RADIUS,
            #        animated=True,
            #        linewidth=ARTICULATION_INTERNAL_SHAPE_LINEWIDTH,
            #        color=ARTICULATION_INTERNAL_SHAPE_COLOR,
            #        fill=False,
            #    )
            # )
            norm_vec = xyz_to_yz(vector / np.linalg.norm(vector))
            norm_rot_vec = rotate_vector(norm_vec, -np.pi / 2)
            vertices = [
                ((norm_rot_vec + norm_vec) * self.articulation_diameter / 2)
                + articulation_origin
            ]
            line_dir = deepcopy(norm_vec)
            for i in range(3):
                _length = (
                    length - self.articulation_diameter
                    if i % 2 == 0
                    else self.articulation_diameter
                )
                vertices.append(vertices[-1] + line_dir * _length)
                line_dir = rotate_vector(line_dir, np.pi / 2)
            for j in range(4):
                self.ax.plot(
                    [vertices[j][0], vertices[j - 1][0]],
                    [vertices[j][1], vertices[j - 1][1]],
                    linewidth=SEGMENTS_LINEWIDTH,
                    color=SEGMENTS_COLOR,
                )
            articulation_origin += norm_vec * length
        return

    def animate(self):
        self.animation = animation.FuncAnimation(
            self.fig, self.update, repeat=False, frames=self.frames
        )
        plt.show()
