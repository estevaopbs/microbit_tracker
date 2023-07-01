import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from joint_tracker import JointTracker
import matplotlib.patches as patches
from copy import deepcopy


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
        # self.ax.set_zlim(self.xyz_lim[2])
        # last_point = np.array([0, 0, 0])
        rot_90 = np.array(
            [
                [1, 0, 0],
                [0, np.cos(np.pi / 2), np.sin(np.pi / 2)],
                [0, -np.sin(np.pi / 2), np.cos(np.pi / 2)],
            ]
        )
        rot_90_neg = np.array(
            [
                [1, 0, 0],
                [0, np.cos(np.pi / 2), -np.sin(np.pi / 2)],
                [0, np.sin(np.pi / 2), np.cos(np.pi / 2)],
            ]
        )
        # norm_vec = self.joint_tracker.state.vectors[0] / np.linalg.norm(
        #    self.joint_tracker.state.vectors[0]
        # )
        # norm_rot_vec = np.dot(rtheta, norm_vec)
        # rectangle_origin = norm_rot_vec * self.articulation_diameter / 2
        # self.ax.add_patch(
        #    patches.Rectangle(
        #        rectangle_origin,
        #        self.lengths[0] - self.articulation_diameter,
        #        self.articulation_diameter,
        #        angle=self.joint_tracker.state.angles[0] + ((3 * np.pi) / 2),
        #        rotation_point=(0, 0),
        #    )
        # )
        articulation_origin = np.zeros(2)
        for vector, angle, length, n in zip(
            self.joint_tracker.state.vectors,
            self.joint_tracker.state.angles,
            self.lengths,
            range(len(self.lengths)),
        ):
            self.ax.add_patch(
                patches.Circle(
                    deepcopy(articulation_origin), radius=self.articulation_diameter / 2
                )
            )
            norm_vec = vector / np.linalg.norm(vector)
            norm_rot_vec_1 = np.dot(rot_90, norm_vec)
            rectangle_origin = norm_vec * self.articulation_diameter / 2
            rectangle_origin = articulation_origin + np.array(
                [rectangle_origin[1], rectangle_origin[2]]
            )

            self.ax.add_patch(
                patches.Rectangle(
                    rectangle_origin,
                    length - self.articulation_diameter,
                    self.articulation_diameter / 2,
                    angle=sum(
                        [
                            np.degrees(angle_)
                            for angle_ in self.joint_tracker.state.angles[:n]
                        ]
                    )
                    + np.degrees(angle)
                    - 90,
                    rotation_point=(
                        articulation_origin[0]
                        + norm_vec[0] * self.articulation_diameter,
                        articulation_origin[1]
                        + norm_vec[1] * self.articulation_diameter,
                    ),
                )
            )
            self.ax.add_patch(
                patches.Rectangle(
                    rectangle_origin,
                    length - self.articulation_diameter,
                    -self.articulation_diameter / 2,
                    angle=sum(
                        [
                            np.degrees(angle_)
                            for angle_ in self.joint_tracker.state.angles[:n]
                        ]
                    )
                    + np.degrees(angle)
                    - 90,
                    rotation_point=(
                        articulation_origin[0]
                        + norm_vec[0] * self.articulation_diameter,
                        articulation_origin[1]
                        + norm_vec[1] * self.articulation_diameter,
                    ),
                )
            )
            articulation_origin += np.array([norm_vec[1], norm_vec[2]]) * length
        # self.ax.quiver(*[0, 0], *self.joint_tracker.fixed_gravity, color="red")
        return

    def animate(self):
        self.animation = animation.FuncAnimation(
            self.fig, self.update, repeat=False, frames=self.frames
        )
        plt.show()
